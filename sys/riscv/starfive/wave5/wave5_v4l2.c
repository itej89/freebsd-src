/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * /dev/video0 -- a V4L2 memory-to-memory decoder node for the Wave511.
 *
 * FreeBSD has no V4L2 m2m framework, so this implements the ioctl ABI
 * directly. The point is that ffmpeg needs no patching: its v4l2_m2m decoder
 * depends only on "linux_videodev2_h sem_timedwait", with no Linux gate, and
 * multimedia/v4l_compat installs that header on FreeBSD. VLC and mpv decode
 * through libavcodec, so they inherit hardware decode with no changes at all.
 *
 * The ioctl numbers work out because v4l_compat's videodev2.h resolves
 * _IOR/_IOW/_IOWR through FreeBSD's <sys/ioccom.h>, not Linux's -- verified,
 * not assumed (VIDIOC_QUERYCAP is 0x40685600, direction bit 0x40000000 =
 * IOC_OUT). Had it used Linux's encoding, every _IOR would have arrived with
 * the direction inverted. The same headers are imported under uapi/ so the
 * struct layouts are shared by construction.
 *
 * The sequence below is what ffmpeg actually performs, read out of
 * libavcodec/v4l2_m2m.c rather than guessed:
 *
 *   probe:  open, QUERYCAP, ENUM_FMT (both queues), TRY_FMT, close
 *   setup:  open, QUERYCAP, S_FMT output, S_FMT capture,
 *           REQBUFS+QUERYBUF+mmap+STREAMON on output
 *   decode: QBUF output; then on the first frame G_FMT capture,
 *           REQBUFS+mmap capture, STREAMON capture, and DQBUF capture
 *
 * Note it defers the capture queue until decoding starts, which is why
 * SOURCE_CHANGE events are not needed here: G_FMT on capture is where the
 * real decoded geometry is reported, and by then a packet has been fed.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/poll.h>
#include <sys/selinfo.h>
#include <sys/queue.h>
#include <sys/uio.h>
#include <sys/fcntl.h>
#include <sys/filio.h>

#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>

#include "wave5_osal.h"
#include "wave5-vpu.h"
#include "wave5-vpuapi.h"
#include "wave5-regdefine.h"
#include "wave5.h"

#include "wave5_softc.h"

#include <linux/videodev2.h>

/*
 * Tripwire. These layouts are shared with userspace; if an import ever pulls
 * in a header whose structs differ, every ioctl silently reads the wrong
 * fields. The values are what the installed FreeBSD header produces.
 */
CTASSERT(sizeof(struct v4l2_buffer) == 88);
CTASSERT(sizeof(struct v4l2_format) == 208);
CTASSERT(sizeof(struct v4l2_capability) == 104);

#define	WAVE5_MAX_BUFS		32
#define	WAVE5_BITSTREAM_SIZE	(4 * 1024 * 1024)
#define	WAVE5_OUTBUF_SIZE	(1024 * 1024)

/* mmap offsets: each buffer gets a distinct page-aligned window. */
#define	WAVE5_OFF_OUTPUT	0x00000000UL
#define	WAVE5_OFF_CAPTURE	0x10000000UL
#define	WAVE5_OFF_STRIDE	0x00400000UL

struct wave5_buf {
	struct vpu_buf		vb;
	uint32_t		bytesused;
	uint32_t		offset;		/* mmap cookie */
	int			index;
	bool			queued;		/* owned by the driver */
	bool			done;		/* ready for DQBUF */
	TAILQ_ENTRY(wave5_buf)	link;
};

struct wave5_queue {
	struct wave5_buf	bufs[WAVE5_MAX_BUFS];
	int			count;
	bool			streaming;
	uint32_t		pixelformat;
	uint32_t		width, height;
	uint32_t		sizeimage;
	uint32_t		bytesperline;
	TAILQ_HEAD(, wave5_buf)	done_q;
};

struct wave5_fh {
	struct wave5_softc	*sc;
	struct vpu_device	*vdev;
	struct vpu_instance	*inst;
	struct mtx		lock;
	struct selinfo		rsel;
	struct wave5_queue	out;
	struct wave5_queue	cap;

	struct vpu_buf		bitstream;
	size_t			bs_used;

	int			non_linear;	/* FBC reference frames */
	int			linear;		/* NV12 outputs */
	bool			opened;		/* decoder instance open */
	bool			seq_done;
	bool			eos;
	bool			draining;
	int			dec_errors;
	int			dq_spins;
};

static d_open_t		wave5_v4l2_open;
static d_ioctl_t	wave5_v4l2_ioctl;
static d_mmap_t		wave5_v4l2_mmap;
static d_poll_t		wave5_v4l2_poll;

static struct cdevsw wave5_v4l2_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	wave5_v4l2_open,
	.d_ioctl =	wave5_v4l2_ioctl,
	.d_mmap =	wave5_v4l2_mmap,
	.d_poll =	wave5_v4l2_poll,
	.d_name =	"wave5_video",
};

static void wave5_fh_stop(struct wave5_fh *fh);

/* ------------------------------------------------------------------ setup */

static void
wave5_queue_free(struct wave5_fh *fh, struct wave5_queue *q)
{
	int i;

	for (i = 0; i < q->count; i++)
		if (q->bufs[i].vb.size != 0)
			wave5_vdi_free_dma_memory(fh->vdev, &q->bufs[i].vb);
	q->count = 0;
	q->streaming = false;
	TAILQ_INIT(&q->done_q);
}

static void
wave5_fh_dtor(void *arg)
{
	struct wave5_fh *fh = arg;

	if (fh->sc->vdev_fh == fh)
		fh->sc->vdev_fh = NULL;
	wave5_fh_stop(fh);
	wave5_queue_free(fh, &fh->out);
	wave5_queue_free(fh, &fh->cap);
	if (fh->bitstream.size != 0)
		wave5_vdi_free_dma_memory(fh->vdev, &fh->bitstream);
	seldrain(&fh->rsel);
	mtx_destroy(&fh->lock);
	kfree(fh);
}

static int
wave5_v4l2_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct wave5_softc *sc = dev->si_drv1;
	struct wave5_fh *fh;
	int err;

	if (sc == NULL || !sc->fw_loaded)
		return (ENXIO);

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (fh == NULL)
		return (ENOMEM);

	fh->sc = sc;
	fh->vdev = &sc->vdev;
	mtx_init(&fh->lock, "wave5fh", NULL, MTX_DEF);
	TAILQ_INIT(&fh->out.done_q);
	TAILQ_INIT(&fh->cap.done_q);

	/* Defaults; the application overrides these with S_FMT. */
	fh->out.pixelformat = V4L2_PIX_FMT_H264;
	fh->out.width = 1920;
	fh->out.height = 1080;
	fh->out.sizeimage = WAVE5_OUTBUF_SIZE;
	fh->cap.pixelformat = V4L2_PIX_FMT_NV12;
	fh->cap.width = 1920;
	fh->cap.height = 1080;

	err = devfs_set_cdevpriv(fh, wave5_fh_dtor);
	if (err != 0) {
		mtx_destroy(&fh->lock);
		kfree(fh);
		return (err);
	}
	sc->vdev_fh = fh;

	return (0);
}

/* --------------------------------------------------------------- formats */

static void
wave5_cap_sizes(struct wave5_queue *q)
{

	q->bytesperline = q->width;
	/* NV12: full luma plane plus a half-height interleaved chroma plane. */
	q->sizeimage = q->width * q->height * 3 / 2;
}

static int
wave5_set_fmt(struct wave5_fh *fh, struct v4l2_format *f, bool try_only)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct wave5_queue *q;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		q = &fh->out;
		if (pix->pixelformat != V4L2_PIX_FMT_H264)
			pix->pixelformat = V4L2_PIX_FMT_H264;
		pix->num_planes = 1;
		pix->plane_fmt[0].sizeimage = WAVE5_OUTBUF_SIZE;
		pix->plane_fmt[0].bytesperline = 0;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		q = &fh->cap;
		if (pix->pixelformat != V4L2_PIX_FMT_NV12)
			pix->pixelformat = V4L2_PIX_FMT_NV12;
		/*
		 * Once the sequence header has been parsed the hardware's
		 * geometry is authoritative -- the application's guess is
		 * whatever the container claimed.
		 */
		if (fh->seq_done) {
			pix->width = fh->cap.width;
			pix->height = fh->cap.height;
		}
		pix->num_planes = 1;
		pix->plane_fmt[0].bytesperline = pix->width;
		pix->plane_fmt[0].sizeimage = pix->width * pix->height * 3 / 2;
	} else
		return (EINVAL);

	if (pix->width == 0 || pix->height == 0)
		return (EINVAL);

	if (!try_only) {
		q->pixelformat = pix->pixelformat;
		if (!(q == &fh->cap && fh->seq_done)) {
			q->width = pix->width;
			q->height = pix->height;
		}
		if (q == &fh->cap)
			wave5_cap_sizes(q);
		else
			q->sizeimage = WAVE5_OUTBUF_SIZE;
	}

	return (0);
}

static int
wave5_get_fmt(struct wave5_fh *fh, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct wave5_queue *q;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q = &fh->out;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q = &fh->cap;
	else
		return (EINVAL);

	memset(pix, 0, sizeof(*pix));
	pix->width = q->width;
	pix->height = q->height;
	pix->pixelformat = q->pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_REC709;
	pix->num_planes = 1;
	pix->plane_fmt[0].bytesperline = q->bytesperline;
	pix->plane_fmt[0].sizeimage = q->sizeimage;

	return (0);
}

/* ---------------------------------------------------------------- buffers */

static int
wave5_reqbufs(struct wave5_fh *fh, struct v4l2_requestbuffers *rb)
{
	struct wave5_queue *q;
	uint32_t base, size;
	int i, ret;

	if (rb->memory != V4L2_MEMORY_MMAP)
		return (EINVAL);

	if (rb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		q = &fh->out;
		base = WAVE5_OFF_OUTPUT;
		size = WAVE5_OUTBUF_SIZE;
	} else if (rb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		q = &fh->cap;
		base = WAVE5_OFF_CAPTURE;
		wave5_cap_sizes(q);
		size = q->sizeimage;
	} else
		return (EINVAL);

	wave5_queue_free(fh, q);
	if (rb->count == 0)
		return (0);
	if (rb->count > WAVE5_MAX_BUFS)
		rb->count = WAVE5_MAX_BUFS;

	for (i = 0; i < (int)rb->count; i++) {
		struct wave5_buf *b = &q->bufs[i];

		memset(b, 0, sizeof(*b));
		b->index = i;
		b->offset = base + i * WAVE5_OFF_STRIDE;
		b->vb.size = size;
		ret = wave5_vdi_allocate_dma_memory(fh->vdev, &b->vb);
		if (ret != 0) {
			q->count = i;
			wave5_queue_free(fh, q);
			return (ENOMEM);
		}
	}
	q->count = rb->count;
	rb->capabilities = V4L2_BUF_CAP_SUPPORTS_MMAP;

	return (0);
}

static void
wave5_fill_vbuf(struct wave5_queue *q, struct wave5_buf *b,
    struct v4l2_buffer *v, struct v4l2_plane *up)
{

	v->index = b->index;
	v->memory = V4L2_MEMORY_MMAP;
	v->length = 1;
	v->flags = V4L2_BUF_FLAG_MAPPED;
	if (b->queued)
		v->flags |= V4L2_BUF_FLAG_QUEUED;
	if (b->done)
		v->flags |= V4L2_BUF_FLAG_DONE;
	v->field = V4L2_FIELD_NONE;
	if (up != NULL) {
		memset(up, 0, sizeof(*up));
		up->length = b->vb.size;
		up->bytesused = b->bytesused;
		up->m.mem_offset = b->offset;
	}
}

/*
 * The multiplanar ioctls carry a userspace pointer to the plane array, so the
 * kernel copy of struct v4l2_buffer only gets the driver there -- the planes
 * themselves must be copied in and out by hand.
 */
static int
wave5_querybuf(struct wave5_fh *fh, struct v4l2_buffer *v)
{
	struct wave5_queue *q;
	struct v4l2_plane plane;
	int err;

	if (v->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q = &fh->out;
	else if (v->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q = &fh->cap;
	else
		return (EINVAL);
	if (v->index >= (uint32_t)q->count)
		return (EINVAL);

	wave5_fill_vbuf(q, &q->bufs[v->index], v, &plane);
	if (v->m.planes == NULL)
		return (EINVAL);
	err = copyout(&plane, v->m.planes, sizeof(plane));

	return (err);
}

static int
wave5_v4l2_mmap(struct cdev *dev, vm_ooffset_t offset, vm_paddr_t *paddr,
    int nprot, vm_memattr_t *memattr)
{
	struct wave5_softc *sc;
	struct wave5_fh *fh;
	struct wave5_queue *q;
	struct wave5_buf *b;
	int i, err = 0;

	/*
	 * NOT devfs_get_cdevpriv(): this runs from the page fault path, where
	 * td_fpop is unset and that call fails with EBADF. See wave5_softc.h.
	 */
	sc = dev->si_drv1;
	if (sc == NULL)
		return (EINVAL);
	fh = sc->vdev_fh;
	if (fh == NULL)
		return (EINVAL);
	(void)err;

	q = ((unsigned long)offset >= WAVE5_OFF_CAPTURE) ? &fh->cap : &fh->out;

	for (i = 0; i < q->count; i++) {
		b = &q->bufs[i];
		if (offset >= b->offset &&
		    offset < b->offset + b->vb.size) {
			/*
			 * Hand userspace the L2 bypass alias, not the real
			 * address. This SoC has no Svpbmt, so VM_MEMATTR_*
			 * cannot make a mapping uncached; the alias is the
			 * only way. It matters in both directions: the VPU is
			 * not coherent with the composable cache, so a cached
			 * view could show an application stale pixels, and
			 * coded data written through a cached mapping might
			 * not have reached DRAM when the hardware reads it.
			 */
			*paddr = (vm_paddr_t)(b->vb.daddr +
			    sifive_ccache_uncached_offset() +
			    (offset - b->offset));
			*memattr = VM_MEMATTR_DEFAULT;
			return (0);
		}
	}

	return (EINVAL);
}

/* --------------------------------------------------- decoder lifecycle */

static void
wave5_test_finish(struct vpu_instance *inst)
{

	complete(&inst->irq_done);
}

static const struct vpu_instance_ops wave5_v4l2_inst_ops = {
	.finish_process = wave5_test_finish,
};

static int
wave5_dec_open(struct wave5_fh *fh)
{
	struct vpu_device *vdev = fh->vdev;
	struct dec_open_param op;
	struct vpu_instance *inst;
	int ret;

	if (fh->opened)
		return (0);

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (inst == NULL)
		return (ENOMEM);
	inst->codec_info = kzalloc(sizeof(*inst->codec_info), GFP_KERNEL);
	if (inst->codec_info == NULL) {
		kfree(inst);
		return (ENOMEM);
	}

	inst->dev = vdev;
	inst->type = VPU_INST_TYPE_DEC;
	inst->std = W_AVC_DEC;
	inst->ops = &wave5_v4l2_inst_ops;
	inst->cbcr_interleave = true;
	inst->nv21 = false;
	inst->output_format = FORMAT_420;
	init_completion(&inst->irq_done);

	inst->id = ida_alloc_max(&vdev->inst_ida, MAX_NUM_INSTANCE - 1,
	    GFP_KERNEL);
	if (inst->id < 0) {
		kfree(inst->codec_info);
		kfree(inst);
		return (ENOSPC);
	}

	/* Must be on the list before any command: the interrupt handler walks
	 * it to decide whose completion to signal. */
	mutex_lock(&vdev->dev_lock);
	list_add_tail(&inst->list, &vdev->instances);
	mutex_unlock(&vdev->dev_lock);

	fh->bitstream.size = WAVE5_BITSTREAM_SIZE;
	ret = wave5_vdi_allocate_dma_memory(vdev, &fh->bitstream);
	if (ret != 0)
		goto fail;

	memset(&op, 0, sizeof(op));
	op.bitstream_buffer = fh->bitstream.daddr;
	op.bitstream_buffer_size = fh->bitstream.size;
	ret = wave5_vpu_dec_open(inst, &op);
	if (ret != 0)
		goto fail;

	fh->inst = inst;
	fh->opened = true;
	fh->bs_used = 0;

	return (0);

fail:
	mutex_lock(&vdev->dev_lock);
	list_del(&inst->list);
	mutex_unlock(&vdev->dev_lock);
	ida_free(&vdev->inst_ida, inst->id);
	if (fh->bitstream.size != 0)
		wave5_vdi_free_dma_memory(vdev, &fh->bitstream);
	kfree(inst->codec_info);
	kfree(inst);
	return (ret < 0 ? -ret : ret);
}

static void
wave5_fh_stop(struct wave5_fh *fh)
{
	struct vpu_instance *inst = fh->inst;
	uint32_t fail_res = 0;

	if (!fh->opened || inst == NULL)
		return;

	wave5_vpu_dec_close(inst, &fail_res);
	mutex_lock(&fh->vdev->dev_lock);
	list_del(&inst->list);
	mutex_unlock(&fh->vdev->dev_lock);
	ida_free(&fh->vdev->inst_ida, inst->id);
	kfree(inst->codec_info);
	kfree(inst);
	fh->inst = NULL;
	fh->opened = false;
	fh->seq_done = false;
}

/*
 * Parse the sequence header. Until this succeeds the real picture size is
 * unknown -- what the application supplied is only what its container
 * claimed -- so G_FMT on the capture queue cannot answer honestly.
 */
static int
wave5_dec_init_seq(struct wave5_fh *fh)
{
	struct dec_initial_info seq;
	int ret;

	if (fh->seq_done)
		return (0);

	ret = wave5_vpu_dec_issue_seq_init(fh->inst);
	if (ret != 0)
		return (-ret);
	if (wave5_vpu_wait_interrupt(fh->inst, VPU_DEC_TIMEOUT) < 0)
		return (ETIMEDOUT);

	memset(&seq, 0, sizeof(seq));
	ret = wave5_vpu_dec_complete_seq_init(fh->inst, &seq);
	if (ret != 0)
		return (-ret);
	if (seq.luma_bitdepth != 8)
		return (ENOTSUP);

	fh->cap.width = seq.pic_width;
	fh->cap.height = seq.pic_height;
	wave5_cap_sizes(&fh->cap);
	fh->inst->src_fmt.width = seq.pic_width;
	fh->inst->src_fmt.height = seq.pic_height;
	fh->inst->dst_fmt.width = seq.pic_width;
	fh->inst->dst_fmt.height = seq.pic_height;
	fh->non_linear = seq.min_frame_buffer_count;
	fh->seq_done = true;

	device_printf(fh->sc->bsddev,
	    "video0: stream %ux%u, %u reference frames\n",
	    seq.pic_width, seq.pic_height, seq.min_frame_buffer_count);

	return (0);
}

/*
 * Register both framebuffer sets. They are not interchangeable: the
 * compressed set is the decoder's own reference store, whose layout the
 * hardware dictates, and the linear set is the NV12 the application reads.
 */
static int
wave5_dec_register_fbs(struct wave5_fh *fh)
{
	struct vpu_instance *inst = fh->inst;
	uint32_t stride, height32, luma, chroma;
	int i, ret;

	stride = fh->cap.width;
	height32 = ALIGN(fh->cap.height, 32);
	luma = stride * height32;
	chroma = ALIGN(stride / 2, 16) * height32;

	if (fh->non_linear + fh->cap.count > MAX_REG_FRAME)
		return (E2BIG);

	for (i = 0; i < fh->non_linear; i++) {
		struct frame_buffer *f = &inst->frame_buf[i];
		struct vpu_buf *vb = &inst->frame_vbuf[i];

		vb->size = luma + chroma;
		ret = wave5_vdi_allocate_dma_memory(fh->vdev, vb);
		if (ret != 0)
			return (-ret);
		f->buf_y = vb->daddr;
		f->buf_cb = vb->daddr + luma;
		f->buf_cr = (dma_addr_t)-1;
		f->size = vb->size;
		f->width = fh->cap.width;
		f->stride = stride;
		f->map_type = COMPRESSED_FRAME_MAP;
		f->update_fb_info = true;
	}

	for (i = 0; i < fh->cap.count; i++) {
		struct frame_buffer *f = &inst->frame_buf[fh->non_linear + i];
		uint32_t lsize = stride * fh->cap.height;

		f->buf_y = fh->cap.bufs[i].vb.daddr;
		f->buf_cb = f->buf_y + lsize;
		f->buf_cr = f->buf_cb + lsize / 4;
		f->size = fh->cap.bufs[i].vb.size;
		f->width = fh->cap.width;
		f->stride = stride;
		f->map_type = LINEAR_FRAME_MAP;
		f->update_fb_info = true;
		wave5_flush_l2_cache(f->buf_y, f->size);
	}
	fh->linear = fh->cap.count;

	ret = wave5_vpu_dec_register_frame_buffer_ex(inst, fh->non_linear,
	    fh->linear, stride, fh->cap.height);
	if (ret != 0)
		return (-ret);

	return (0);
}

/* Decode one picture; returns the linear index displayed, or -1. */
static int
wave5_dec_one(struct wave5_fh *fh)
{
	struct dec_output_info out;
	uint32_t fail_res = 0;
	int ret;

	ret = wave5_vpu_dec_start_one_frame(fh->inst, &fail_res);
	if (ret != 0) {
		if (wave5_debug || fh->dec_errors++ < 3)
			device_printf(fh->sc->bsddev,
			    "video0: start_one_frame: %d (res %#x)\n",
			    ret, fail_res);
		return (-1);
	}
	if (wave5_vpu_wait_interrupt(fh->inst, VPU_DEC_TIMEOUT) < 0) {
		device_printf(fh->sc->bsddev, "video0: decode timed out\n");
		return (-1);
	}

	memset(&out, 0, sizeof(out));
	ret = wave5_vpu_dec_get_output_info(fh->inst, &out);
	if (ret != 0) {
		device_printf(fh->sc->bsddev,
		    "video0: get_output_info: %d\n", ret);
		return (-1);
	}

	if (out.index_frame_display >= 0 &&
	    out.index_frame_display < fh->linear)
		return (out.index_frame_display);

	if (wave5_debug || fh->dec_errors++ < 5)
		device_printf(fh->sc->bsddev,
		    "video0: no display frame (disp %d decoded %d, %ux%u)\n",
		    out.index_frame_display, out.index_frame_decoded,
		    out.dec_pic_width, out.dec_pic_height);

	/*
	 * Only DISPLAY_IDX_FLAG_SEQ_END actually ends the stream. NO_FB means
	 * every framebuffer is still held by the application, and SKIP means
	 * this call produced no picture -- both are transient, and treating
	 * either as end of stream truncates playback (it stopped at 7 of 30
	 * frames).
	 */
	if (out.index_frame_display == DISPLAY_IDX_FLAG_SEQ_END)
		fh->eos = true;

	return (-1);
}

/* ------------------------------------------------------ queue operations */

static int
wave5_qbuf(struct wave5_fh *fh, struct v4l2_buffer *v)
{
	struct v4l2_plane plane;
	struct wave5_buf *b;
	int err;

	if (v->index >= WAVE5_MAX_BUFS)
		return (EINVAL);

	if (v->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (v->index >= (uint32_t)fh->out.count)
			return (EINVAL);
		if (v->m.planes == NULL)
			return (EINVAL);
		err = copyin(v->m.planes, &plane, sizeof(plane));
		if (err != 0)
			return (err);

		b = &fh->out.bufs[v->index];
		b->bytesused = plane.bytesused;

		/*
		 * Coded data arrives in a buffer the application mmap'd; the
		 * decoder reads from its own ring. Copy it across and tell the
		 * firmware how much is there. The copy is of compressed data,
		 * so it is small relative to a frame.
		 */
		if (b->bytesused > 0 && fh->opened) {
			if (fh->bs_used + b->bytesused > fh->bitstream.size)
				fh->bs_used = 0;
			err = wave5_vdi_write_memory(fh->vdev, &fh->bitstream,
			    fh->bs_used, (u8 *)b->vb.vaddr, b->bytesused);
			if (err < 0)
				return (EIO);
			fh->bs_used += b->bytesused;
			wave5_vpu_dec_update_bitstream_buffer(fh->inst,
			    b->bytesused);
		}

		/* Consumed immediately, so it can go straight back. */
		mtx_lock(&fh->lock);
		b->done = true;
		TAILQ_INSERT_TAIL(&fh->out.done_q, b, link);
		mtx_unlock(&fh->lock);
		return (0);
	}

	if (v->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return (EINVAL);
	if (v->index >= (uint32_t)fh->cap.count)
		return (EINVAL);

	b = &fh->cap.bufs[v->index];
	b->queued = true;
	b->done = false;
	if (fh->seq_done && fh->inst != NULL)
		wave5_vpu_dec_clr_disp_flag(fh->inst, v->index);

	/*
	 * Returning a buffer only makes room; it does not decode. Decoding
	 * happens in DQBUF, on demand. Doing it here as well issued more
	 * decodes than there were buffers coming back, and the decoder ran
	 * dry after one pass through the queue with DISPLAY_IDX_FLAG_NO_FB.
	 */
	return (0);
}

static int
wave5_dqbuf(struct wave5_fh *fh, struct v4l2_buffer *v, int flags)
{
	struct v4l2_plane plane;
	struct wave5_queue *q;
	struct wave5_buf *b;
	int err, idx;

	if (v->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q = &fh->out;
	else if (v->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q = &fh->cap;
	else
		return (EINVAL);

	for (;;) {
		mtx_lock(&fh->lock);
		b = TAILQ_FIRST(&q->done_q);
		if (b != NULL) {
			TAILQ_REMOVE(&q->done_q, b, link);
			b->done = false;
			mtx_unlock(&fh->lock);
			fh->dq_spins = 0;
			break;
		}
		mtx_unlock(&fh->lock);

		if (q == &fh->cap && fh->cap.streaming && fh->out.streaming) {
			/* Nothing ready: try to produce one. */
			idx = wave5_dec_one(fh);
			if (idx >= 0) {
				b = &fh->cap.bufs[idx];
				b->queued = false;
				b->bytesused = fh->cap.sizeimage;
				break;
			}
		}

		if (fh->eos)
			return (EPIPE);
		if ((flags & O_NONBLOCK) != 0)
			return (EAGAIN);
		/*
		 * Bounded: without this a decoder that stops producing leaves
		 * the caller blocked forever with no way out but a reboot.
		 */
		if (++fh->dq_spins > 500) {
			fh->dq_spins = 0;
			return (EPIPE);
		}
		if (pause("wave5dq", hz / 100) == EINTR)
			return (EINTR);
	}

	wave5_fill_vbuf(q, b, v, &plane);
	v->bytesused = b->bytesused;
	if (v->m.planes != NULL) {
		err = copyout(&plane, v->m.planes, sizeof(plane));
		if (err != 0)
			return (err);
	}

	return (0);
}

static int
wave5_streamon(struct wave5_fh *fh, uint32_t type)
{
	int err;

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		err = wave5_dec_open(fh);
		if (err != 0)
			return (err);
		fh->out.streaming = true;
		return (0);
	}

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return (EINVAL);
	if (!fh->opened)
		return (EINVAL);

	err = wave5_dec_init_seq(fh);
	if (err != 0) {
		device_printf(fh->sc->bsddev,
		    "video0: sequence init failed: %d\n", err);
		return (err);
	}
	err = wave5_dec_register_fbs(fh);
	if (err != 0) {
		device_printf(fh->sc->bsddev,
		    "video0: framebuffer registration failed: %d "
		    "(%d reference + %d output, %ux%u)\n", err,
		    fh->non_linear, fh->cap.count, fh->cap.width,
		    fh->cap.height);
		return (err);
	}

	fh->cap.streaming = true;
	return (0);
}

static int
wave5_v4l2_poll(struct cdev *dev, int events, struct thread *td)
{
	struct wave5_fh *fh;
	int revents = 0;

	if (devfs_get_cdevpriv((void **)&fh) != 0)
		return (POLLERR);

	mtx_lock(&fh->lock);
	if ((events & (POLLIN | POLLRDNORM)) != 0 &&
	    (!TAILQ_EMPTY(&fh->cap.done_q) || fh->eos))
		revents |= events & (POLLIN | POLLRDNORM);
	if ((events & (POLLOUT | POLLWRNORM)) != 0)
		revents |= events & (POLLOUT | POLLWRNORM);
	if (revents == 0 && (events & (POLLIN | POLLRDNORM)) != 0)
		selrecord(td, &fh->rsel);
	mtx_unlock(&fh->lock);

	return (revents);
}

/* -------------------------------------------------------------- dispatch */

static int
wave5_v4l2_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct wave5_fh *fh;
	int err;

	err = devfs_get_cdevpriv((void **)&fh);
	if (err != 0)
		return (err);

	switch (cmd) {
	case VIDIOC_QUERYCAP: {
		struct v4l2_capability *c = (struct v4l2_capability *)data;

		memset(c, 0, sizeof(*c));
		strlcpy((char *)c->driver, "wave5-dec", sizeof(c->driver));
		strlcpy((char *)c->card, "wave5-dec", sizeof(c->card));
		strlcpy((char *)c->bus_info, "platform:130a0000.vpu_dec",
		    sizeof(c->bus_info));
		c->version = 1;
		c->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
		c->capabilities = c->device_caps | V4L2_CAP_DEVICE_CAPS;
		return (0);
	}

	case VIDIOC_ENUM_FMT: {
		struct v4l2_fmtdesc *f = (struct v4l2_fmtdesc *)data;

		if (f->index != 0)
			return (EINVAL);
		if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
			f->pixelformat = V4L2_PIX_FMT_H264;
			f->flags = V4L2_FMT_FLAG_COMPRESSED;
			strlcpy((char *)f->description, "H.264",
			    sizeof(f->description));
			return (0);
		}
		if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			f->pixelformat = V4L2_PIX_FMT_NV12;
			f->flags = 0;
			strlcpy((char *)f->description, "Y/UV 4:2:0",
			    sizeof(f->description));
			return (0);
		}
		return (EINVAL);
	}

	case VIDIOC_G_FMT:
		/*
		 * ffmpeg calls this on the capture queue to learn the real
		 * geometry before allocating buffers, so parse the sequence
		 * header first if data is available.
		 */
		if (((struct v4l2_format *)data)->type ==
		    V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
		    fh->opened && !fh->seq_done && fh->bs_used > 0)
			(void)wave5_dec_init_seq(fh);
		return (wave5_get_fmt(fh, (struct v4l2_format *)data));

	case VIDIOC_S_FMT:
		return (wave5_set_fmt(fh, (struct v4l2_format *)data, false));

	case VIDIOC_TRY_FMT:
		return (wave5_set_fmt(fh, (struct v4l2_format *)data, true));

	case VIDIOC_REQBUFS:
		return (wave5_reqbufs(fh,
		    (struct v4l2_requestbuffers *)data));

	case VIDIOC_QUERYBUF:
		return (wave5_querybuf(fh, (struct v4l2_buffer *)data));

	case VIDIOC_QBUF:
		return (wave5_qbuf(fh, (struct v4l2_buffer *)data));

	case VIDIOC_DQBUF:
		return (wave5_dqbuf(fh, (struct v4l2_buffer *)data, fflag));

	case VIDIOC_STREAMON:
		return (wave5_streamon(fh, *(uint32_t *)data));

	case VIDIOC_STREAMOFF: {
		uint32_t type = *(uint32_t *)data;

		if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			fh->out.streaming = false;
		else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
			fh->cap.streaming = false;
		else
			return (EINVAL);
		return (0);
	}

	case VIDIOC_DECODER_CMD: {
		struct v4l2_decoder_cmd *c = (struct v4l2_decoder_cmd *)data;

		if (c->cmd == V4L2_DEC_CMD_STOP)
			fh->draining = true;
		return (0);
	}

	/*
	 * Deliberately refused. ffmpeg treats all of these as optional and
	 * carries on: source-change events are unnecessary because it learns
	 * the geometry from G_FMT on the capture queue before allocating
	 * buffers, and the selection/control ioctls only refine cropping and
	 * encoder parameters.
	 */
	case VIDIOC_SUBSCRIBE_EVENT:
	case VIDIOC_UNSUBSCRIBE_EVENT:
	case VIDIOC_DQEVENT:
	case VIDIOC_G_SELECTION:
	case VIDIOC_S_SELECTION:
	case VIDIOC_CROPCAP:
	case VIDIOC_S_PARM:
	case VIDIOC_G_PARM:
	case VIDIOC_S_EXT_CTRLS:
	case VIDIOC_G_EXT_CTRLS:
		return (EINVAL);

	case FIONBIO:
	case FIOASYNC:
		return (0);
	}

	return (ENOTTY);
}

/* ---------------------------------------------------------------- attach */

int
wave5_v4l2_attach(struct wave5_softc *sc)
{

	sc->vdev_cdev = make_dev(&wave5_v4l2_cdevsw, 0, UID_ROOT, GID_VIDEO,
	    0660, "video0");
	if (sc->vdev_cdev == NULL)
		return (ENXIO);
	sc->vdev_cdev->si_drv1 = sc;
	device_printf(sc->bsddev, "/dev/video0: H.264 decoder\n");

	return (0);
}

void
wave5_v4l2_detach(struct wave5_softc *sc)
{

	if (sc->vdev_cdev != NULL) {
		destroy_dev(sc->vdev_cdev);
		sc->vdev_cdev = NULL;
	}
}
