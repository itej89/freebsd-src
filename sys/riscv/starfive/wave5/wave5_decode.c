/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Wave511 decode self-test.
 *
 * FreeBSD has no V4L2 m2m framework, so the vendor's decoder front end
 * (wave5-vpu-dec.c) has no equivalent here yet. This drives the same ported
 * hardware API that front end drives -- open, sequence init, framebuffer
 * registration, per-picture decode -- entirely from the kernel, and hashes
 * each decoded frame.
 *
 * The point is verification. The identical clip decoded by the vendor driver
 * on the Debian board produces known per-frame MD5s (see
 * vf2-ddk-mesa-test/reference/wave5/), so matching hashes prove this port
 * decodes video correctly, pixel for pixel, rather than merely running without
 * error. Building a userspace interface first would have meant debugging the
 * decoder and the interface at the same time.
 *
 *	sysctl hw.wave5.selftest=1
 *
 * with the bitstream at /boot/firmware/wave5_test.h264 (firmware(9) opens that
 * path directly). Output frames are NV12.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/firmware.h>
#include <sys/md5.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>

#include "wave5_osal.h"
#include "wave5-vpu.h"
#include "wave5-vpuapi.h"
#include "wave5-regdefine.h"
#include "wave5.h"

#define	WAVE5_TEST_STREAM	"wave5_test.h264"

/* Linear output buffers beyond the decoder's stated minimum. */
#define	WAVE5_TEST_EXTRA_FBS	3

int wave5_run_selftest(struct vpu_device *vdev);

/*
 * Called from the interrupt handler when a picture completes. The vendor
 * driver uses this hook to hand a buffer back to V4L2; here the decode loop is
 * simply waiting, so it only needs waking.
 */
static void
wave5_test_finish_process(struct vpu_instance *inst)
{

	complete(&inst->irq_done);
}

static const struct vpu_instance_ops wave5_test_ops = {
	.finish_process = wave5_test_finish_process,
};

/*
 * Hash a decoded frame.
 *
 * Read through the SiFive L2 bypass alias rather than the cached mapping the
 * buffer was allocated with. The VPU is not coherent with the composable
 * cache -- that is why this driver flushes by hand around every buffer it
 * hands the firmware -- so a cached read here could be served by a stale line
 * and produce a hash of data the hardware never wrote.
 */
static int
wave5_test_hash_frame(struct vpu_device *vdev, bus_addr_t daddr, size_t len,
    char *out, size_t outlen)
{
	MD5_CTX ctx;
	uint8_t digest[16];
	volatile uint8_t *p;
	uint64_t off;
	size_t i;

	off = sifive_ccache_uncached_offset();
	p = pmap_mapdev((vm_paddr_t)daddr + off, len);
	if (p == NULL)
		return (ENOMEM);

	MD5Init(&ctx);
	/*
	 * MD5Update() wants a plain pointer; the mapping is only volatile to
	 * stop the compiler caching loads, and nothing else touches it here.
	 */
	MD5Update(&ctx, __DEVOLATILE(const void *, p), len);
	MD5Final(digest, &ctx);

	pmap_unmapdev(__DEVOLATILE(void *, p), len);

	for (i = 0; i < sizeof(digest); i++)
		snprintf(out + i * 2, outlen - i * 2, "%02x", digest[i]);

	return (0);
}

static void
wave5_test_free_frames(struct vpu_instance *inst, int count)
{
	int i;

	for (i = 0; i < count && i < MAX_REG_FRAME; i++)
		if (inst->frame_vbuf[i].size != 0)
			wave5_vdi_free_dma_memory(inst->dev,
			    &inst->frame_vbuf[i]);
}

int
wave5_run_selftest(struct vpu_device *vdev)
{
	const struct firmware *fw;
	struct vpu_instance *inst;
	struct dec_open_param open_param;
	struct dec_initial_info seq;
	struct dec_output_info out;
	char hash[40];
	size_t frame_size;
	u32 fail_res;
	u32 fb_stride, fb_height, luma_size, chroma_size;
	int non_linear, linear, total;
	int i, ret, frames = 0, err = 0;

	fw = firmware_get(WAVE5_TEST_STREAM);
	if (fw == NULL) {
		device_printf(vdev->dev->bsddev,
		    "selftest: %s not found; put it in /boot/firmware\n",
		    WAVE5_TEST_STREAM);
		return (ENOENT);
	}
	device_printf(vdev->dev->bsddev, "selftest: %s, %zu bytes\n",
	    WAVE5_TEST_STREAM, fw->datasize);

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (inst == NULL) {
		err = ENOMEM;
		goto out_fw;
	}
	inst->codec_info = kzalloc(sizeof(*inst->codec_info), GFP_KERNEL);
	if (inst->codec_info == NULL) {
		err = ENOMEM;
		goto out_inst;
	}

	inst->dev = vdev;
	inst->type = VPU_INST_TYPE_DEC;
	inst->std = W_AVC_DEC;
	inst->ops = &wave5_test_ops;
	/* NV12: chroma interleaved, Cb first. */
	inst->cbcr_interleave = true;
	inst->nv21 = false;
	inst->output_format = FORMAT_420;
	init_completion(&inst->irq_done);

	inst->id = ida_alloc_max(&vdev->inst_ida, MAX_NUM_INSTANCE - 1,
	    GFP_KERNEL);
	if (inst->id < 0) {
		err = ENOSPC;
		goto out_codec;
	}

	/*
	 * The instance must be on the device list before any command is
	 * issued: the interrupt handler walks that list to find whose
	 * completion to signal, so an instance that is not on it would simply
	 * never be woken.
	 */
	mutex_lock(&vdev->dev_lock);
	list_add_tail(&inst->list, &vdev->instances);
	mutex_unlock(&vdev->dev_lock);

	/* Bitstream buffer: the whole clip at once, no ring wrap to handle. */
	inst->bitstream_vbuf.size = ALIGN(fw->datasize, 1024);
	ret = wave5_vdi_allocate_dma_memory(vdev, &inst->bitstream_vbuf);
	if (ret != 0) {
		err = -ret;
		goto out_list;
	}
	ret = wave5_vdi_write_memory(vdev, &inst->bitstream_vbuf, 0,
	    __DECONST(u8 *, fw->data), fw->datasize);
	if (ret < 0) {
		err = -ret;
		goto out_bitstream;
	}

	memset(&open_param, 0, sizeof(open_param));
	open_param.bitstream_buffer = inst->bitstream_vbuf.daddr;
	open_param.bitstream_buffer_size = inst->bitstream_vbuf.size;

	ret = wave5_vpu_dec_open(inst, &open_param);
	if (ret != 0) {
		device_printf(vdev->dev->bsddev, "selftest: open failed: %d\n",
		    ret);
		err = -ret;
		goto out_bitstream;
	}

	/* Tell the decoder how much of the buffer actually holds data. */
	ret = wave5_vpu_dec_update_bitstream_buffer(inst, fw->datasize);
	if (ret != 0) {
		device_printf(vdev->dev->bsddev,
		    "selftest: update_bitstream_buffer failed: %d\n", ret);
		err = -ret;
		goto out_close;
	}

	ret = wave5_vpu_dec_issue_seq_init(inst);
	if (ret != 0) {
		device_printf(vdev->dev->bsddev,
		    "selftest: issue_seq_init failed: %d\n", ret);
		err = -ret;
		goto out_close;
	}
	if (wave5_vpu_wait_interrupt(inst, VPU_DEC_TIMEOUT) < 0) {
		device_printf(vdev->dev->bsddev,
		    "selftest: timed out waiting for sequence init\n");
		err = ETIMEDOUT;
		goto out_close;
	}

	memset(&seq, 0, sizeof(seq));
	ret = wave5_vpu_dec_complete_seq_init(inst, &seq);
	if (ret != 0) {
		device_printf(vdev->dev->bsddev,
		    "selftest: complete_seq_init failed: %d reason %#x\n",
		    ret, seq.seq_init_err_reason);
		err = -ret;
		goto out_close;
	}

	device_printf(vdev->dev->bsddev,
	    "selftest: stream is %ux%u, %u-bit, needs %u framebuffers\n",
	    seq.pic_width, seq.pic_height, seq.luma_bitdepth,
	    seq.min_frame_buffer_count);

	if (seq.luma_bitdepth != 8) {
		device_printf(vdev->dev->bsddev,
		    "selftest: only 8-bit is handled here\n");
		err = ENOTSUP;
		goto out_close;
	}

	inst->src_fmt.width = seq.pic_width;
	inst->src_fmt.height = seq.pic_height;
	inst->dst_fmt.width = seq.pic_width;
	inst->dst_fmt.height = seq.pic_height;

	non_linear = seq.min_frame_buffer_count;
	linear = seq.min_frame_buffer_count + WAVE5_TEST_EXTRA_FBS;
	total = non_linear + linear;
	if (total > MAX_REG_FRAME) {
		device_printf(vdev->dev->bsddev,
		    "selftest: %d framebuffers exceeds MAX_REG_FRAME\n", total);
		err = E2BIG;
		goto out_close;
	}
	inst->fbc_buf_count = non_linear;

	/*
	 * Two sets of buffers. The first are the decoder's own compressed
	 * reference frames, whose layout is dictated by the hardware; the
	 * second are the linear NV12 outputs we actually read.
	 */
	fb_stride = inst->dst_fmt.width;
	fb_height = ALIGN(inst->dst_fmt.height, 32);
	luma_size = fb_stride * fb_height;
	chroma_size = ALIGN(fb_stride / 2, 16) * fb_height;

	for (i = 0; i < non_linear; i++) {
		struct frame_buffer *f = &inst->frame_buf[i];
		struct vpu_buf *vb = &inst->frame_vbuf[i];

		vb->size = luma_size + chroma_size;
		ret = wave5_vdi_allocate_dma_memory(vdev, vb);
		if (ret != 0) {
			err = -ret;
			goto out_frames;
		}
		f->buf_y = vb->daddr;
		f->buf_cb = vb->daddr + luma_size;
		f->buf_cr = (dma_addr_t)-1;
		f->size = vb->size;
		f->width = inst->src_fmt.width;
		f->stride = fb_stride;
		f->map_type = COMPRESSED_FRAME_MAP;
		f->update_fb_info = true;
	}

	frame_size = (size_t)fb_stride * inst->dst_fmt.height * 3 / 2;
	for (i = 0; i < linear; i++) {
		struct frame_buffer *f = &inst->frame_buf[non_linear + i];
		struct vpu_buf *vb = &inst->frame_vbuf[non_linear + i];
		u32 lsize = fb_stride * inst->dst_fmt.height;

		vb->size = frame_size;
		ret = wave5_vdi_allocate_dma_memory(vdev, vb);
		if (ret != 0) {
			err = -ret;
			goto out_frames;
		}
		f->buf_y = vb->daddr;
		f->buf_cb = vb->daddr + lsize;
		f->buf_cr = f->buf_cb + lsize / 4;
		f->size = vb->size;
		f->width = inst->src_fmt.width;
		f->stride = fb_stride;
		f->map_type = LINEAR_FRAME_MAP;
		f->update_fb_info = true;

		wave5_flush_l2_cache(f->buf_y, f->size);
	}

	ret = wave5_vpu_dec_register_frame_buffer_ex(inst, non_linear, linear,
	    fb_stride, inst->dst_fmt.height);
	if (ret != 0) {
		device_printf(vdev->dev->bsddev,
		    "selftest: register_frame_buffer failed: %d\n", ret);
		err = -ret;
		goto out_frames;
	}

	device_printf(vdev->dev->bsddev,
	    "selftest: decoding (frame is %zu bytes NV12)\n", frame_size);

	for (;;) {
		fail_res = 0;
		ret = wave5_vpu_dec_start_one_frame(inst, &fail_res);
		if (ret != 0) {
			/* A clean end of stream arrives as a start failure. */
			device_printf(vdev->dev->bsddev,
			    "selftest: stopped after %d frames (%d, res %#x)\n",
			    frames, ret, fail_res);
			break;
		}

		if (wave5_vpu_wait_interrupt(inst, VPU_DEC_TIMEOUT) < 0) {
			device_printf(vdev->dev->bsddev,
			    "selftest: timed out on frame %d\n", frames);
			err = ETIMEDOUT;
			break;
		}

		memset(&out, 0, sizeof(out));
		ret = wave5_vpu_dec_get_output_info(inst, &out);
		if (ret != 0) {
			device_printf(vdev->dev->bsddev,
			    "selftest: get_output_info failed: %d\n", ret);
			err = -ret;
			break;
		}

		if (out.index_frame_display >= 0 &&
		    out.index_frame_display < linear) {
			struct frame_buffer *f =
			    &inst->frame_buf[non_linear + out.index_frame_display];

			if (wave5_test_hash_frame(vdev, f->buf_y, frame_size,
			    hash, sizeof(hash)) == 0)
				device_printf(vdev->dev->bsddev,
				    "selftest: frame %d %ux%u md5 %s\n",
				    frames, out.dec_pic_width,
				    out.dec_pic_height, hash);
			frames++;
			wave5_vpu_dec_clr_disp_flag(inst,
			    out.index_frame_display);
		} else if (out.index_frame_decoded < 0) {
			device_printf(vdev->dev->bsddev,
			    "selftest: no more pictures after %d frames\n",
			    frames);
			break;
		}

		if (frames > 1000)
			break;
	}

	device_printf(vdev->dev->bsddev, "selftest: decoded %d frames\n",
	    frames);

out_frames:
	wave5_test_free_frames(inst, total);
out_close:
	fail_res = 0;
	wave5_vpu_dec_close(inst, &fail_res);
out_bitstream:
	if (inst->bitstream_vbuf.size != 0)
		wave5_vdi_free_dma_memory(vdev, &inst->bitstream_vbuf);
out_list:
	mutex_lock(&vdev->dev_lock);
	list_del(&inst->list);
	mutex_unlock(&vdev->dev_lock);
	ida_free(&vdev->inst_ida, inst->id);
out_codec:
	kfree(inst->codec_info);
out_inst:
	kfree(inst);
out_fw:
	firmware_put(fw, FIRMWARE_UNLOAD);
	return (err);
}
