// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave5 series multi-standard codec IP - low level access functions
 *
 * Copyright (C) 2021-2023 CHIPS&MEDIA INC
 */

#include "wave5-vdi.h"
#include "wave5-vpu.h"
#include "wave5-regdefine.h"

/*
 * The VPU is not coherent with the SiFive composable cache, which is why the
 * vendor driver flushes by hand around every buffer it hands the firmware.
 * FreeBSD exposes the same two operations under different names; the 512K
 * threshold above which flushing the whole cache beats walking it line by
 * line is the vendor's, kept as-is.
 */
void wave5_flush_l2_cache(unsigned long start, unsigned long len)
{
	/*
	 * Always flush the actual range, unlike the vendor driver which
	 * switches to a whole-cache flush above 512K.
	 *
	 * FreeBSD's sifive_ccache_flush_all() is not equivalent to Linux's
	 * sifive_ccache_flush_entire(): it issues FLUSH64 for 128K of
	 * addresses starting at 0x40000000, on the theory that this covers
	 * every set and each FLUSH64 evicts all ways. If FLUSH64 instead
	 * flushes only the line matching the physical address it is given --
	 * and the firmware image lives at 0x76c00000, far outside that 128K --
	 * then the image would never reach DRAM and the VCPU would boot from
	 * whatever DRAM held before. Flushing the range we actually wrote
	 * removes the question entirely.
	 *
	 * The cost is bounded: the largest flush here is the ~950K firmware,
	 * which is ~15000 FLUSH64 writes, once, at attach.
	 */
	sifive_ccache_flush_range((vm_paddr_t)start, len);
}

static int wave5_vdi_allocate_common_memory(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);

	if (!vpu_dev->common_mem.vaddr) {
		int ret;

		if (vpu_dev->product_code == WAVE515_CODE)
			vpu_dev->common_mem.size = WAVE515_SIZE_COMMON;
		else
			vpu_dev->common_mem.size = WAVE521_SIZE_COMMON;

		ret = wave5_vdi_allocate_dma_memory(vpu_dev, &vpu_dev->common_mem);
		if (ret) {
			dev_err(dev, "unable to allocate common buffer\n");
			return ret;
		}
	}

	/* %pad is a Linux printf extension; FreeBSD has no equivalent. */
	dev_dbg(dev, "[VDI] common_mem: daddr=%#jx size=%zu vaddr=0x%p\n",
		(uintmax_t)vpu_dev->common_mem.daddr, vpu_dev->common_mem.size,
		vpu_dev->common_mem.vaddr);

	return 0;
}

int wave5_vdi_init(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	int ret;

	ret = wave5_vdi_allocate_common_memory(dev);
	if (ret < 0) {
		dev_err(dev, "[VDI] failed to get vpu common buffer from driver\n");
		return ret;
	}

	if (!PRODUCT_CODE_W_SERIES(vpu_dev->product_code)) {
		WARN_ONCE(1, "unsupported product code: 0x%x\n", vpu_dev->product_code);
		return -EOPNOTSUPP;
	}

	/* if BIT processor is not running. */
	if (wave5_vdi_read_register(vpu_dev, W5_VCPU_CUR_PC) == 0) {
		int i;

		for (i = 0; i < 64; i++)
			wave5_vdi_write_register(vpu_dev, (i * 4) + 0x100, 0x0);
	}

	dev_dbg(dev, "[VDI] driver initialized successfully\n");

	return 0;
}

int wave5_vdi_release(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);

	vpu_dev->vdb_register = NULL;
	wave5_vdi_free_dma_memory(vpu_dev, &vpu_dev->common_mem);

	return 0;
}

/*
 * Register tracing, for diffing against the Linux reference traces in
 * vf2-ddk-mesa-test/reference/wave5/. Set hw.wave5.trace_regs=N (a tunable, so
 * it can be armed before the driver attaches) to log the first N accesses in
 * exactly the format the instrumented Linux driver emits.
 *
 * Reads are deduplicated for the same reason they are there: a busy-wait poll
 * issues millions of identical reads, and on this board's 115200-baud serial
 * console each printed line costs about 11 ms, so logging them all would both
 * bury the sequence and change the timing being observed.
 */
int wave5_trace_regs;

static u32 wave5_trace_last_addr = 0xffffffff, wave5_trace_last_val;
static unsigned int wave5_trace_rep;

static void wave5_trace_flush_rep(void)
{
	if (wave5_trace_rep > 1)
		printf("W5 R %04x -> %08x (x%u)\n", wave5_trace_last_addr,
		    wave5_trace_last_val, wave5_trace_rep);
	wave5_trace_rep = 0;
	wave5_trace_last_addr = 0xffffffff;
}

void wave5_vdi_write_register(struct vpu_device *vpu_dev, u32 addr, u32 data)
{
	if (wave5_trace_regs > 0) {
		wave5_trace_flush_rep();
		wave5_trace_regs--;
		printf("W5 W %04x <- %08x\n", addr, data);
	}
	writel(data, vpu_dev->vdb_register + addr);
}

unsigned int wave5_vdi_read_register(struct vpu_device *vpu_dev, u32 addr)
{
	u32 v = readl(vpu_dev->vdb_register + addr);

	if (wave5_trace_regs > 0) {
		if (addr == wave5_trace_last_addr && v == wave5_trace_last_val) {
			wave5_trace_rep++;
		} else {
			wave5_trace_flush_rep();
			wave5_trace_regs--;
			printf("W5 R %04x -> %08x\n", addr, v);
			wave5_trace_last_addr = addr;
			wave5_trace_last_val = v;
			wave5_trace_rep = 1;
		}
	}
	return v;
}

int wave5_vdi_clear_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb)
{
	if (!vb || !vb->vaddr) {
		dev_err(vpu_dev->dev, "%s: unable to clear unmapped buffer\n", __func__);
		return -EINVAL;
	}

	memset(vb->vaddr, 0, vb->size);
	wave5_flush_l2_cache(vb->daddr, vb->size);
	return vb->size;
}

int wave5_vdi_write_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb, size_t offset,
			   u8 *data, size_t len)
{
	if (!vb || !vb->vaddr) {
		dev_err(vpu_dev->dev, "%s: unable to write to unmapped buffer\n", __func__);
		return -EINVAL;
	}

	if (offset > vb->size || len > vb->size || offset + len > vb->size) {
		dev_err(vpu_dev->dev, "%s: buffer too small\n", __func__);
		return -ENOSPC;
	}

	memcpy(vb->vaddr + offset, data, len);
	wave5_flush_l2_cache(vb->daddr + offset, len);

	return len;
}

/*
 * bus_dma replaces dma_alloc_coherent(). The callback runs synchronously
 * because the tag is created without BUS_DMA_ALLOCNOW deferral and the
 * allocation is a single contiguous segment, so taking the address out of it
 * like this is safe.
 */
static void
wave5_vdi_dma_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{

	if (error != 0 || nseg != 1)
		return;
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

int wave5_vdi_allocate_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb)
{
	bus_addr_t daddr = 0;
	void *vaddr = NULL;
	bus_dma_tag_t tag;
	bus_dmamap_t map;
	int err;

	if (!vb->size) {
		dev_err(vpu_dev->dev, "%s: requested size==0\n", __func__);
		return -EINVAL;
	}

	/*
	 * A child of the device tag, sized for this buffer: bus_dmamem_alloc()
	 * allocates maxsize from the tag it is given, so a shared tag cannot
	 * serve allocations of different sizes. The parent already carries the
	 * 4G ceiling and the alignment the core needs.
	 */
	err = bus_dma_tag_create(vpu_dev->dev->dmat, WAVE5_DMA_ALIGN, 0,
	    WAVE5_DMA_HIGHADDR, BUS_SPACE_MAXADDR, NULL, NULL,
	    vb->size, 1, vb->size, 0, NULL, NULL, &tag);
	if (err != 0) {
		dev_err(vpu_dev->dev, "%s: bus_dma_tag_create(%zu): %d\n",
		    __func__, vb->size, err);
		return -ENOMEM;
	}

	err = bus_dmamem_alloc(tag, &vaddr, BUS_DMA_WAITOK | BUS_DMA_ZERO,
	    &map);
	if (err != 0) {
		dev_err(vpu_dev->dev, "%s: bus_dmamem_alloc(%zu): %d\n",
		    __func__, vb->size, err);
		bus_dma_tag_destroy(tag);
		return -ENOMEM;
	}

	err = bus_dmamap_load(tag, map, vaddr, vb->size,
	    wave5_vdi_dma_cb, &daddr, BUS_DMA_NOWAIT);
	if (err != 0 || daddr == 0) {
		dev_err(vpu_dev->dev, "%s: bus_dmamap_load(%zu): %d\n",
		    __func__, vb->size, err);
		bus_dmamem_free(tag, vaddr, map);
		bus_dma_tag_destroy(tag);
		return -ENOMEM;
	}

	/*
	 * The core drives 32-bit addresses; the tag is built with a 4G
	 * boundary so this should never fire, but a silently truncated
	 * address would corrupt memory rather than fail, so check it.
	 */
	if (daddr + vb->size > WAVE5_DMA_HIGHADDR + 1) {
		dev_err(vpu_dev->dev, "%s: buffer above 4G: %#jx\n",
		    __func__, (uintmax_t)daddr);
		bus_dmamap_unload(tag, map);
		bus_dmamem_free(tag, vaddr, map);
		bus_dma_tag_destroy(tag);
		return -ENOMEM;
	}

	vb->vaddr = vaddr;
	vb->daddr = daddr;
	vb->tag = tag;
	vb->map = map;

	wave5_flush_l2_cache(daddr, vb->size);

	return 0;
}

int wave5_vdi_free_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb)
{
	if (vb->size == 0)
		return -EINVAL;

	if (!vb->vaddr) {
		dev_err(vpu_dev->dev, "%s: requested free of unmapped buffer\n", __func__);
	} else {
		bus_dmamap_unload(vb->tag, vb->map);
		bus_dmamem_free(vb->tag, vb->vaddr, vb->map);
		bus_dma_tag_destroy(vb->tag);
	}

	memset(vb, 0, sizeof(*vb));

	return 0;
}

int wave5_vdi_allocate_array(struct vpu_device *vpu_dev, struct vpu_buf *array, unsigned int count,
			     size_t size)
{
	struct vpu_buf vb_buf;
	int i, ret = 0;

	vb_buf.size = size;

	for (i = 0; i < count; i++) {
		if (array[i].size == size)
			continue;

		if (array[i].size != 0)
			wave5_vdi_free_dma_memory(vpu_dev, &array[i]);

		ret = wave5_vdi_allocate_dma_memory(vpu_dev, &vb_buf);
		if (ret)
			return -ENOMEM;
		array[i] = vb_buf;
	}

	for (i = count; i < MAX_REG_FRAME; i++)
		wave5_vdi_free_dma_memory(vpu_dev, &array[i]);

	return 0;
}

/*
 * On-chip SRAM is not wired up on FreeBSD.
 *
 * Upstream these draw from a gen_pool describing the AXI-attached SRAM and
 * hand the core a faster scratch area for a few internal buffers. It is
 * strictly an optimisation: every consumer checks sram_buf.size and falls
 * back to ordinary DRAM, which is what happens here with sram_buf left zero.
 *
 * Implementing it needs an SRAM allocator FreeBSD does not have, so it is
 * deliberately left out until decode throughput is measured and shown to want
 * it. Leaving the vendor bodies in place with a stubbed gen_pool would have
 * compiled to the same no-op while looking like working code.
 */
void wave5_vdi_allocate_sram(struct vpu_device *vpu_dev)
{
}

void wave5_vdi_free_sram(struct vpu_device *vpu_dev)
{
}
