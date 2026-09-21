/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * FreeBSD replacement for the vendor driver's wave5-vpu.h.
 *
 * Upstream this header is mostly V4L2: m2m buffer wrappers, container_of
 * helpers from struct v4l2_fh, and the register/unregister entry points for
 * the two video devices. None of that exists here. What the ported hardware
 * files (wave5-hw.c, wave5-vpuapi.c, wave5-vdi.c) actually take from it is
 * the SRAM geometry, the two buffer-sync direction constants and one function
 * declaration, so that is all this carries.
 */

#ifndef	_WAVE5_VPU_H_
#define	_WAVE5_VPU_H_

#include "wave5_osal.h"
#include "wave5-vpuconfig.h"
#include "wave5-vpuapi.h"

#define	VPU_BUF_SYNC_TO_DEVICE		0
#define	VPU_BUF_SYNC_FROM_DEVICE	1

#define	VDI_SRAM_BASE_ADDR		0x00000000
#define	VDI_WAVE511_SRAM_SIZE		0x2D000

/*
 * Blocks until the firmware raises the interrupt for this instance, or
 * timeout milliseconds elapse. Returns 0 or -ETIMEDOUT. Implemented in
 * wave5_freebsd.c against the newbus interrupt handler.
 */
int	wave5_vpu_wait_interrupt(struct vpu_instance *inst, unsigned int timeout);

#endif	/* _WAVE5_VPU_H_ */
