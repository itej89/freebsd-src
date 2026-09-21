/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Shared softc for the JH7110 Wave511 decoder. Split out of wave5_freebsd.c
 * when the V4L2 node (wave5_v4l2.c) needed to reach the same state; keeping
 * one definition avoids the two files drifting apart.
 */

#ifndef	_WAVE5_SOFTC_H_
#define	_WAVE5_SOFTC_H_

#include <sys/conf.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/pwrdom/pwrdom.h>

#include "wave5_osal.h"
#include "wave5-vpuapi.h"

#define	WAVE5_MAX_CLKS		8
#define	WAVE5_MAX_RESETS	8

struct wave5_softc {
	/*
	 * vpu_device must be reachable from struct device's drvdata, because
	 * the ported files do dev_get_drvdata() to find it. Keeping both here
	 * and pointing them at each other in attach avoids a second
	 * allocation.
	 */
	struct vpu_device	vdev;
	struct device		dev;

	device_t		bsddev;
	struct resource		*mem_res;
	struct resource		*irq_res;
	void			*irq_cookie;
	int			mem_rid;
	int			irq_rid;

	clk_t			clks[WAVE5_MAX_CLKS];
	int			nclks;
	hwreset_t		resets[WAVE5_MAX_RESETS];
	int			nresets;
	pwrdom_t		pwrdom;

	/* /dev/video0 -- the V4L2 decoder node; see wave5_v4l2.c */
	struct cdev		*vdev_cdev;
	/*
	 * The open file, reachable without cdevpriv. d_mmap runs from the page
	 * fault handler, where curthread->td_fpop is not set and
	 * devfs_get_cdevpriv() therefore fails with EBADF -- which shows up as
	 * "dev_pager_getpage: map function returns error 9" and a mapping that
	 * faults on every access. Only one open is allowed, so a single pointer
	 * suffices.
	 */
	void			*vdev_fh;

	const struct firmware	*fw;
	uint32_t		fw_revision;
	uint32_t		product_id;
	bool			fw_loaded;
};

/* wave5_v4l2.c -- the /dev/video0 decoder node. */
int	wave5_v4l2_attach(struct wave5_softc *sc);
void	wave5_v4l2_detach(struct wave5_softc *sc);

#endif	/* _WAVE5_SOFTC_H_ */
