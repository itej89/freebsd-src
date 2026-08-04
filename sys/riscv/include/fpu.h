/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * Kernel FPU context for RISC-V.
 */

#ifndef _MACHINE_FPU_H_
#define	_MACHINE_FPU_H_

#define	FPU_KERN_NORMAL	0x0000
#define	FPU_KERN_NOWAIT	0x0001
#define	FPU_KERN_KTHR	0x0002
#define	FPU_KERN_NOCTX	0x0004

struct fpu_kern_ctx;

struct fpu_kern_ctx *fpu_kern_alloc_ctx(u_int flags);
void fpu_kern_free_ctx(struct fpu_kern_ctx *ctx);
void fpu_kern_enter(struct thread *td, struct fpu_kern_ctx *ctx, u_int flags);
int fpu_kern_leave(struct thread *td, struct fpu_kern_ctx *ctx);

#endif /* _MACHINE_FPU_H_ */
