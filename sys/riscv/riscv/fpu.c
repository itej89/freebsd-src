/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * Kernel FPU context management for RISC-V.
 * Provides fpu_kern_enter/leave for safe FPU use in kernel context.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/pcpu.h>

#include <machine/reg.h>
#include <machine/pcb.h>
#include <machine/fpe.h>
#include <machine/fpu.h>

static MALLOC_DEFINE(M_FPUKERN_CTX, "fpukern_ctx",
    "Kernel FPU context");

struct fpu_kern_ctx {
	struct fpreg	state;
	uint32_t	flags;
#define	FPU_KERN_CTX_INUSE	0x01
};

struct fpu_kern_ctx *
fpu_kern_alloc_ctx(u_int flags)
{

	return (malloc(sizeof(struct fpu_kern_ctx), M_FPUKERN_CTX,
	    ((flags & FPU_KERN_NOWAIT) ? M_NOWAIT : M_WAITOK) | M_ZERO));
}

void
fpu_kern_free_ctx(struct fpu_kern_ctx *ctx)
{

	KASSERT((ctx->flags & FPU_KERN_CTX_INUSE) == 0,
	    ("freeing inuse FPU ctx"));
	free(ctx, M_FPUKERN_CTX);
}

void
fpu_kern_enter(struct thread *td, struct fpu_kern_ctx *ctx, u_int flags)
{
	struct pcb *pcb;

	pcb = td->td_pcb;

	if ((flags & FPU_KERN_NOCTX) != 0) {
		critical_enter();
		if (pcb->pcb_fpflags & PCB_FP_STARTED)
			fpe_state_save(td);
		fpe_enable();
		pcb->pcb_fpflags |= PCB_FP_KERN | PCB_FP_NOSAVE |
		    PCB_FP_STARTED;
		return;
	}

	if (ctx != NULL) {
		ctx->flags = FPU_KERN_CTX_INUSE;
		if (pcb->pcb_fpflags & PCB_FP_STARTED)
			fpe_state_save(td);
		fpe_enable();
		pcb->pcb_fpflags |= PCB_FP_KERN | PCB_FP_STARTED;
	}
}

int
fpu_kern_leave(struct thread *td, struct fpu_kern_ctx *ctx)
{
	struct pcb *pcb;

	pcb = td->td_pcb;

	if ((pcb->pcb_fpflags & PCB_FP_NOSAVE) != 0) {
		fpe_disable();
		pcb->pcb_fpflags &= ~(PCB_FP_KERN | PCB_FP_NOSAVE |
		    PCB_FP_STARTED);
		critical_exit();
		return (0);
	}

	if (ctx != NULL) {
		ctx->flags &= ~FPU_KERN_CTX_INUSE;
		pcb->pcb_fpflags &= ~PCB_FP_KERN;
	}

	return (0);
}
