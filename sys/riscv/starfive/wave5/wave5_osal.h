/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * OS abstraction for the Chips&Media Wave5 (JH7110 Wave511) codec core.
 *
 * The vendor driver splits cleanly in two. wave5-hw.c, wave5-vpuapi.c and
 * wave5-vdi.c -- 3950 lines -- drive the hardware and contain no V4L2 or
 * videobuf2 reference at all; wave5-vpu-dec.c and friends are the V4L2 m2m
 * glue. FreeBSD has no V4L2 m2m framework, so only the first half is ported
 * and the second half is replaced by wave5_freebsd.c.
 *
 * Between the hardware half and FreeBSD sits exactly this file. The whole
 * Linux surface those 3950 lines touch is small enough to list:
 *
 *     dev_err/dev_warn/dev_dbg   ALIGN   BIT   GENMASK   FIELD_GET
 *     mutex_lock_interruptible/mutex_unlock       complete
 *     read_poll_timeout   DIV_ROUND_UP   min_t   memcpy/memset
 *     readl/writel   dma_alloc_coherent   WARN_ONCE   gen_pool (SRAM only)
 *
 * so it is shimmed natively here rather than pulled in through LinuxKPI.
 * That is a deliberate choice: LinuxKPI has twice silently mis-served this
 * project's OF/platform devices (dma_set_mask and request_irq both succeed
 * and then do nothing for a non-PCI device), and a 300-line shim we own is
 * cheaper to trust than a compatibility layer we have already been bitten by.
 *
 * Linux error convention is kept *inside* the ported files -- they return
 * -EINVAL, -ETIMEDOUT and so on -- and is negated back at the FreeBSD
 * boundary in wave5_freebsd.c. Do not "fix" a negative return in here.
 */

#ifndef _WAVE5_OSAL_H_
#define _WAVE5_OSAL_H_

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <riscv/sifive/sifive_ccache.h>

/* ------------------------------------------------------------------ types */

typedef uint8_t		u8;
typedef uint16_t	u16;
typedef uint32_t	u32;
typedef uint64_t	u64;
typedef int8_t		s8;
typedef int16_t		s16;
typedef int32_t		s32;
typedef int64_t		s64;

typedef bus_addr_t	dma_addr_t;

#define	__iomem
#define	__force
#define	__must_check

/*
 * The core is wired with a 32-bit DMA mask (the DT says so and the firmware
 * loads addresses into 32-bit registers), so every buffer must live below 4G.
 * This is the same constraint the GPU has on this SoC.
 */
#define	WAVE5_DMA_HIGHADDR	((bus_addr_t)0xffffffffUL)

/*
 * The firmware programs buffer addresses into registers that hold a 4K-page
 * index in some cases, and several internal buffers are documented as
 * 4K-aligned, so every allocation is aligned to a page.
 */
#define	WAVE5_DMA_ALIGN		((bus_size_t)4096)

/* --------------------------------------------------------------- device */

/*
 * A stand-in for Linux's struct device. The ported files only ever use it to
 * log and to reach back to the softc via dev_get_drvdata(), so it carries the
 * newbus device_t, that pointer, and the DMA tag buffers are allocated from.
 */
struct device {
	device_t	bsddev;
	void		*drvdata;
	bus_dma_tag_t	dmat;
};

#define	dev_get_drvdata(d)	((d)->drvdata)
#define	dev_set_drvdata(d, p)	((d)->drvdata = (p))

extern int wave5_debug;

#define	dev_err(d, ...)		device_printf((d)->bsddev, __VA_ARGS__)
#define	dev_warn(d, ...)	device_printf((d)->bsddev, __VA_ARGS__)
#define	dev_info(d, ...)	device_printf((d)->bsddev, __VA_ARGS__)
#define	dev_dbg(d, ...)	do {						\
	if (__predict_false(wave5_debug))				\
		device_printf((d)->bsddev, __VA_ARGS__);		\
} while (0)

/* ---------------------------------------------------------------- memory */

MALLOC_DECLARE(M_WAVE5);

#define	kzalloc(sz, fl)		malloc((sz), M_WAVE5, M_NOWAIT | M_ZERO)
#define	kmalloc(sz, fl)		malloc((sz), M_WAVE5, M_NOWAIT)
#define	kcalloc(n, sz, fl)	mallocarray((n), (sz), M_WAVE5, M_NOWAIT | M_ZERO)
#define	kfree(p)		free(__DECONST(void *, p), M_WAVE5)
#define	vzalloc(sz)		malloc((sz), M_WAVE5, M_NOWAIT | M_ZERO)
#define	vfree(p)		free((p), M_WAVE5)

#define	GFP_KERNEL		0
#define	GFP_ATOMIC		0

/* -------------------------------------------------------------------- io */

/*
 * RISC-V device access needs explicit ordering: a plain load or store may be
 * reordered against normal memory, and the VPU's command protocol depends on
 * register writes landing in program order. These mirror the fences Linux's
 * riscv readl()/writel() emit.
 */
static inline u32
readl(const volatile void *addr)
{
	u32 v;

	v = *(const volatile u32 *)addr;
	__asm __volatile("fence i,r" ::: "memory");
	return (v);
}

static inline void
writel(u32 val, volatile void *addr)
{

	__asm __volatile("fence w,o" ::: "memory");
	*(volatile u32 *)addr = val;
}

/* ----------------------------------------------------------------- locks */

struct mutex {
	struct sx	sx;
};

#define	mutex_init(m)		sx_init(&(m)->sx, "wave5")
#define	mutex_destroy(m)	sx_destroy(&(m)->sx)
#define	mutex_lock(m)		sx_xlock(&(m)->sx)
#define	mutex_unlock(m)		sx_xunlock(&(m)->sx)
#define	mutex_is_locked(m)	sx_xlocked(&(m)->sx)

/* Returns 0 or -EINTR, as Linux does. */
static inline int
mutex_lock_interruptible(struct mutex *m)
{

	return (sx_xlock_sig(&m->sx) != 0 ? -EINTR : 0);
}

typedef struct {
	struct mtx	mtx;
} spinlock_t;

#define	spin_lock_init(l)	mtx_init(&(l)->mtx, "wave5sl", NULL, MTX_DEF)
#define	spin_lock_irqsave(l, f)	do { (f) = 0; mtx_lock(&(l)->mtx); } while (0)
#define	spin_unlock_irqrestore(l, f) do { (void)(f); mtx_unlock(&(l)->mtx); } while (0)
#define	spin_lock(l)		mtx_lock(&(l)->mtx)
#define	spin_unlock(l)		mtx_unlock(&(l)->mtx)

/* ----------------------------------------------------------- completion */

struct completion {
	struct mtx	mtx;
	int		done;
};

static inline void
init_completion(struct completion *c)
{

	bzero(c, sizeof(*c));
	mtx_init(&c->mtx, "wave5cp", NULL, MTX_DEF);
}

static inline void
destroy_completion(struct completion *c)
{

	mtx_destroy(&c->mtx);
}

static inline void
reinit_completion(struct completion *c)
{

	mtx_lock(&c->mtx);
	c->done = 0;
	mtx_unlock(&c->mtx);
}

static inline void
complete(struct completion *c)
{

	mtx_lock(&c->mtx);
	c->done = 1;
	wakeup(c);
	mtx_unlock(&c->mtx);
}

/*
 * Linux returns remaining jiffies (0 on timeout); callers here only test for
 * zero, so return 1 or 0.
 */
static inline int
wait_for_completion_timeout(struct completion *c, int ticks_to_wait)
{
	int ret = 1;

	mtx_lock(&c->mtx);
	while (c->done == 0) {
		if (msleep(c, &c->mtx, 0, "wave5wt", ticks_to_wait) != 0) {
			ret = (c->done != 0);
			break;
		}
	}
	mtx_unlock(&c->mtx);
	return (ret);
}

#define	msecs_to_jiffies(ms)	(((ms) * hz + 999) / 1000)

/* ----------------------------------------------------------------- delay */

#define	udelay(us)		DELAY(us)
#define	mdelay(ms)		DELAY((ms) * 1000)
#define	usleep_range(lo, hi)	DELAY(lo)

/*
 * Deliberately no msleep() shim: FreeBSD's msleep() is a core kernel function
 * with an entirely different signature, and none of the ported files call the
 * Linux one. Shadowing it would break the first header that declares it.
 */

/* --------------------------------------------------------------- iopoll */

/*
 * read_poll_timeout(op, val, cond, sleep_us, timeout_us, sleep_before, args)
 * Returns 0 once cond holds, or -ETIMEDOUT. timeout_us == 0 means no limit,
 * matching Linux.
 */
#define	read_poll_timeout(op, val, cond, sleep_us, timeout_us, sleep_before, ...) \
({									\
	uint64_t __waited = 0;						\
	int __ret = 0;							\
	if (sleep_before && (sleep_us))					\
		DELAY(sleep_us);					\
	for (;;) {							\
		(val) = (op)(__VA_ARGS__);				\
		if (cond)						\
			break;						\
		if ((timeout_us) != 0 &&				\
		    __waited >= (uint64_t)(timeout_us)) {		\
			(val) = (op)(__VA_ARGS__);			\
			if (!(cond))					\
				__ret = -ETIMEDOUT;			\
			break;						\
		}							\
		if (sleep_us) {						\
			DELAY(sleep_us);				\
			__waited += (sleep_us);				\
		} else {						\
			DELAY(1);					\
			__waited += 1;					\
		}							\
	}								\
	__ret;								\
})

/* ----------------------------------------------------------------- bitops */

#ifndef BIT
#define	BIT(n)			(1UL << (n))
#endif
#define	GENMASK(h, l)							\
	(((~0UL) - (1UL << (l)) + 1) & (~0UL >> (BITS_PER_LONG - 1 - (h))))
#define	BITS_PER_LONG		64

#define	__bf_shf(m)		(__builtin_ffsll((long long)(m)) - 1)
#define	FIELD_GET(_mask, _reg)						\
	((typeof(_mask))(((_reg) & (_mask)) >> __bf_shf(_mask)))
#define	FIELD_PREP(_mask, _val)						\
	(((typeof(_mask))(_val) << __bf_shf(_mask)) & (_mask))

/* ------------------------------------------------------------- arithmetic */

/*
 * FreeBSD's <machine/param.h> already has a one-argument ALIGN(p) that aligns
 * a pointer to the stack alignment, so the guard that would normally protect
 * this definition silently kept the wrong one and every one of the 43 call
 * sites in the ported files failed to compile. The vendor's two-argument form
 * has to win here.
 *
 * This is safe only because the system headers are included at the top of
 * THIS file, before the redefinition, and the ported files include nothing
 * else. wave5_freebsd.c likewise includes this header last. Anything that
 * includes a FreeBSD header after this one will get the wrong ALIGN.
 */
#undef ALIGN
#define	ALIGN(x, a)		(((x) + ((a) - 1)) & ~((__typeof__(x))(a) - 1))
#define	PAGE_ALIGN(x)		ALIGN((x), PAGE_SIZE)
#define	DIV_ROUND_UP(n, d)	(((n) + (d) - 1) / (d))
#define	ARRAY_SIZE(a)		(sizeof(a) / sizeof((a)[0]))

#ifndef min_t
#define	min_t(t, a, b)		((t)(a) < (t)(b) ? (t)(a) : (t)(b))
#define	max_t(t, a, b)		((t)(a) > (t)(b) ? (t)(a) : (t)(b))
#endif
#ifndef clamp
#define	clamp(v, lo, hi)	((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#endif

#define	WARN_ONCE(cond, ...)	({					\
	static int __warned;						\
	int __c = !!(cond);						\
	if (__c && !__warned) {						\
		__warned = 1;						\
		printf("wave5: " __VA_ARGS__);				\
	}								\
	__c;								\
})
#define	WARN_ON(cond)		({ int __c = !!(cond); __c; })

/*
 * Linux's WARN() logs and returns the condition; the backtrace it would print
 * is not worth a panic here, so this is a printf. Callers use it for
 * impossible-parameter paths, not for hardware faults.
 */
#define	WARN(cond, ...)		({					\
	int __c = !!(cond);						\
	if (__c)							\
		printf("wave5: " __VA_ARGS__);				\
	__c;								\
})

/*
 * Rate limiting matters: the FIO timeout path that uses this can fire once
 * per register access on a wedged core, and this board's console is a
 * 115200-baud serial line, so an unthrottled message would itself stall the
 * machine. One message per second, and only when debugging is on.
 */
#define	dev_dbg_ratelimited(d, ...)	do {				\
	static int __last;						\
	if (__predict_false(wave5_debug) && ticks - __last >= hz) {	\
		__last = ticks;						\
		device_printf((d)->bsddev, __VA_ARGS__);		\
	}								\
} while (0)

/* ------------------------------------------------------------ Linux lists */

/*
 * The ported hardware files never walk a list; these exist only so the shared
 * struct definitions in wave5-vpuapi.h compile, and for wave5_freebsd.c to
 * track instances.
 */
struct list_head {
	struct list_head *next, *prev;
};

#define	INIT_LIST_HEAD(h)	do {					\
	(h)->next = (h); (h)->prev = (h);				\
} while (0)

static inline void
list_add_tail(struct list_head *n, struct list_head *head)
{

	n->prev = head->prev;
	n->next = head;
	head->prev->next = n;
	head->prev = n;
}

static inline void
list_del(struct list_head *n)
{

	n->prev->next = n->next;
	n->next->prev = n->prev;
	n->next = n->prev = n;
}

static inline int
list_empty(const struct list_head *h)
{

	return (h->next == h);
}

#define	list_entry(ptr, type, member)	__containerof(ptr, type, member)
#define	list_first_entry(h, type, member) list_entry((h)->next, type, member)
#define	list_for_each_entry(p, h, member)				\
	for ((p) = list_entry((h)->next, __typeof__(*(p)), member);	\
	     &(p)->member != (h);					\
	     (p) = list_entry((p)->member.next, __typeof__(*(p)), member))
#define	list_for_each_entry_safe(p, n, h, member)			\
	for ((p) = list_entry((h)->next, __typeof__(*(p)), member),	\
	     (n) = list_entry((p)->member.next, __typeof__(*(p)), member); \
	     &(p)->member != (h);					\
	     (p) = (n), (n) = list_entry((n)->member.next, __typeof__(*(n)), member))

/* ------------------------------------------------------------ id allocator */

/*
 * Instances are few (MAX_NUM_INSTANCE is 32), so a single word bitmap stands
 * in for Linux's struct ida.
 */
struct ida {
	uint32_t	map;
};

#define	ida_init(i)		((i)->map = 0)
#define	ida_destroy(i)		((i)->map = 0)

static inline int
ida_alloc_max(struct ida *ida, unsigned int max, int gfp)
{
	unsigned int i;

	for (i = 0; i <= max && i < 32; i++) {
		if ((ida->map & (1U << i)) == 0) {
			ida->map |= (1U << i);
			return ((int)i);
		}
	}
	return (-ENOSPC);
}

static inline void
ida_free(struct ida *ida, unsigned int id)
{

	if (id < 32)
		ida->map &= ~(1U << id);
}

/* ------------------------------------------------------------------- sram */

/*
 * The vendor driver can put some working buffers in on-chip SRAM via a
 * gen_pool. It is strictly an optimisation -- every caller checks for a NULL
 * pool and carries on without it -- so the pool is left unimplemented and
 * sram_pool stays NULL. Revisit only if decode throughput needs it.
 */
struct gen_pool;

#define	gen_pool_avail(p)		(0UL)
#define	gen_pool_dma_alloc(p, sz, dp)	(NULL)
#define	gen_pool_free(p, a, sz)		do { } while (0)

#endif /* _WAVE5_OSAL_H_ */
