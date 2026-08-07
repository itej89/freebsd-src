/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 */

#ifndef _RISCV_SIFIVE_CCACHE_H_
#define	_RISCV_SIFIVE_CCACHE_H_

#include <sys/types.h>
#include <vm/vm.h>

void sifive_ccache_flush_range(vm_paddr_t paddr, size_t len);
void sifive_ccache_flush_all(void);
bool sifive_ccache_is_available(void);
uint64_t sifive_ccache_uncached_offset(void);

#endif /* _RISCV_SIFIVE_CCACHE_H_ */
