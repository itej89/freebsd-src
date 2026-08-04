/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * Power domain consumer interface for FreeBSD.
 * Follows the same pattern as dev/hwreset/hwreset.h.
 */

#ifndef _DEV_PWRDOM_PWRDOM_H_
#define	_DEV_PWRDOM_PWRDOM_H_

#include "opt_platform.h"
#include <sys/types.h>
#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#endif

typedef struct pwrdom *pwrdom_t;

#ifdef FDT
void pwrdom_register_ofw_provider(device_t provider_dev);
void pwrdom_unregister_ofw_provider(device_t provider_dev);
#endif

int pwrdom_get_by_id(device_t consumer_dev, device_t provider_dev,
    intptr_t id, pwrdom_t *pd);
void pwrdom_release(pwrdom_t pd);

int pwrdom_enable(pwrdom_t pd);
int pwrdom_disable(pwrdom_t pd);
int pwrdom_is_enabled(pwrdom_t pd, bool *value);

#ifdef FDT
int pwrdom_get_by_ofw_idx(device_t consumer_dev, phandle_t node, int idx,
    pwrdom_t *pd);
#endif

#endif /* _DEV_PWRDOM_PWRDOM_H_ */
