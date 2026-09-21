/*
 * Kernel stand-in for the linux/types.h that multimedia/v4l_compat installs
 * for userland (which just pulls in <stdint.h>). In the kernel the same fixed
 * width types come from <sys/types.h>, so this only has to exist and be empty
 * of anything that would conflict.
 */
#ifndef _WAVE5_UAPI_LINUX_TYPES_H_
#define	_WAVE5_UAPI_LINUX_TYPES_H_
#include <sys/types.h>
#endif
