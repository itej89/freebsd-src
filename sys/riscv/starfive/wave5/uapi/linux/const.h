/*
 * Minimal stand-in for linux/const.h, for the imported V4L2 uapi headers.
 * Only the bit helpers they actually reference.
 */
#ifndef _WAVE5_UAPI_LINUX_CONST_H_
#define	_WAVE5_UAPI_LINUX_CONST_H_

#define	__AC(X, Y)	(X##Y)
#define	__CONST(X, Y)	__AC(X, Y)

#define	_UL(x)		(__AC(x, UL))
#define	_ULL(x)		(__AC(x, ULL))
#define	_AT(T, X)	((T)(X))

#define	_BITUL(x)	(_UL(1) << (x))
#define	_BITULL(x)	(_ULL(1) << (x))

#endif
