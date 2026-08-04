.PATH:	${.CURDIR}

KMOD=	lkpi_platform_test
SRCS=	lkpi_platform_test.c

SRCS+=	device_if.h bus_if.h
SRCS+=	ofw_bus_if.h ofw_if.h pwrdom_if.h
SRCS+=	opt_fdt.h opt_platform.h

CFLAGS+= ${LINUXKPI_INCLUDES}

.include <bsd.kmod.mk>
