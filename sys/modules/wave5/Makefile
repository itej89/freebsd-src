# StarFive JH7110 Wave511 video decoder.
#
# Built as a module rather than compiled into the kernel so the driver can be
# reloaded without a reboot during bring-up; the JH7110 display and GPU work
# the same way.
#
# wave5-hw.c, wave5-vpuapi.c and wave5-vdi.c are ported Chips&Media/Linux
# sources kept as close to upstream as possible, so that fixes can still be
# diffed against the vendor tree. They are not written to FreeBSD's warning
# settings, and reformatting them to suit would destroy that diffability, so
# the warnings they trip are switched off for those files alone -- not for
# wave5_freebsd.c, which is ours and is held to the normal standard.

.PATH:	${SRCTOP}/sys/riscv/starfive/wave5

KMOD=	wave5

SRCS=	wave5_freebsd.c \
	wave5-vdi.c \
	wave5-vpuapi.c \
	wave5-hw.c

SRCS+=	device_if.h bus_if.h ofw_bus_if.h opt_platform.h \
	clkdev_if.h clknode_if.h hwreset_if.h pwrdom_if.h

CFLAGS+= -I${SRCTOP}/sys/riscv/starfive/wave5

# The clock, reset and power-domain headers gate their OF accessors on FDT,
# and a module's generated opt_platform.h is empty. Without this, clk_enable()
# and friends are merely implicitly declared and the driver would link against
# nothing. riscv is always FDT, so defining it unconditionally is correct.
CFLAGS+= -DFDT

VENDOR_CWARNFLAGS= -Wno-unused-parameter -Wno-sign-compare \
	-Wno-missing-prototypes -Wno-cast-qual -Wno-unused-but-set-variable \
	-Wno-pointer-arith

CWARNFLAGS.wave5-hw.c+=		${VENDOR_CWARNFLAGS}
CWARNFLAGS.wave5-vpuapi.c+=	${VENDOR_CWARNFLAGS}
CWARNFLAGS.wave5-vdi.c+=	${VENDOR_CWARNFLAGS}

.include <bsd.kmod.mk>
