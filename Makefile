KMOD=	sume
SRCS=	sume_main.c
SRCS+=  device_if.h bus_if.h pci_if.h

.include <bsd.kmod.mk>
