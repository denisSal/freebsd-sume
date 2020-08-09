KMOD=	if_sume
SRCS=	if_sume.c
SRCS+=  device_if.h bus_if.h pci_if.h
MAN=	sume.4

.include <bsd.kmod.mk>
