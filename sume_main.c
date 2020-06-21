/*-
 * Copyright (c) 2020 Denis Salopek
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/time.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/malloc.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <machine/bus.h>
#include <sys/rman.h>

#include <sys/socket.h>
#include <sys/sockio.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>

#include <sys/endian.h>
#include <sys/interrupt.h>

#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/types.h>
#include <machine/atomic.h>

#include "adapter.h"

#define PCI_VENDOR_ID_XILINX 0x10ee

/* SUME bus driver interface */
static int sume_probe(device_t);
static int sume_attach(device_t);
static int sume_detach(device_t);
static device_method_t sume_methods[] = {
	DEVMETHOD(device_probe,		sume_probe),
	DEVMETHOD(device_attach,	sume_attach),
	DEVMETHOD(device_detach,	sume_detach),

	DEVMETHOD_END
};
static driver_t sume_driver = {
	"sume",
	sume_methods,
	sizeof(struct sume_adapter)
};

MALLOC_DECLARE(M_SUME);
MALLOC_DEFINE(M_SUME, "sume", "NetFPGA SUME device driver");

static unsigned int sume_nports __read_mostly = SUME_PORTS_MAX;
TUNABLE_INT("sume.nports", &sume_nports);

static int mod_event(module_t, int, void *);
void sume_intr_handler(void *);
static int sume_intr_filter(void *);
static int
sume_if_ioctl(struct ifnet *, unsigned long, caddr_t);
static int sume_riffa_fill_sg_buf(struct sume_adapter *,
    struct riffa_chnl_dir *, enum dma_data_direction,
    unsigned long long);

static inline unsigned int read_reg(struct sume_adapter *, int);
static inline void write_reg(struct sume_adapter *, int, unsigned int);

struct {
	uint16_t device;
	char *desc;
} sume_pciids[] = {
	{0x7028, "NetFPGA SUME"},
};

static inline unsigned int
read_reg(struct sume_adapter *adapter, int offset)
{
	return (bus_space_read_4(adapter->bt, adapter->bh, offset << 2));
}

static inline void
write_reg(struct sume_adapter *adapter, int offset, unsigned int val)
{
	bus_space_write_4(adapter->bt, adapter->bh, offset << 2, val);
}

static int
sume_probe(device_t dev)
{
	int i;
	uint16_t v = pci_get_vendor(dev);
	uint16_t d = pci_get_device(dev);

	if (v != PCI_VENDOR_ID_XILINX)
		return (ENXIO);

	for (i = 0; i < nitems(sume_pciids); i++) {
		if (d == sume_pciids[i].device) {
			device_set_desc(dev, sume_pciids[i].desc);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

void
sume_intr_handler(void *arg)
{
	printf("Interrupt handler!\n");
	struct sume_adapter *adapter = arg;
	unsigned int vect, vect0, vect1;
	int i, error, loops, len;
	device_t dev = adapter->dev;

	mtx_lock_spin(&adapter->lock); // lock here and/or in the filter?
	vect0 = adapter->vect0;
	vect1 = adapter->vect1;
	mtx_unlock_spin(&adapter->lock); // lock here and/or in the filter?

	printf("Got vect0: %d\n", vect0);
	printf("Got vect1: %d\n", vect1);

	/*
	 * We only have one interrupt for all channels and no way
	 * to quickly lookup for which channel(s) we got an interrupt?
	 */
	for (i = 0; i < adapter->num_chnls; i++) {
		if (i < 6)
			vect = vect0;
		else
			vect = vect1;

		//SUME_LOCK_TX(adapter, i, flags);
		mtx_lock_spin(&adapter->send[i]->lock);
		loops = 0;
		while ((vect & ((1 << ((5 * i) + 3)) | (1 << ((5 * i) + 4)))) &&
		    loops <= 5) {
			//if (sume_debug_level)
				device_printf(dev, "%s: TX ch %d state %u "
				    "vect=0x%08x\n", __func__, i,
				    adapter->send[i]->state, vect);
			switch (adapter->send[i]->state) {
			case SUME_RIFFA_CHAN_STATE_IDLE:
				break;
			case SUME_RIFFA_CHAN_STATE_READY:
				if (vect & (1 << ((5 * i) + 3))) {
					adapter->send[i]->state =
					    SUME_RIFFA_CHAN_STATE_READ;
					vect &= ~(1 << ((5 * i) + 3));
				} else {
					device_printf(dev, "%s: ch %d unexpected "
					    "interrupt in send+3 state %u: "
					    "vect=0x%08x\n", __func__,
					    i, adapter->send[i]->state, vect);
					adapter->send[i]->flags |=
					    SUME_CHAN_STATE_RECOVERY_FLAG;
				}
				break;
			case SUME_RIFFA_CHAN_STATE_READ:
				if (vect & (1 << ((5 * i) + 4))) {

					adapter->send[i]->state =
					    SUME_RIFFA_CHAN_STATE_LEN;

					len = read_reg(adapter,
					    RIFFA_CHNL_REG(i,
					    RIFFA_RX_TNFR_LEN_REG_OFF));
					/*
					 * XXX-BZ should compare length with
					 * expected amount of data transfered;
					 * only on match advance state?
					 */
					if (i ==
					    SUME_RIFFA_CHANNEL_DATA(adapter))
						adapter->send[i]->state =
						    SUME_RIFFA_CHAN_STATE_IDLE;
					else if (i ==
					    SUME_RIFFA_CHANNEL_REG(adapter)) {
						//wake_up_interruptible(
						    //&adapter->send[i]->waitq);
						//mtx_unlock_spin(&adapter->send[i]->send_sleep);
						wakeup(&adapter->send[i]->event);
						printf("Wake up send thread!\n");
					} else {
						device_printf(dev, "%s: "
						    "interrupt on ch %d "
						    "unexpected in send+4 "
						    "state %u: vect=0x%08x\n",
						    __func__, i,
						    adapter->send[i]->state,
						    vect);
						adapter->send[i]->flags |=
						    SUME_CHAN_STATE_RECOVERY_FLAG;
					}
					vect &= ~(1 << ((5 * i) + 4));
				} else {
					device_printf(dev, "%s: ch %d unexpected "
					    "interrupt in send+4 state %u: "
					    "vect=0x%08x\n", __func__,
					    i, adapter->send[i]->state, vect);
					adapter->send[i]->flags |=
					    SUME_CHAN_STATE_RECOVERY_FLAG;
				}
				break;
			case SUME_RIFFA_CHAN_STATE_LEN:
				break;
			default:
				printf("WARNON!\n");
				//WARN_ON(1);
			}
			loops++;
		}

		if ((vect & ((1 << ((5 * i) + 3)) | (1 << ((5 * i) + 4)))) &&
		    ((adapter->send[i]->flags & SUME_CHAN_STATE_RECOVERY_FLAG)
		    != 0))
			device_printf(dev, "%s: ignoring vect=0x%08x "
			    "during TX; not in recovery; state=%d loops=%d\n",
			    __func__, vect, adapter->send[i]->state, loops);
		//SUME_UNLOCK_TX(adapter, i, flags);
		mtx_unlock_spin(&adapter->send[i]->lock);

		//SUME_LOCK_RX(adapter, i, flags);
		mtx_lock_spin(&adapter->recv[i]->lock);
		loops = 0;
		while ((vect & ((1 << ((5 * i) + 0)) | (1 << ((5 * i) + 1)) |
		    (1 << ((5 * i) + 2)))) && loops < 5) {
			//if (sume_debug_level)
				device_printf(dev, "%s: RX ch %d state %u "
				    "vect=0x%08x\n", __func__, i,
				    adapter->recv[i]->state, vect);
			switch (adapter->recv[i]->state) {
			case SUME_RIFFA_CHAN_STATE_IDLE:
				if (vect & (1 << ((5 * i) + 0))) {
					unsigned long max_ptr;

					/* Clear recovery state. */
					adapter->recv[i]->flags &=
					    ~SUME_CHAN_STATE_RECOVERY_FLAG;

					/* Get offset and length. */
					adapter->recv[i]->offlast = read_reg(
					    adapter, RIFFA_CHNL_REG(i,
					    RIFFA_TX_OFFLAST_REG_OFF));
					adapter->recv[i]->len =
					    read_reg(adapter, RIFFA_CHNL_REG(i,
						RIFFA_TX_LEN_REG_OFF));

					printf("Got offlast = %d\n", adapter->recv[i]->offlast);
					printf("Got len = %d\n", adapter->recv[i]->len);
					/* Boundary checks. */
					max_ptr = (unsigned long)
					    ((char *)adapter->recv[i]->buf_addr + SUME_RIFFA_OFFSET(
						adapter->recv[i]->offlast) +
					    SUME_RIFFA_LEN(
						adapter->recv[i]->len) - 1);
					printf("Max ptr = 0x%lx\n", max_ptr);
					if (max_ptr < (unsigned long)
					    adapter->recv[i]->buf_addr) {
						device_printf(dev, "%s: receive "
						    "buffer wrap-around "
						    "overflow.\n", __func__);
						/* XXX-BZ recover? */
					}
					if ((SUME_RIFFA_OFFSET(
					    adapter->recv[i]->offlast) +
					    SUME_RIFFA_LEN(
					    adapter->recv[i]->len)) >
					    adapter->recv[i]->bouncebuf_len) {
						device_printf(dev, "%s: receive "
						    "buffer too small.\n",
						    __func__);
						/* XXX-BZ recover? */
					}

					printf("Building sg map...\n");
					/* Build and load S/G map. */
					error = sume_riffa_fill_sg_buf(adapter,
					    adapter->recv[i], DMA_FROM_DEVICE,
					    SUME_RIFFA_LEN(
					    adapter->recv[i]->len));
					if (error != 0) {
						device_printf(dev, "%s: Failed "
						    "to build S/G map.\n",
						    __func__);
						/* XXX-BZ recover? */
					}
					printf("Built.\n");
					bus_dmamap_sync(adapter->recv[i]->my_tag, adapter->recv[i]->my_map, BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
					write_reg(adapter,
					    RIFFA_CHNL_REG(i,
						RIFFA_TX_SG_ADDR_LO_REG_OFF),
					    (adapter->recv[i]->buf_hw_addr &
						0xFFFFFFFF));
					write_reg(adapter,
					    RIFFA_CHNL_REG(i,
						RIFFA_TX_SG_ADDR_HI_REG_OFF),
					    ((adapter->recv[i]->buf_hw_addr >>
						32) & 0xFFFFFFFF));
					write_reg(adapter,
					    RIFFA_CHNL_REG(i,
						RIFFA_TX_SG_LEN_REG_OFF),
					    4 * adapter->recv[i]->num_sg);
					bus_dmamap_sync(adapter->recv[i]->my_tag, adapter->recv[i]->my_map, BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

					adapter->recv[i]->state =
					    SUME_RIFFA_CHAN_STATE_READY;
					vect &= ~(1 << ((5 * i) + 0));
				} else {
					device_printf(dev, "%s: ch %d unexpected "
					    "interrupt in recv+0 state %u: "
					    "vect=0x%08x\n", __func__,
					    i, adapter->recv[i]->state, vect);
					adapter->recv[i]->flags |=
					    SUME_CHAN_STATE_RECOVERY_FLAG;
				}
				break;
			case SUME_RIFFA_CHAN_STATE_READY:
				if (vect & (1 << ((5 * i) + 1))) {
					adapter->recv[i]->state =
					    SUME_RIFFA_CHAN_STATE_READ;
					vect &= ~(1 << ((5 * i) + 1));
				} else {
					device_printf(dev, "%s: ch %d unexpected "
					    "interrupt in recv+1 state %u: "
					    "vect=0x%08x\n", __func__,
					    i, adapter->recv[i]->state, vect);
					adapter->recv[i]->flags |=
					    SUME_CHAN_STATE_RECOVERY_FLAG;
				}
				break;
			case SUME_RIFFA_CHAN_STATE_READ:
				if (vect & (1 << ((5 * i) + 2))) {
					len = read_reg(adapter,
					    RIFFA_CHNL_REG(i,
						RIFFA_TX_TNFR_LEN_REG_OFF));
					/* XXX-BZ compare to expected len? */

					/*
					 * Remember, len and recv[i]->len
					 * are words.
					 */
					if (i ==
					   SUME_RIFFA_CHANNEL_DATA(adapter)) {
						//error = sume_rx_build_skb(
						    //adapter, i, len << 2);
						printf("build skb before recv\n");
						adapter->recv[i]->state =
						    SUME_RIFFA_CHAN_STATE_IDLE;
					} else if (i ==
					    SUME_RIFFA_CHANNEL_REG(adapter)) {
						//wake_up_interruptible(
						   //&adapter->recv[i]->waitq);
						//mtx_unlock_spin(&adapter->recv[i]->recv_sleep);
						wakeup(&adapter->recv[i]->event);
						printf("Wake up recv thread!\n");
					} else {
						device_printf(dev, "%s: "
						    "interrupt on ch %d "
						    "unexpected in recv+2 "
						    "state %u: vect=0x%08x\n",
						    __func__, i,
						    adapter->recv[i]->state,
						    vect);
						adapter->recv[i]->flags |=
						    SUME_CHAN_STATE_RECOVERY_FLAG;
					}
					vect &= ~(1 << ((5 * i) + 2));

				} else {
					device_printf(dev, "%s: ch %d unexpected "
					    "interrupt in recv+2 state %u: "
					    "vect=0x%08x\n", __func__,
					    i, adapter->recv[i]->state, vect);
					adapter->recv[i]->flags |=
					    SUME_CHAN_STATE_RECOVERY_FLAG;
				}
				break;
			case SUME_RIFFA_CHAN_STATE_LEN:
				break;
			default:
				printf("WARNON!\n");
				//WARN_ON(1);
			}
			loops++;
		}

		if ((vect & ((1 << ((5 * i) + 0)) | (1 << ((5 * i) + 1)) |
		    (1 << ((5 * i) + 2)))) &&
		    ((adapter->recv[i]->flags & SUME_CHAN_STATE_RECOVERY_FLAG)
		    != 0))
			device_printf(dev, "%s: ignoring vect=0x%08x "
			    "during RX; not in recovery; state=%d, loops=%d\n",
			    __func__, vect, adapter->recv[i]->state, loops);
		//SUME_UNLOCK_RX(adapter, i, flags);
		mtx_unlock_spin(&adapter->recv[i]->lock);
	}
}

static int
sume_intr_filter(void *arg)
{
	printf("Interrupt filter!\n");
	struct sume_adapter *adapter = arg;
	unsigned int vect0, vect1;

	/*
	 * Ignore early interrupts from RIFFA given we cannot disable interrupt
	 * generation.
	 */
	if (atomic_load_int(&adapter->running) == 0)
		return (FILTER_STRAY);

	//SUME_LOCK(adapter, flags);
	/* XXX-BZ We would turn interrupt generation off. */
	mtx_lock_spin(&adapter->lock); // lock here and/or in the handler?

	vect0 = read_reg(adapter, RIFFA_IRQ_REG0_OFF);
	/* XXX-BZ magic number */
	if((vect0 & 0xC0000000) != 0) {
		printf("WARNON!\n");
		return (FILTER_STRAY); // nor really stray, only warning?
	}
	if (adapter->num_chnls > 6) {
		vect1 = read_reg(adapter, RIFFA_IRQ_REG1_OFF);
		/* XXX-BZ magic number */
		if((vect1 & 0xC0000000) != 0) {
			printf("WARNON!\n");
			return (FILTER_STRAY); // nor really stray, only warning?
		}
	} else
		vect1 = 0;

	//SUME_UNLOCK(adapter, flags);
	/* XXX-BZ We would turn interrupt generation back on. */
	adapter->vect0 = vect0;
	adapter->vect1 = vect1;
	mtx_unlock_spin(&adapter->lock); // lock here and/or in the handler?

	return (FILTER_SCHEDULE_THREAD);
}

static int
sume_probe_riffa_pci(struct sume_adapter *adapter)
{
	device_t dev = adapter->dev;
	int error, count, capmem;
	uint32_t reg, devctl, linkctl;

	adapter->rid = PCIR_BAR(0);
	adapter->bar0_addr = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &adapter->rid, RF_ACTIVE);
	if (adapter->bar0_addr == NULL) {
		device_printf(dev, "Unable to allocate bus resource: bar0_addr\n");
		return (ENXIO);
	}
	adapter->bt = rman_get_bustag(adapter->bar0_addr);
	adapter->bh = rman_get_bushandle(adapter->bar0_addr);
	adapter->bar0_len = rman_get_size(adapter->bar0_addr);
	if (adapter->bar0_len != 1024) {        /* XXX-BZ magic number */
		device_printf(dev, "%s: bar0_len %lu != 1024\n", __func__,
				adapter->bar0_len);
		return (ENXIO);
        }

	count = pci_msi_count(dev);
	error = pci_alloc_msi(dev, &count);
	if (error) {
		device_printf(dev, "Unable to allocate bus resource: PCI MSI\n");
		return (error);
	}

	adapter->irq.rid = 1; // should be 1, thus says pci_alloc_msi()
	adapter->irq.res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &adapter->irq.rid, RF_SHAREABLE | RF_ACTIVE);
	if (adapter->irq.res == NULL) {
		device_printf(dev, "Unable to allocate bus resource: IRQ memory\n");
		return (ENXIO);
	}

	error = bus_setup_intr(dev, adapter->irq.res, INTR_MPSAFE | INTR_TYPE_NET, sume_intr_filter, sume_intr_handler, adapter, &adapter->irq.tag);
	if (error) {
		device_printf(dev, "failed to setup interrupt for rid %d, name %s: %d\n", adapter->irq.rid, "SUME_INTR", error);
		return (ENXIO);
	} else
		bus_describe_intr(dev, adapter->irq.res, adapter->irq.tag, "%s", "SUME_INTR");

	if (pci_find_cap(dev, PCIY_EXPRESS, &capmem) != 0) {
		device_printf(dev, "%s: pcie_capability_read_dword PCI_EXP_DEVCTL error\n", __func__);
		return (ENXIO);
	}

	devctl = pci_read_config(dev, capmem + PCIER_DEVICE_CTL, 2);
	pci_write_config(dev, capmem + PCIER_DEVICE_CTL, (devctl | PCIEM_CTL_EXT_TAG_FIELD), 2);

	devctl = pci_read_config(dev, capmem + PCIER_DEVICE_CTL2, 2);
	pci_write_config(dev, capmem + PCIER_DEVICE_CTL2, (devctl | PCIEM_CTL2_ID_ORDERED_REQ_EN), 2);

	linkctl = pci_read_config(dev, capmem + PCIER_LINK_CTL, 2);
	pci_write_config(dev, capmem + PCIER_LINK_CTL, (linkctl | PCIEM_LINK_CTL_RCB), 2);

	reg = read_reg(adapter, RIFFA_INFO_REG_OFF);
	adapter->num_chnls =    SUME_RIFFA_CHANNELS(reg & 0xf);
	adapter->num_sg =       RIFFA_SG_ELEMS * ((reg >> 19) & 0xf);
	adapter->sg_buf_size =  RIFFA_SG_BUF_SIZE * ((reg >> 19) & 0xf);

	error = ENODEV;
	/* Check bus master is enabled. */
	if (((reg >> 4) & 0x1) != 1) {
		device_printf(dev, "%s: bus master not enabled: %d\n",
		    __func__, ((reg >> 4) & 0x1));
		return (error);
	}
	/* Check link parameters are valid. */
	if (((reg >> 5) & 0x3f) == 0 || ((reg >> 11) & 0x3) == 0) {
		device_printf(dev, "%s: link parameters not valid: %d %d\n",
		    __func__, ((reg >> 5) & 0x3f), ((reg >> 11) & 0x3));
		return (error);
	}
	/* Check # of channels are within valid range. */
	if ((reg & 0xf) == 0 || (reg & 0xf) > RIFFA_MAX_CHNLS) {
		device_printf(dev, "%s: number of channels out of range: %d\n",
		    __func__, (reg & 0xf));
		return (error);
	}
	/* Check bus width. */
	if (((reg >> 19) & 0xf) == 0 ||
	    ((reg >> 19) & 0xf) > RIFFA_MAX_BUS_WIDTH_PARAM) {
		device_printf(dev, "%s: bus width out f range: %d\n",
		    __func__, ((reg >> 19) & 0xf));
		return (error);
	}

	device_printf(dev, "[riffa] # of channels: %d\n",
	    (reg & 0xf));
	device_printf(dev, "[riffa] bus interface width: %d\n",
	    (((reg >> 19) & 0xf) << 5));
	device_printf(dev, "[riffa] bus master enabled: %d\n",
	    ((reg >> 4) & 0x1));
	device_printf(dev, "[riffa] negotiated link width: %d\n",
	    ((reg >> 5) & 0x3f));
	device_printf(dev, "[riffa] negotiated rate width: %d MTs\n",
	    ((reg >> 11) & 0x3) * 2500);
	device_printf(dev, "[riffa] max downstream payload: %d B\n",
	    (128 << ((reg >> 13) & 0x7)));
	device_printf(dev, "[riffa] max upstream payload: %d B\n",
	    (128 << ((reg >> 16) & 0x7)));

	return (0);

}

static void
sume_if_init(void *arg)
{
	printf("DEBUG: %s\n", __func__);
}

/* Helper functions. */
static int
sume_riffa_fill_sg_buf(struct sume_adapter *adapter,
    struct riffa_chnl_dir *p, enum dma_data_direction dir,
    unsigned long long len)
{
	uint32_t *sgtablep;

	sgtablep = (uint32_t *) p->buf_addr;

	sgtablep[0] = (p->buf_hw_addr + 3*sizeof(uint32_t)) & 0xffffffff;
	sgtablep[1] = ((p->buf_hw_addr + 3*sizeof(uint32_t)) >> 32) & 0xffffffff;
	//sgtablep[2] = 4;
	sgtablep[2] = len;
	p->num_sg = 1;

	return (0);
}

/* Register read/write. */
static int
sume_reg_wr_locked(struct sume_adapter *adapter, int i)
{
	int error, last, offset;
	device_t dev = adapter->dev;

	/* Let the FPGA know about the transfer. */
	offset = 0;
	last = 1;
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_OFFLAST_REG_OFF),
	    ((offset << 1) | (last & 0x01)));
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_LEN_REG_OFF),
	    adapter->send[i]->len);		/* words */

	/* Fill the S/G map. */
	error = sume_riffa_fill_sg_buf(adapter,
	    adapter->send[i], DMA_TO_DEVICE,
	    SUME_RIFFA_LEN(adapter->send[i]->len));
	if (error != 0) {
		device_printf(dev, "%s: failed to map S/G buffer\n", __func__);
		return (EFAULT);
	}

	/* Update the state before intiating the DMA to avoid races. */
	adapter->send[i]->state = SUME_RIFFA_CHAN_STATE_READY;

	bus_dmamap_sync(adapter->send[i]->my_tag, adapter->send[i]->my_map, BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	/* DMA. */
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_SG_ADDR_LO_REG_OFF),
	    (adapter->send[i]->buf_hw_addr & 0xFFFFFFFF));
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_SG_ADDR_HI_REG_OFF),
	    ((adapter->send[i]->buf_hw_addr >> 32) & 0xFFFFFFFF));
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_SG_LEN_REG_OFF),
	    4 * adapter->send[i]->num_sg);
	bus_dmamap_sync(adapter->send[i]->my_tag, adapter->send[i]->my_map, BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	return (0);
}

/*
 * Request a register read or write (depending on strb).
 * If strb is set (0x1f) this will result in a register write,
 * otherwise this will result in a register read request at the given
 * address and the result will need to be DMAed back.
 */
static int
sume_initiate_reg_write(struct sume_port *sume_port, struct sume_ifreq *sifr,
    uint32_t strb)
{
	struct sume_adapter *adapter;
	uint32_t *p32;
	int error = 0, i;

	adapter = sume_port->adapter;

	/*
	 * 1. Make sure the channel is free;  otherwise return EBUSY.
	 * 2. Prepare the memory in the bounce buffer (which we always
	 *    use for regs).
	 * 3. Start the DMA process.
	 * 4. Sleep and wait for result and return success or error.
	 */
	i = SUME_RIFFA_CHANNEL_REG(sume_port);
	//SUME_LOCK(adapter, flags);
	//SUME_LOCK_TX(adapter, i, flags);
	mtx_lock_spin(&adapter->send[i]->send_sleep);

	if (adapter->send[i]->state != SUME_RIFFA_CHAN_STATE_IDLE) {
		//SUME_UNLOCK_TX(adapter, i, flags);
		//SUME_UNLOCK(adapter, flags);
		mtx_unlock_spin(&adapter->send[i]->send_sleep);
		return (EBUSY);
	}

	//p32 = (uint32_t *)adapter->send[i]->bouncebuf + 3;
	p32 = (uint32_t *)adapter->send[i]->buf_addr + 3;
	*p32++ = htole32(sifr->addr);
	*p32++ = htole32(sifr->val);
	/* Tag to indentify request. */
	*p32++ = htole32(++adapter->send[i]->rtag);
	*p32 = htole32(strb);		/* This is STRB; write a val. */
	adapter->send[i]->len = 4;		/* words */

	error = sume_reg_wr_locked(adapter, i);
	if (error) {
		//SUME_UNLOCK_TX(adapter, i, flags);
		//SUME_UNLOCK(adapter, flags);
		mtx_unlock_spin(&adapter->send[i]->send_sleep);
		return (EFAULT);
	}

	/*
	 * We have to drop the lock to void deadlocks. I wish Linux had versions
	 * like sleep(9) on BSD, which take the mtx, so we can check under lock.
	 */
	//SUME_UNLOCK_TX(adapter, i, flags);
	//SUME_UNLOCK(adapter, flags);

	/* Timeout after 1s. */
	//error = wait_event_interruptible_timeout(adapter->send[i]->waitq,
	//    adapter->send[i]->state == SUME_RIFFA_CHAN_STATE_LEN, HZ);
	error = msleep_spin(&adapter->send[i]->event, &adapter->send[i]->send_sleep, "Waiting recv to finish", 1 * hz);

	if (error == EWOULDBLOCK) {
		device_printf(adapter->dev, "%s: wait error: %d\n", __func__, error);
		mtx_unlock_spin(&adapter->send[i]->send_sleep);
		return (EWOULDBLOCK);
	}

	/* This was a write so we are done; were interrupted, or timed out. */
	if (strb != 0x00 || error != 0 || error == EWOULDBLOCK) {
		//SUME_LOCK(adapter, flags);
		//SUME_LOCK_TX(adapter, i, flags);
		adapter->send[i]->state = SUME_RIFFA_CHAN_STATE_IDLE;
		//SUME_UNLOCK_TX(adapter, i, flags);
		//SUME_UNLOCK(adapter, flags);
		if (strb == 0x00)
			error = EWOULDBLOCK;
		else
			error = 0;
	} else
		error = 0;

	/*
	 * For read requests we will update state once we are done
	 * having read the result to avoid any two outstanding
	 * transactions, or we need a queue and validate tags,
	 * which is a lot of work for a low priority, infrequent
	 * event.
	 */

	mtx_unlock_spin(&adapter->send[i]->send_sleep);

	return (error);
}

static int
sume_read_reg_result(struct sume_port *sume_port, struct sume_ifreq *sifr)
{
	struct sume_adapter *adapter;
	uint32_t *p32;
	int error, i;
	device_t dev;

	adapter = sume_port->adapter;
	dev = adapter->dev;

	/*
	 * 0. Sleep waiting for result if needed (unless condition is
	 *    true already).
	 * 2. Read DMA results.
	 * 3. Update state on *TX* to IDLE to allow next read to start.
	 */
	i = SUME_RIFFA_CHANNEL_REG(sume_port);

	/* We only need to be woken up at the end of the transaction. */
	/* Timeout after 1s. */
	//error = wait_event_interruptible_timeout(adapter->recv[i]->waitq,
	    //adapter->recv[i]->state == SUME_RIFFA_CHAN_STATE_READ, HZ);
	mtx_lock_spin(&adapter->recv[i]->recv_sleep);
	bus_dmamap_sync(adapter->recv[i]->my_tag, adapter->recv[i]->my_map, BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	error = msleep_spin(&adapter->recv[i]->event, &adapter->recv[i]->recv_sleep, "Waiting transaction to finish", 1 * hz);

	if (error == EWOULDBLOCK) {
		device_printf(dev, "%s: wait error: %d\n", __func__, error);
		mtx_unlock_spin(&adapter->recv[i]->recv_sleep);
		return (EWOULDBLOCK);
	}
	bus_dmamap_sync(adapter->recv[i]->my_tag, adapter->recv[i]->my_map, BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	//SUME_LOCK(adapter, flags);
	//SUME_LOCK_RX(adapter, i, flags);

	/*
	 * Read reply data and validate address and tag.
	 * Note: we do access the send side without lock but the state
	 * machine does prevent the data from changing.
	 */
	//p32 = (uint32_t *)adapter->recv[i]->bouncebuf;
	p32 = (uint32_t *)adapter->recv[i]->buf_addr + 3;
	if (le32toh(*(p32+2)) != adapter->send[i]->rtag) {
		device_printf(dev, "%s: rtag error: 0x%08x 0x%08x\n", __func__,
		    le32toh(*(p32+2)), adapter->send[i]->rtag);
	}
	sifr->val = le32toh(*(p32+1));
#if 0
	(*p32 >> 5) & 0x03;		/* Response STRB? */
#endif
	adapter->recv[i]->state = SUME_RIFFA_CHAN_STATE_IDLE;
	//SUME_UNLOCK_RX(adapter, i, flags);
	mtx_unlock_spin(&adapter->recv[i]->recv_sleep);

	/* We are done. */
	//SUME_LOCK_TX(adapter, i, flags);
	mtx_lock_spin(&adapter->recv[i]->lock);
	adapter->send[i]->state = SUME_RIFFA_CHAN_STATE_IDLE;
	mtx_unlock_spin(&adapter->recv[i]->lock);
	//SUME_UNLOCK_TX(adapter, i, flags);
	//SUME_UNLOCK(adapter, flags);

	return (0);
}

static int
sume_if_ioctl(struct ifnet *ifp, unsigned long cmd, caddr_t data)
{
	struct sume_port *sume_port;
	struct ifreq *ifr = (struct ifreq *)data;
	struct sume_ifreq sifr;
	int error = 0;
	struct sume_adapter *adapter;

	sume_port = ifp->if_softc;
	if (sume_port == NULL || sume_port->adapter == NULL)
		return (EINVAL);

	adapter = sume_port->adapter;

	switch (cmd) {
	case SUME_IOCTL_CMD_WRITE_REG:
		error = copyin(ifr_data_get_ptr(ifr), &sifr, sizeof(sifr));
		if (error) {
			error = EINVAL;
			break;
		}
		error = sume_initiate_reg_write(sume_port, &sifr, 0x1f);
		break;

	case SUME_IOCTL_CMD_READ_REG:
		error = copyin(ifr_data_get_ptr(ifr), &sifr, sizeof(sifr));
		if (error) {
			error = EINVAL;
			break;
		}

		error = sume_initiate_reg_write(sume_port, &sifr, 0x00);
		if (error)
			break;

		error = sume_read_reg_result(sume_port, &sifr);
		if (error)
			break;

		error = copyout(&sifr, ifr_data_get_ptr(ifr), sizeof(sifr));
		if (error)
			error = EINVAL;

		break;

	default:
		error = ENOTSUP;
		break;
	}

	return (error);
}

static int
sume_netdev_alloc(struct sume_adapter *adapter, unsigned int port)
{
	struct ifnet *ifp;	
	struct sume_port *sume_port = &adapter->port[port];	
	device_t dev = adapter->dev;

	ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
                device_printf(dev, "Cannot allocate ifnet\n");
                return (ENOMEM);
        }

	adapter->netdev[port] = ifp;
	ifp->if_softc = sume_port;

	if_initname(ifp, SUME_ETH_DEVICE_NAME, port);
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST; // XXX

	ifp->if_init = sume_if_init;
	ifp->if_ioctl = sume_if_ioctl;

	sume_port->adapter = adapter;
	sume_port->netdev = ifp;
	sume_port->port = port;
	sume_port->riffa_channel = SUME_RIFFA_CHANNEL_DATA(sume_port);

	adapter->netdev[port] = ifp;

	uint8_t hw_addr[ETHER_ADDR_LEN] = DEFAULT_ETHER_ADDRESS;
	hw_addr[ETHER_ADDR_LEN-1] = port;
	ether_ifattach(ifp, hw_addr);

	return (0);
}

static void
callback_dma(void *arg, bus_dma_segment_t *segs, int nseg, int err)
{
        if (err) {
                printf("DMA error\n");
                return;
        }

        *(bus_addr_t *) arg = segs[0].ds_addr;
}

static int
sume_probe_riffa_buffer(const struct sume_adapter *adapter, 
		struct riffa_chnl_dir ***p, const char *dir)
{
	struct riffa_chnl_dir **rp;
	bus_addr_t hw_addr;
	int error, i;
	device_t dev = adapter->dev;

	error = ENOMEM;
	*p = (struct riffa_chnl_dir **) malloc(adapter->num_chnls * sizeof(struct riffa_chnl_dir *), M_SUME, M_ZERO | M_WAITOK);
	if (*p == NULL) {
		device_printf(dev, "%s: malloc(%s) failed.\n", __func__, dir);
		return (error);
	}

	rp = *p;
	/* Allocate the chnl_dir structs themselves. */
	for (i = 0; i < adapter->num_chnls; i++) {
		/* One direction. */
		rp[i] = (struct riffa_chnl_dir *)
		    malloc(sizeof(struct riffa_chnl_dir), M_SUME, M_ZERO | M_WAITOK);
		if (rp[i] == NULL) {
			device_printf(dev, "%s: malloc(%s[%d]) riffa_chnl_dir "
			    "failed.\n", __func__, dir, i);
			return (error);
		}

		int err = bus_dma_tag_create(bus_get_dma_tag(dev),
						4, 0,
						BUS_SPACE_MAXADDR,
						BUS_SPACE_MAXADDR,
						NULL, NULL,
						adapter->sg_buf_size, // CHECK SIZE?
						1,
						adapter->sg_buf_size, // SIZE
						0,
						NULL,
						NULL,
						&rp[i]->my_tag);

		if (err) {
			device_printf(dev, "%s: bus_dma_tag_create(%s[%d]) failed.\n", __func__, dir, i);
			return (err);
		}

		err = bus_dmamem_alloc(rp[i]->my_tag, (void **) &rp[i]->buf_addr, BUS_DMA_WAITOK | BUS_DMA_COHERENT | BUS_DMA_ZERO, &rp[i]->my_map);
		if (err) {
			device_printf(dev, "%s: bus_dmamem_alloc(%s[%d]) rp[i]->buf_addr failed.\n", __func__, dir, i);
			return (err);
		}

		bzero(rp[i]->buf_addr, adapter->sg_buf_size);

		err = bus_dmamap_load(rp[i]->my_tag, rp[i]->my_map, rp[i]->buf_addr, adapter->sg_buf_size, callback_dma, &hw_addr, BUS_DMA_NOWAIT);
		if (err) {
			device_printf(dev, "%s: bus_dmamap_load(%s[%d]) hw_addr failed.\n", __func__, dir, i);
			return (err);
		}
		rp[i]->buf_hw_addr = hw_addr;

		rp[i]->bouncebuf_len = PAGE_SIZE;
		rp[i]->bouncebuf = malloc(rp[i]->bouncebuf_len, M_SUME, M_WAITOK);
		if (rp[i]->bouncebuf == NULL) {
			device_printf(dev, "%s: malloc(%s[%d]) bouncebuffer failed.\n", __func__, dir, i);
			return (error);
		}

		/* Initialize state. */
//#ifndef SUME_GLOBAL_LOCK
//		spin_lock_init(&rp[i]->lock);
//#endif
//		init_waitqueue_head(&rp[i]->waitq);
		mtx_init(&rp[i]->send_sleep, "Send sleep", NULL, MTX_SPIN);
		mtx_init(&rp[i]->recv_sleep, "Recv sleep", NULL, MTX_SPIN);
		mtx_init(&rp[i]->lock, "Channel lock", NULL, MTX_SPIN);

		rp[i]->rtag = -3;
		rp[i]->state = SUME_RIFFA_CHAN_STATE_IDLE;
	}

	return (0);
}

static int
sume_probe_riffa_buffers(struct sume_adapter *adapter) {
	int error;

	error = sume_probe_riffa_buffer(adapter, &adapter->recv, "recv");
	if (error)
		return (error);

	error = sume_probe_riffa_buffer(adapter, &adapter->send, "send");

	return (error);
}

static int
sume_attach(device_t dev)
{
	struct sume_adapter *adapter = device_get_softc(dev);
	adapter->dev = dev;
	int error, i;

	/* Start with the safety check to avoid malfunctions further down. */
	/* XXX-BZ they should bake this informaton into the bitfile. */
	if (sume_nports < 1 || sume_nports > SUME_PORTS_MAX) {
		device_printf(dev, "%s: sume_nports out of range: %d (1..%d). "
		    "Using max.\n", __func__, sume_nports, SUME_PORTS_MAX);
		sume_nports = SUME_PORTS_MAX;
	}

	atomic_set_int(&adapter->running, 0);

	/* OK finish up RIFFA. */
	error = sume_probe_riffa_pci(adapter);
	if (error != 0)
		goto error;

	error = sume_probe_riffa_buffers(adapter);
	if (error != 0)
		goto error;

	/* Now do the network interfaces. */
	for (i = 0; i < sume_nports; i++) {
		error = sume_netdev_alloc(adapter, i);
		if (error != 0)
			goto error;
	}

	//error = sume_register_netdevs(adapter);
	//if (error != 0)
		//goto error;

	/* Register debug sysctls. */
	//sume_init_sysctl();

	/* Reset the HW. */
	read_reg(adapter, RIFFA_INFO_REG_OFF);

	/* Ready to go, "enable" IRQ. */
	atomic_set_int(&adapter->running, 1);

	bus_generic_attach(dev); // where?

	return (0);

error:
	sume_detach(dev);

	return (error);
}

static void
sume_remove_riffa_buffer(const struct sume_adapter *adapter,
	struct riffa_chnl_dir **pp)
{
	int i;

	for (i = 0; i < adapter->num_chnls; i++) {
		if (pp[i] == NULL)
			continue;

		/* Wakeup anyone asleep before the waitq goes boom. */
		/* XXX-BZ not really good enough. */
		//wake_up_interruptible(&pp[i]->waitq);

		if (pp[i]->bouncebuf != NULL)
			free(pp[i]->bouncebuf, M_SUME);

		if (pp[i]->buf_hw_addr != 0) {
			bus_dmamem_free(pp[i]->my_tag, pp[i]->buf_addr, pp[i]->my_map);
			pp[i]->buf_hw_addr = 0;
		}

		free(pp[i], M_SUME);
	}
}

static void
sume_remove_riffa_buffers(struct sume_adapter *adapter)
{
	if (adapter->send != NULL) {
		sume_remove_riffa_buffer(adapter, adapter->send);
		free(adapter->send, M_SUME);
		adapter->send = NULL;
	}
	if (adapter->recv != NULL) {
		sume_remove_riffa_buffer(adapter, adapter->recv);
		free(adapter->recv, M_SUME);
		adapter->recv = NULL;
	}
}

static int
sume_detach(device_t dev)
{
	struct sume_adapter *adapter = device_get_softc(dev);
	int rc, i;

	sume_remove_riffa_buffers(adapter);

	for (i = 0; i < sume_nports; i++) {
		if (adapter->netdev[i] != NULL)
			ether_ifdetach(adapter->netdev[i]);
	}

	rc = bus_generic_detach(dev);
	if (rc)
		return (rc);

	if (adapter->irq.tag)
		bus_teardown_intr(dev, adapter->irq.res, adapter->irq.tag);
	if (adapter->irq.res)
		bus_release_resource(dev, SYS_RES_IRQ, adapter->irq.rid, adapter->irq.res);

	device_delete_children(dev);

	pci_release_msi(dev);

	if (adapter->bar0_addr)
		bus_release_resource(dev, SYS_RES_MEMORY, adapter->rid, adapter->bar0_addr);

	return (0);
}

static int
mod_event(module_t mod, int cmd, void *arg)
{
	int rc = 0;

	switch (cmd) {
	case MOD_LOAD:
		printf("MOD_LOAD\n");
		break;

	case MOD_UNLOAD:
		printf("MOD_UNLOAD\n");
		break;
	}

	return (rc);
}
static devclass_t sume_devclass;

DRIVER_MODULE(sume, pci, sume_driver, sume_devclass, mod_event, 0);
MODULE_VERSION(sume, 1);
#ifdef DEV_NETMAP
MODULE_DEPEND(sume, netmap, 1, 1, 1);
#endif /* DEV_NETMAP */
