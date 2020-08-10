/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2015 University of Cambridge
 * Copyright (c) 2015 Bjoern A. Zeeb
 * Copyright (c) 2020 Denis Salopek
 * All rights reserved
 *
 * This software was developed by Stanford University and the University of
 * Cambridge Computer Laboratory under National Science Foundation under Grant
 * No. CNS-0855268, the University of Cambridge Computer Laboratory under EPSRC
 * INTERNET Project EP/H040536/1 and by the University of Cambridge Computer
 * Laboratory under DARPA/AFRL contract FA8750-11-C-0249 ("MRC2"), as part of
 * the DARPA MRC research programme.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: $");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>

#include <netinet/in.h>
#include <netinet/if_ether.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <machine/bus.h>

#include "adapter.h"

#define	PCI_VENDOR_ID_XILINX	0x10ee
#define	PCI_DEVICE_ID_SUME	0x7028

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

/*
 * The DMA engine for SUME generates interrupts for each RX/TX transaction.
 * Depending on the channel (0 if packet transaction, 1 if register transaction)
 * the used bits of the interrupt vector will be the lowest or the second lowest
 * 5 bits.
 *
 * When receiving packets from SUME (RX):
 * (1) SUME received a packet on one of the interfaces.
 * (2) SUME generates an interrupt vector, bit 00001 is set (channel 0 - new RX
 *     transaction).
 * (3) We read the length of the incoming packet and the offset (is offset even
 *     used for anything except checking boundaries?) along with the 'last' flag
 *     from the SUME registers.
 * (4) We prepare for the DMA transaction by setting the bouncebuffer on the
 *     address buf_addr. For now, this is how it's done:
 *     - First 3*sizeof(uint32_t) bytes are: lower and upper 32 bits of physical
 *     address where we want the data to arrive (buf_addr[0] and buf_addr[1]),
 *     and length of incoming data (buf_addr[2]).
 *     - Data will start right after, at buf_addr+3*sizeof(uint32_t). The
 *     physical address buf_hw_addr is a block of contiguous memory mapped to
 *     buf_addr, so we can set the incoming data's physical address (buf_addr[0]
 *     and buf_addr[1]) to buf_hw_addr+3*sizeof(uint32_t).
 * (5) We notify SUME that the bouncebuffer is ready for the transaction by
 *     writing the lower/upper physical address buf_hw_addr to the SUME
 *     registers RIFFA_TX_SG_ADDR_LO_REG_OFF and RIFFA_TX_SG_ADDR_HI_REG_OFF as
 *     well as the number of segments to the register RIFFA_TX_SG_LEN_REG_OFF.
 * (6) SUME generates an interrupt vector, bit 00010 is set (channel 0 -
 *     bouncebuffer received).
 * (7) SUME generates an interrupt vector, bit 00100 is set (channel 0 -
 *     transaction is done).
 * (8) We read the first 16 bytes (metadata) of the received data and note the
 *     incoming interface so we can later forward it to the right one in the OS
 *     (nf0, nf1, nf2 or nf3).
 * (9) We create an mbuf and copy the data from the bouncebuffer to the mbuf and
 *     set the mbuf rcvif to the incoming interface.
 * (10) We forward the mbuf to the appropriate interface via ifp->if_input.
 *
 * When sending packets to SUME (TX):
 * (1) The OS calls sume_start_xmit() function on TX.
 * (2) We get the mbuf packet data and copy it to the
 *     buf_addr+3*sizeof(uint32_t) + metadata 16 bytes.
 * (3) We create the metadata based on the output interface and copy it to the
 *     buf_addr+3*sizeof(uint32_t).
 * (4) We write the offset/last and length of the packet to the SUME registers
 *     RIFFA_RX_OFFLAST_REG_OFF and RIFFA_RX_LEN_REG_OFF.
 * (5) We fill the bouncebuffer by filling the first 3*sizeof(uint32_t) bytes
 *     with the physical address and length just as in RX step (4).
 * (6) We notify SUME that the bouncebuffer is ready by writing to SUME
 *     registers RIFFA_RX_SG_ADDR_LO_REG_OFF, RIFFA_RX_SG_ADDR_HI_REG_OFF and
 *     RIFFA_RX_SG_LEN_REG_OFF just as in RX step (5).
 * (7) SUME generates an interrupt vector, bit 01000 is set (channel 0 -
 *     bouncebuffer is read).
 * (8) SUME generates an interrupt vector, bit 10000 is set (channel 0 -
 *     transaction is done).
 *
 * Internal registers
 * Every module in the SUME hardware has its own set of internal registers
 * (IDs, for debugging and statistic purposes, etc.). Their base addresses are
 * defined in 'projects/reference_nic/hw/tcl/reference_nic_defines.tcl' and the
 * offsets to different memory locations of every module are defined in their
 * corresponding folder inside the library. These registers can be RO/RW and
 * there is a special method to fetch/change this data over 1 or 2 DMA
 * transactions. For writing, by calling the sume_module_reg_write(). For
 * reading, by calling the sume_module_reg_write() and then
 * sume_module_reg_read(). Check those functions for more information.
 */

MALLOC_DECLARE(M_SUME);
MALLOC_DEFINE(M_SUME, "sume", "NetFPGA SUME device driver");

static void check_queues(struct sume_adapter *);
static void sume_fill_bb_desc(struct sume_adapter *, struct riffa_chnl_dir *,
    uint64_t);

static struct unrhdr *unr;

static struct {
	uint16_t device;
	char *desc;
} sume_pciids[] = {
	{PCI_DEVICE_ID_SUME, "NetFPGA SUME"},
};

static inline uint32_t
read_reg(struct sume_adapter *adapter, int offset)
{

	return (bus_space_read_4(adapter->bt, adapter->bh, offset << 2));
}

static inline void
write_reg(struct sume_adapter *adapter, int offset, uint32_t val)
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

/*
 * Building mbuf for packet received from SUME. We expect to receive 'len'
 * bytes of data (including metadata) written from the bouncebuffer address
 * buf_addr+3*sizeof(uint32_t). Metadata will tell us which SUME interface
 * received the packet (sport will be 1, 2, 4 or 8), the packet length (plen),
 * and the magic word needs to be 0xcafe. When we have the packet data, we
 * create an mbuf and copy the data to it using m_copyback() function, set the
 * correct interface to rcvif and send the packet to the OS with if_input.
 */
static struct mbuf *
sume_rx_build_mbuf(struct sume_adapter *adapter, uint32_t len)
{
	struct nf_priv *nf_priv;
	struct mbuf *m;
	struct ifnet *ifp = NULL;
	int np;
	uint16_t sport, dport, plen, magic;
	device_t dev = adapter->dev;
	uint8_t *indata = (uint8_t *)
	    adapter->recv[SUME_RIFFA_CHANNEL_DATA]->buf_addr +
	    sizeof(struct nf_bb_desc);
	struct nf_metadata *mdata = (struct nf_metadata *) indata;

	/* The metadata header is 16 bytes. */
	if (len < sizeof(struct nf_metadata)) {
		device_printf(dev, "%s: short frame (%d)\n",
		    __func__, len);
		adapter->packets_err++;
		adapter->bytes_err += len;
		return (NULL);
	}

	sport = le16toh(mdata->sport);
	dport = le16toh(mdata->dport);
	plen = le16toh(mdata->plen);
	magic = le16toh(mdata->magic);

	if ((sizeof(struct nf_metadata) + plen) > len ||
	    magic != SUME_RIFFA_MAGIC) {
		device_printf(dev, "%s: corrupted packet (%zd + %d > %d "
		    "|| magic 0x%04x != 0x%04x)\n", __func__, sizeof(struct
		    nf_metadata), plen, len, magic, SUME_RIFFA_MAGIC);
		return (NULL);
	}

	/* We got the packet from one of the even bits */
	np = (ffs(dport & SUME_DPORT_MASK) >> 1) - 1;
	if (np > SUME_NPORTS) {
		device_printf(dev, "%s: invalid destination port 0x%04x"
		    "(%d)\n", __func__, dport, np);
		adapter->packets_err++;
		adapter->bytes_err += plen;
		return (NULL);
	}
	ifp = adapter->ifp[np];

	/* If the interface is down, well, we are done. */
	nf_priv = ifp->if_softc;
	if (!(ifp->if_flags & IFF_UP)) {
		if (adapter->sume_debug)
			device_printf(dev, "Device nf%d not up.\n", np);
		adapter->packets_err++;
		adapter->bytes_err += plen;
		return (NULL);
	}

	if (adapter->sume_debug)
		printf("Building mbuf with length: %d\n", plen);

	m = m_getm(NULL, plen, M_NOWAIT, MT_DATA);
	if (m == NULL) {
		adapter->packets_err++;
		adapter->bytes_err += plen;
		return (NULL);
	}

	/* Copy the data in at the right offset. */
	m_copyback(m, 0, plen, (void *) (indata + sizeof(struct nf_metadata)));
	m->m_pkthdr.rcvif = ifp;

	nf_priv->stats.rx_packets++;
	nf_priv->stats.rx_bytes += plen;
	return (m);
}

/*
 * SUME interrupt handler for when we get a valid interrupt from the board.
 * There are 2 interrupt vectors, we use vect0 when the number of channels is
 * lower then 7 and vect1 otherwise. Theoretically, we can receive interrupt
 * for any of the available channels, but RIFFA DMA uses only 2: 0 and 1, so we
 * use only vect0. The vector is a 32 bit number, using 5 bits for every
 * channel, the least significant bits correspond to channel 0 and the next 5
 * bits correspond to channel 1. Vector bits for RX/TX are:
 * RX
 * bit 0 - new transaction from SUME
 * bit 1 - SUME received our bouncebuffer address
 * bit 2 - SUME copied the received data to our bouncebuffer, transaction done
 * TX
 * bit 3 - SUME received our bouncebuffer address
 * bit 4 - SUME copied the data from our bouncebuffer, transaction done
 *
 * There are two finite state machines (one for TX, one for RX). We loop
 * through channels 0 and 1 to check and our current state and which interrupt
 * bit is set.
 * TX
 * SUME_RIFFA_CHAN_STATE_IDLE: waiting for the first TX transaction.
 * SUME_RIFFA_CHAN_STATE_READY: we prepared (filled with data) the bouncebuffer
 * and triggered the SUME for the TX transaction. Waiting for interrupt bit 3
 * to go to the next state.
 * SUME_RIFFA_CHAN_STATE_READ: waiting for interrupt bit 4 (for SUME to send
 * our packet). Then we get the length of the sent data and go back to the
 * IDLE state.
 * RX
 * SUME_RIFFA_CHAN_STATE_IDLE: waiting for the interrupt bit 0 (new RX
 * transaction). When we get it, we prepare our bouncebuffer for reading and
 * trigger the SUME to start the transaction. Go to the next state.
 * SUME_RIFFA_CHAN_STATE_READY: waiting for the interrupt bit 1 (SUME got our
 * bouncebuffer). Go to the next state.
 * SUME_RIFFA_CHAN_STATE_READ: SUME copied data and our bouncebuffer is ready,
 * we can build the mbuf and go back to the IDLE state.
 */
static void
sume_intr_handler(void *arg)
{
	struct sume_adapter *adapter = arg;
	uint32_t vect, vect0, len;
	int i, loops;
	device_t dev = adapter->dev;
	struct mbuf *m = NULL;
	struct ifnet *ifp = NULL;
	struct riffa_chnl_dir *send, *recv;

	SUME_LOCK(adapter);

	vect0 = read_reg(adapter, RIFFA_IRQ_REG0_OFF);
	if ((vect0 & SUME_INVALID_VECT) != 0) {
		SUME_UNLOCK(adapter);
		return;
	}

	/*
	 * We only have one interrupt for all channels and no way
	 * to quickly lookup for which channel(s) we got an interrupt?
	 */
	for (i = 0; i < SUME_RIFFA_CHANNELS; i++) {
		vect = vect0 >> (5 * i);
		send = adapter->send[i];
		recv = adapter->recv[i];

		loops = 0;
		while ((vect & (SUME_MSI_TXBUF | SUME_MSI_TXDONE)) &&
		    loops <= 5) {
			if (adapter->sume_debug)
				device_printf(dev, "%s: TX ch %d state %u "
				    "vect = 0x%08x\n", __func__, i,
				    send->state, vect);
			switch (send->state) {
			case SUME_RIFFA_CHAN_STATE_IDLE:
				break;
			case SUME_RIFFA_CHAN_STATE_READY:
				if (!(vect & SUME_MSI_TXBUF)) {
					device_printf(dev, "%s: ch %d "
					    "unexpected interrupt in send+3 "
					    "state %u: vect = 0x%08x\n",
					    __func__, i, send->state, vect);
					send->recovery = 1;
					break;
				}
				send->state = SUME_RIFFA_CHAN_STATE_READ;
				vect &= ~SUME_MSI_TXBUF;
				break;
			case SUME_RIFFA_CHAN_STATE_READ:
				if (!(vect & SUME_MSI_TXDONE)) {
					device_printf(dev, "%s: ch %d "
					    "unexpected interrupt in send+4 "
					    "state %u: vect = 0x%08x\n",
					    __func__, i, send->state, vect);
					send->recovery = 1;
					send->state =
					    SUME_RIFFA_CHAN_STATE_LEN;
					break;
				}

				len = read_reg(adapter, RIFFA_CHNL_REG(i,
				    RIFFA_RX_TNFR_LEN_REG_OFF));
				if (i == SUME_RIFFA_CHANNEL_DATA) {
					send->state =
					    SUME_RIFFA_CHAN_STATE_IDLE;
					check_queues(adapter);
				} else if (i == SUME_RIFFA_CHANNEL_REG)
					wakeup(&send->event);
				else {
					device_printf(dev, "%s: interrupt on "
					    "ch %d unexpected in send+4 state "
					    "%u: vect = 0x%08x\n", __func__, i,
					    send->state, vect);
					send->recovery = 1;
				}
				vect &= ~SUME_MSI_TXDONE;
				break;
			case SUME_RIFFA_CHAN_STATE_LEN:
				break;
			default:
				device_printf(dev, "Unknown TX state!\n");
			}
			loops++;
		}

		if ((vect & (SUME_MSI_TXBUF | SUME_MSI_TXDONE)) &&
		    send->recovery)
			device_printf(dev, "%s: ignoring vect = 0x%08x during "
			    "TX; not in recovery; state = %d loops = %d\n",
			    __func__, vect, send->state, loops);

		loops = 0;
		while ((vect & (SUME_MSI_RXQUE | SUME_MSI_RXBUF |
			SUME_MSI_RXDONE)) && loops < 5) {
			if (adapter->sume_debug)
				device_printf(dev, "%s: RX ch %d state %u "
				    "vect = 0x%08x\n", __func__, i,
				    recv->state, vect);
			switch (recv->state) {
			case SUME_RIFFA_CHAN_STATE_IDLE:
				if (!(vect & SUME_MSI_RXQUE)) {
					device_printf(dev, "%s: ch %d "
					    "unexpected interrupt in recv+0 "
					    "state %u: vect = 0x%08x\n",
					    __func__, i, recv->state, vect);
					recv->recovery = 1;
					break;
				}
				uint32_t max_ptr;

				/* Clear recovery state. */
				recv->recovery = 0;

				/* Get offset and length. */
				recv->offlast = read_reg(adapter,
				    RIFFA_CHNL_REG(i,
				    RIFFA_TX_OFFLAST_REG_OFF));
				recv->len = read_reg(adapter, RIFFA_CHNL_REG(i,
				    RIFFA_TX_LEN_REG_OFF));

				/* Boundary checks. */
				max_ptr = (uint32_t) ((char *) recv->buf_addr +
				    SUME_RIFFA_OFFSET(recv->offlast) +
				    SUME_RIFFA_LEN(recv->len) - 1);
				if (max_ptr < (uint32_t) recv->buf_addr) {
					device_printf(dev, "%s: receive buffer"
					    " wrap-around overflow.\n",
					    __func__);
				}
				if ((SUME_RIFFA_OFFSET(recv->offlast) +
				    SUME_RIFFA_LEN(recv->len)) >
				    adapter->sg_buf_size) {
					device_printf(dev, "%s: receive buffer"
					    " too small.\n", __func__);
				}

				/* Fill the bouncebuf "descriptor". */
				sume_fill_bb_desc(adapter, recv,
				    SUME_RIFFA_LEN(recv->len));

				bus_dmamap_sync(recv->my_tag, recv->my_map,
				    BUS_DMASYNC_PREREAD |
				    BUS_DMASYNC_PREWRITE);
				write_reg(adapter, RIFFA_CHNL_REG(i,
				    RIFFA_TX_SG_ADDR_LO_REG_OFF),
				    SUME_RIFFA_LO_ADDR(recv->buf_hw_addr));
				write_reg(adapter, RIFFA_CHNL_REG(i,
				    RIFFA_TX_SG_ADDR_HI_REG_OFF),
				    SUME_RIFFA_HI_ADDR(recv->buf_hw_addr));
				write_reg(adapter, RIFFA_CHNL_REG(i,
				    RIFFA_TX_SG_LEN_REG_OFF),
				    4 *recv->num_sg);
				bus_dmamap_sync(recv->my_tag, recv->my_map,
				    BUS_DMASYNC_POSTREAD |
				    BUS_DMASYNC_POSTWRITE);

				recv->state = SUME_RIFFA_CHAN_STATE_READY;
				vect &= ~SUME_MSI_RXQUE;
				break;
			case SUME_RIFFA_CHAN_STATE_READY:
				if (!(vect & SUME_MSI_RXBUF)) {
					device_printf(dev, "%s: ch %d "
					    "unexpected interrupt in recv+1 "
					    "state %u: vect = 0x%08x\n",
					    __func__, i, recv->state, vect);
					recv->recovery = 1;
					break;
				}
				recv->state = SUME_RIFFA_CHAN_STATE_READ;
				vect &= ~SUME_MSI_RXBUF;
				break;
			case SUME_RIFFA_CHAN_STATE_READ:
				if (!(vect & SUME_MSI_RXDONE)) {
					device_printf(dev, "%s: ch %d "
					    "unexpected interrupt in recv+2 "
					    "state %u: vect = 0x%08x\n",
					    __func__, i, recv->state, vect);
					recv->recovery = 1;
					break;
				}
				len = read_reg(adapter, RIFFA_CHNL_REG(i,
				    RIFFA_TX_TNFR_LEN_REG_OFF));

				/*
				 * Remember, len and recv[i]->len
				 * are words.
				 */
				if (i == SUME_RIFFA_CHANNEL_DATA) {
					m = sume_rx_build_mbuf(adapter, 
					    len << 2);
					recv->state =
					    SUME_RIFFA_CHAN_STATE_IDLE;
				} else if (i == SUME_RIFFA_CHANNEL_REG)
					wakeup(&recv->event);
				else {
					device_printf(dev, "%s: interrupt on "
					    "ch %d unexpected in recv+2 state "
					    "%u: vect = 0x%08x\n", __func__, i,
					    recv->state, vect);
					recv->recovery = 1;
				}
				vect &= ~SUME_MSI_RXDONE;
				break;
			case SUME_RIFFA_CHAN_STATE_LEN:
				break;
			default:
				device_printf(dev, "Unknown RX state!\n");
			}
			loops++;
		}

		if ((vect & (SUME_MSI_RXQUE | SUME_MSI_RXBUF |
		    SUME_MSI_RXDONE)) && recv->recovery)
			device_printf(dev, "%s: ignoring vect = 0x%08x during "
			    "RX; not in recovery; state = %d, loops = %d\n",
			    __func__, vect, recv->state, loops);
	}
	SUME_UNLOCK(adapter);

	if (m != NULL) {
		ifp = m->m_pkthdr.rcvif;
		(*ifp->if_input)(ifp, m);
	}
}

/*
 * Filtering interrupts. We wait for the adapter to go into the 'running' state
 * to start accepting them. Also, we ignore them if the first two bits of the
 * vector are set.
 */
static int
sume_intr_filter(void *arg)
{
	struct sume_adapter *adapter = arg;

	/*
	 * Ignore early interrupts from RIFFA given we cannot disable interrupt
	 * generation.
	 */
	if (adapter->running == 0)
		return (FILTER_STRAY);

	return (FILTER_SCHEDULE_THREAD);
}

static int
sume_probe_riffa_pci(struct sume_adapter *adapter)
{
	device_t dev = adapter->dev;
	int error, count, capmem;
	uint32_t reg, devctl, linkctl;

	pci_enable_busmaster(dev);

	adapter->rid = PCIR_BAR(0);
	adapter->bar0_addr = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &adapter->rid, RF_ACTIVE);
	if (adapter->bar0_addr == NULL) {
		device_printf(dev, "Unable to allocate bus resource: "
		    "bar0_addr\n");
		return (ENXIO);
	}
	adapter->bt = rman_get_bustag(adapter->bar0_addr);
	adapter->bh = rman_get_bushandle(adapter->bar0_addr);
	adapter->bar0_len = rman_get_size(adapter->bar0_addr);
	if (adapter->bar0_len != 1024) {
		device_printf(dev, "%s: bar0_len %lu != 1024\n", __func__,
		    adapter->bar0_len);
		return (ENXIO);
	}

	count = pci_msi_count(dev);
	error = pci_alloc_msi(dev, &count);
	if (error) {
		device_printf(dev, "Unable to allocate bus resource: PCI "
		    "MSI\n");
		return (error);
	}

	adapter->irq.rid = 1; /* Should be 1, thus says pci_alloc_msi() */
	adapter->irq.res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &adapter->irq.rid, RF_SHAREABLE | RF_ACTIVE);
	if (adapter->irq.res == NULL) {
		device_printf(dev, "Unable to allocate bus resource: IRQ "
		    "memory\n");
		return (ENXIO);
	}

	error = bus_setup_intr(dev, adapter->irq.res, INTR_MPSAFE |
	    INTR_TYPE_NET, sume_intr_filter, sume_intr_handler, adapter,
	    &adapter->irq.tag);
	if (error) {
		device_printf(dev, "failed to setup interrupt for rid %d, name"
		    " %s: %d\n", adapter->irq.rid, "SUME_INTR", error);
		return (ENXIO);
	} else
		bus_describe_intr(dev, adapter->irq.res, adapter->irq.tag,
		    "%s", "SUME_INTR");

	if (pci_find_cap(dev, PCIY_EXPRESS, &capmem) != 0) {
		device_printf(dev, "%s: pcie_capability_read_dword "
		    "PCI_EXP_DEVCTL error\n", __func__);
		return (ENXIO);
	}

	devctl = pci_read_config(dev, capmem + PCIER_DEVICE_CTL, 2);
	pci_write_config(dev, capmem + PCIER_DEVICE_CTL, (devctl |
	    PCIEM_CTL_EXT_TAG_FIELD), 2);

	devctl = pci_read_config(dev, capmem + PCIER_DEVICE_CTL2, 2);
	pci_write_config(dev, capmem + PCIER_DEVICE_CTL2, (devctl |
	    PCIEM_CTL2_ID_ORDERED_REQ_EN), 2);

	linkctl = pci_read_config(dev, capmem + PCIER_LINK_CTL, 2);
	pci_write_config(dev, capmem + PCIER_LINK_CTL, (linkctl |
	    PCIEM_LINK_CTL_RCB), 2);

	reg = read_reg(adapter, RIFFA_INFO_REG_OFF);
	adapter->num_sg =	RIFFA_SG_ELEMS * ((reg >> 19) & 0xf);
	adapter->sg_buf_size =	RIFFA_SG_BUF_SIZE * ((reg >> 19) & 0xf);

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

/* If there is no sume_if_init, the ether_ioctl panics. */
static void
sume_if_init(void *sc)
{
}

/* Helper functions. */
static void
sume_fill_bb_desc(struct sume_adapter *adapter, struct riffa_chnl_dir *p,
    uint64_t len)
{
	struct nf_bb_desc *bouncebuf = (struct nf_bb_desc *) p->buf_addr;

	bouncebuf->lower = (p->buf_hw_addr + sizeof(struct nf_bb_desc));
	bouncebuf->upper = (p->buf_hw_addr + sizeof(struct nf_bb_desc)) >> 32;
	bouncebuf->len = len >> 2;

	p->num_sg = 1;
}

/* Register read/write. */
static int
sume_modreg_write_locked(struct sume_adapter *adapter)
{
	struct riffa_chnl_dir *send = adapter->send[SUME_RIFFA_CHANNEL_REG];

	/* Let the FPGA know about the transfer. */
	write_reg(adapter, RIFFA_CHNL_REG(SUME_RIFFA_CHANNEL_REG,
	    RIFFA_RX_OFFLAST_REG_OFF), SUME_OFFLAST);
	write_reg(adapter, RIFFA_CHNL_REG(SUME_RIFFA_CHANNEL_REG,
	    RIFFA_RX_LEN_REG_OFF), send->len);	/* words */

	/* Fill the bouncebuf "descriptor". */
	sume_fill_bb_desc(adapter, send, SUME_RIFFA_LEN(send->len));

	/* Update the state before intiating the DMA to avoid races. */
	send->state = SUME_RIFFA_CHAN_STATE_READY;

	bus_dmamap_sync(send->my_tag, send->my_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	/* DMA. */
	write_reg(adapter, RIFFA_CHNL_REG(SUME_RIFFA_CHANNEL_REG,
	    RIFFA_RX_SG_ADDR_LO_REG_OFF),
	    SUME_RIFFA_LO_ADDR(send->buf_hw_addr));
	write_reg(adapter, RIFFA_CHNL_REG(SUME_RIFFA_CHANNEL_REG,
	    RIFFA_RX_SG_ADDR_HI_REG_OFF),
	    SUME_RIFFA_HI_ADDR(send->buf_hw_addr));
	write_reg(adapter, RIFFA_CHNL_REG(SUME_RIFFA_CHANNEL_REG,
	    RIFFA_RX_SG_LEN_REG_OFF), 4 * send->num_sg);
	bus_dmamap_sync(send->my_tag, send->my_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	return (0);
}

/*
 * Request a register read or write (depending on strb).
 * If strb is set (0x1f) this will result in a register write,
 * otherwise this will result in a register read request at the given
 * address and the result will need to be DMAed back.
 */
static int
sume_module_reg_write(struct nf_priv *nf_priv, struct sume_ifreq *sifr,
    uint32_t strb)
{
	struct sume_adapter *adapter;
	struct nf_regop_data *data;
	int error;
	struct riffa_chnl_dir *send;

	KASSERT(nf_priv->riffa_channel == SUME_RIFFA_CHANNEL_REG,
	    ("write on non-reg channel"));

	adapter = nf_priv->adapter;
	send = adapter->send[SUME_RIFFA_CHANNEL_REG];

	/*
	 * 1. Make sure the channel is free;  otherwise return EBUSY.
	 * 2. Prepare the memory in the bounce buffer (which we always
	 *    use for regs).
	 * 3. Start the DMA process.
	 * 4. Sleep and wait for result and return success or error.
	 */
	SUME_LOCK(adapter);

	if (send->state != SUME_RIFFA_CHAN_STATE_IDLE) {
		SUME_UNLOCK(adapter);
		return (EBUSY);
	}

	data = (struct nf_regop_data *) (send->buf_addr +
	    sizeof(struct nf_bb_desc));
	data->addr = htole32(sifr->addr);
	data->val = htole32(sifr->val);
	/* Tag to indentify request. */
	data->rtag = htole32(++send->rtag);
	data->strb = htole32(strb);
	send->len = 4;	/* words */

	error = sume_modreg_write_locked(adapter);
	if (error) {
		SUME_UNLOCK(adapter);
		return (EFAULT);
	}

	/* Timeout after 1s. */
	if (send->state != SUME_RIFFA_CHAN_STATE_LEN)
		error = msleep(&send->event, &adapter->lock, 0,
		    "Waiting recv finish", 1 * hz);

	/* This was a write so we are done; were interrupted, or timed out. */
	if (strb != 0x00 || error != 0 || error == EWOULDBLOCK) {
		send->state = SUME_RIFFA_CHAN_STATE_IDLE;
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

	SUME_UNLOCK(adapter);

	return (error);
}

static int
sume_module_reg_read(struct nf_priv *nf_priv, struct sume_ifreq *sifr)
{
	struct sume_adapter *adapter;
	struct nf_regop_data *data;
	int error = 0;
	struct riffa_chnl_dir *recv, *send;

	KASSERT(nf_priv->riffa_channel == SUME_RIFFA_CHANNEL_REG,
	    ("write on non-reg channel"));

	adapter = nf_priv->adapter;
	recv = adapter->recv[SUME_RIFFA_CHANNEL_REG];
	send = adapter->send[SUME_RIFFA_CHANNEL_REG];

	/*
	 * 0. Sleep waiting for result if needed (unless condition is
	 *    true already).
	 * 2. Read DMA results.
	 * 3. Update state on *TX* to IDLE to allow next read to start.
	 */
	SUME_LOCK(adapter);

	bus_dmamap_sync(recv->my_tag, recv->my_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	/*
	 * We only need to be woken up at the end of the transaction.
	 * Timeout after 1s.
	 */
	if (recv->state != SUME_RIFFA_CHAN_STATE_READ)
		error = msleep(&recv->event, &adapter->lock, 0,
		    "Waiting transaction finish", 1 * hz);

	if (recv->state != SUME_RIFFA_CHAN_STATE_READ || error == EWOULDBLOCK) {
		SUME_UNLOCK(adapter);
		device_printf(adapter->dev, "%s: wait error: %d\n", __func__,
		    error);
		return (EWOULDBLOCK);
	}

	bus_dmamap_sync(recv->my_tag, recv->my_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	/*
	 * Read reply data and validate address and tag.
	 * Note: we do access the send side without lock but the state
	 * machine does prevent the data from changing.
	 */
	data = (struct nf_regop_data *) (recv->buf_addr +
		sizeof(struct nf_bb_desc));

	if (le32toh(data->rtag) != send->rtag)
		device_printf(adapter->dev, "%s: rtag error: 0x%08x 0x%08x\n",
		    __func__, le32toh(data->rtag), send->rtag);

	sifr->val = le32toh(data->val);
	recv->state = SUME_RIFFA_CHAN_STATE_IDLE;

	/* We are done. */
	send->state = SUME_RIFFA_CHAN_STATE_IDLE;

	SUME_UNLOCK(adapter);

	return (0);
}

static int
sume_if_ioctl(struct ifnet *ifp, unsigned long cmd, caddr_t data)
{
	struct nf_priv *nf_priv;
	struct ifreq *ifr = (struct ifreq *) data;
	struct sume_ifreq sifr;
	int error = 0;
	struct sume_adapter *adapter;

	if (ifp == NULL)
		return (ENODEV);

	nf_priv = ifp->if_softc;
	if (nf_priv == NULL || nf_priv->adapter == NULL)
		return (EINVAL);

	adapter = nf_priv->adapter;
	if (adapter->running == 0)
		return (ENODEV);

	switch (cmd) {
	case SIOCGIFMEDIA:
	case SIOCGIFXMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &nf_priv->media, cmd);
		break;

	case SUME_IOCTL_CMD_WRITE_REG:
		error = copyin(ifr_data_get_ptr(ifr), &sifr, sizeof(sifr));
		if (error) {
			error = EINVAL;
			break;
		}
		error = sume_module_reg_write(nf_priv, &sifr, 0x1f);
		break;

	case SUME_IOCTL_CMD_READ_REG:
		error = copyin(ifr_data_get_ptr(ifr), &sifr, sizeof(sifr));
		if (error) {
			error = EINVAL;
			break;
		}

		error = sume_module_reg_write(nf_priv, &sifr, 0x00);
		if (error)
			break;

		error = sume_module_reg_read(nf_priv, &sifr);
		if (error)
			break;

		error = copyout(&sifr, ifr_data_get_ptr(ifr), sizeof(sifr));
		if (error)
			error = EINVAL;

		break;

	case SIOCSIFFLAGS:
		/* Silence tcpdump 'promisc mode not supported' warning. */
		if (ifp->if_flags & IFF_PROMISC)
			break;

	default:
		error = ether_ioctl(ifp, cmd, data);
		break;
	}

	return (error);
}

static int
sume_media_change(struct ifnet *ifp)
{
	struct nf_priv *nf_priv = ifp->if_softc;
	struct ifmedia *ifm = &nf_priv->media;

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
		return (EINVAL);

	if (IFM_SUBTYPE(ifm->ifm_media) == IFM_10G_SR)
		ifp->if_baudrate = ifmedia_baudrate(IFM_ETHER | IFM_10G_SR);
	else
		ifp->if_baudrate = ifmedia_baudrate(ifm->ifm_media);

	return (0);
}

static void
sume_media_status(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct nf_priv *nf_priv = ifp->if_softc;
	struct ifmedia *ifm = &nf_priv->media;
	struct sume_ifreq sifr;
	int error;
	int link_status = 0;

	if (ifm->ifm_cur->ifm_media == (IFM_ETHER | IFM_10G_SR) &&
	    (ifp->if_flags & IFF_UP))
		ifmr->ifm_active = IFM_ETHER | IFM_10G_SR;
	else
		ifmr->ifm_active = ifm->ifm_cur->ifm_media;

	ifmr->ifm_status |= IFM_AVALID;

	sifr.addr = SUME_STATUS_ADDR(nf_priv->port);
	sifr.val = 0;

	error = sume_module_reg_write(nf_priv, &sifr, 0x00);
	if (error)
		return;

	error = sume_module_reg_read(nf_priv, &sifr);
	if (error)
		return;

	link_status = SUME_LINK_STATUS(sifr.val);
	if (link_status)
		ifmr->ifm_status |= IFM_ACTIVE;

	return;
}

/*
 * Packet to transmit. We take the packet data from the mbuf and copy it to the
 * bouncebuffer address buf_addr+3*sizeof(uint32_t)+16. The 16 bytes before the
 * packet data are for metadata: sport/dport (depending on our source
 * interface), packet length and magic 0xcafe. We tell the SUME about the
 * transfer, fill the first 3*sizeof(uint32_t) bytes of the bouncebuffer with
 * the information about the start and length of the packet and trigger the
 * transaction.
 */
static int
sume_if_start_locked(struct ifnet *ifp)
{
	struct mbuf *m;
	struct nf_priv *nf_priv = ifp->if_softc;
	struct sume_adapter *adapter = nf_priv->adapter;
	struct riffa_chnl_dir *send = adapter->send[SUME_RIFFA_CHANNEL_DATA];
	uint8_t *outbuf;
	struct nf_metadata *mdata;
	int plen = SUME_MIN_PKT_SIZE;

	KASSERT(mtx_owned(&adapter->lock), ("SUME lock not owned"));
	KASSERT(nf_priv->riffa_channel == SUME_RIFFA_CHANNEL_DATA,
	    ("TX on non-data channel"));
	KASSERT(send->state == SUME_RIFFA_CHAN_STATE_IDLE,
	    ("SUME not in IDLE state"));

	IFQ_DEQUEUE(&ifp->if_snd, m);
	if (m == NULL)
		return (EINVAL);

	/* Packets large enough do not need to be padded */
	if (m->m_pkthdr.len > SUME_MIN_PKT_SIZE)
		plen = m->m_pkthdr.len;

	if (adapter->sume_debug)
		printf("Sending %d bytes to nf%d\n", plen, nf_priv->unit);

	outbuf = (uint8_t *) send->buf_addr + sizeof(struct nf_bb_desc);
	mdata = (struct nf_metadata *) outbuf;

	/* Clear the recovery flag. */
	send->recovery = 0;

	/* Make sure we fit with the 16 bytes nf_metadata. */
	if ((m->m_pkthdr.len + sizeof(struct nf_metadata)) >
	    adapter->sg_buf_size) {
		device_printf(adapter->dev, "%s: Packet too big for bounce "
		    "buffer (%d)\n", __func__, m->m_pkthdr.len);
		m_freem(m);
		nf_priv->stats.tx_dropped++;
		return (ENOMEM);
	}

	bus_dmamap_sync(send->my_tag, send->my_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	/* Zero out the padded data */
	if (m->m_pkthdr.len < SUME_MIN_PKT_SIZE)
		bzero(outbuf + sizeof(struct nf_metadata), SUME_MIN_PKT_SIZE);
	/* Skip the first 16 bytes for the metadata. */
	m_copydata(m, 0, m->m_pkthdr.len, outbuf + sizeof(struct nf_metadata));
	send->len = (sizeof(struct nf_metadata) + plen + 3) / 4;

	/* Fill in the metadata: CPU(DMA) ports are odd, MAC ports are even. */
	mdata->sport = htole16(1 << (nf_priv->port * 2 + 1));
	mdata->dport = htole16(1 << (nf_priv->port * 2));
	mdata->plen = htole16(plen);
	mdata->magic = htole16(SUME_RIFFA_MAGIC);
	mdata->t1 = htole32(0);
	mdata->t2 = htole32(0);

	/* Let the FPGA know about the transfer. */
	write_reg(adapter, RIFFA_CHNL_REG(SUME_RIFFA_CHANNEL_DATA,
	    RIFFA_RX_OFFLAST_REG_OFF), SUME_OFFLAST);
	write_reg(adapter, RIFFA_CHNL_REG(SUME_RIFFA_CHANNEL_DATA,
	    RIFFA_RX_LEN_REG_OFF), send->len);

	/* Fill the bouncebuf "descriptor". */
	sume_fill_bb_desc(adapter, send, SUME_RIFFA_LEN(send->len));

	/* Update the state before intiating the DMA to avoid races. */
	send->state = SUME_RIFFA_CHAN_STATE_READY;

	bus_dmamap_sync(send->my_tag, send->my_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	/* DMA. */
	write_reg(adapter, RIFFA_CHNL_REG(SUME_RIFFA_CHANNEL_DATA,
	    RIFFA_RX_SG_ADDR_LO_REG_OFF),
	    SUME_RIFFA_LO_ADDR(send->buf_hw_addr));
	write_reg(adapter, RIFFA_CHNL_REG(SUME_RIFFA_CHANNEL_DATA,
	    RIFFA_RX_SG_ADDR_HI_REG_OFF),
	    SUME_RIFFA_HI_ADDR(send->buf_hw_addr));
	write_reg(adapter, RIFFA_CHNL_REG(SUME_RIFFA_CHANNEL_DATA,
	    RIFFA_RX_SG_LEN_REG_OFF), 4 * send->num_sg);

	bus_dmamap_sync(send->my_tag, send->my_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	nf_priv->stats.tx_packets++;
	nf_priv->stats.tx_bytes += plen;

	/* We can free as long as we use the bounce buffer. */
	m_freem(m);

	adapter->last_ifc = nf_priv->port;

	return (0);
}

static void
sume_if_start(struct ifnet *ifp)
{
	struct nf_priv *nf_priv = ifp->if_softc;
	struct sume_adapter *adapter = nf_priv->adapter;

	KASSERT(nf_priv->riffa_channel == SUME_RIFFA_CHANNEL_DATA,
	    ("TX on non-data channel"));

	if (!adapter->running)
		return;
	if (!(ifp->if_flags & IFF_UP))
		return;

	SUME_LOCK(adapter);
	if (adapter->send[SUME_RIFFA_CHANNEL_DATA]->state !=
	    SUME_RIFFA_CHAN_STATE_IDLE) {
		SUME_UNLOCK(adapter);
		return;
	}

	sume_if_start_locked(ifp);
	SUME_UNLOCK(adapter);
}

static void
check_queues(struct sume_adapter *adapter)
{
	int i, last_ifc;

	KASSERT(mtx_owned(&adapter->lock), ("SUME lock not owned"));

	if (!adapter->running)
		return;

	last_ifc = adapter->last_ifc;

	/* Check all interfaces */
	for (i = last_ifc + 1; i < last_ifc + SUME_NPORTS + 1; i++) {
		struct ifnet *ifp = adapter->ifp[i % SUME_NPORTS];

		if (!(ifp->if_flags & IFF_UP))
			continue;

		if (!sume_if_start_locked(ifp))
			break;
	}
}

static int
sume_ifp_alloc(struct sume_adapter *adapter, uint32_t port)
{
	struct ifnet *ifp;
	struct nf_priv *nf_priv = malloc(sizeof(struct nf_priv), M_SUME,
	    M_ZERO | M_WAITOK);

	ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(adapter->dev, "Cannot allocate ifnet\n");
		return (ENOMEM);
	}

	adapter->ifp[port] = ifp;
	ifp->if_softc = nf_priv;

	nf_priv->unit = alloc_unr(unr);

	if_initname(ifp, SUME_ETH_DEVICE_NAME, nf_priv->unit);
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;

	ifp->if_init = sume_if_init;
	ifp->if_start = sume_if_start;
	ifp->if_ioctl = sume_if_ioctl;

	nf_priv->adapter = adapter;
	nf_priv->ifp = ifp;
	nf_priv->port = port;
	nf_priv->riffa_channel = SUME_RIFFA_CHANNEL_DATA;

	uint8_t hw_addr[ETHER_ADDR_LEN] = DEFAULT_ETHER_ADDRESS;
	hw_addr[ETHER_ADDR_LEN-1] = nf_priv->unit;
	ether_ifattach(ifp, hw_addr);

	ifmedia_init(&nf_priv->media, IFM_IMASK, sume_media_change,
	    sume_media_status);
	ifmedia_add(&nf_priv->media, IFM_ETHER | IFM_10G_SR, 0, NULL);
	ifmedia_set(&nf_priv->media, IFM_ETHER | IFM_10G_SR);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;

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
	*p = (struct riffa_chnl_dir **) malloc(SUME_RIFFA_CHANNELS *
	    sizeof(struct riffa_chnl_dir *), M_SUME, M_ZERO | M_WAITOK);
	if (*p == NULL) {
		device_printf(dev, "%s: malloc(%s) failed.\n", __func__, dir);
		return (error);
	}

	rp = *p;
	/* Allocate the chnl_dir structs themselves. */
	for (i = 0; i < SUME_RIFFA_CHANNELS; i++) {
		/* One direction. */
		rp[i] = (struct riffa_chnl_dir *)
		    malloc(sizeof(struct riffa_chnl_dir), M_SUME,
		    M_ZERO | M_WAITOK);
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
		    adapter->sg_buf_size, // CHECK
		    1,
		    adapter->sg_buf_size, // CHECK
		    0,
		    NULL,
		    NULL,
		    &rp[i]->my_tag);

		if (err) {
			device_printf(dev, "%s: bus_dma_tag_create(%s[%d]) "
			    "failed.\n", __func__, dir, i);
			return (err);
		}

		err = bus_dmamem_alloc(rp[i]->my_tag, (void **)
		    &rp[i]->buf_addr, BUS_DMA_WAITOK | BUS_DMA_COHERENT |
		    BUS_DMA_ZERO, &rp[i]->my_map);
		if (err) {
			device_printf(dev, "%s: bus_dmamem_alloc(%s[%d]) "
			    "rp[i]->buf_addr failed.\n", __func__, dir, i);
			return (err);
		}

		bzero(rp[i]->buf_addr, adapter->sg_buf_size);

		err = bus_dmamap_load(rp[i]->my_tag, rp[i]->my_map,
		    rp[i]->buf_addr, adapter->sg_buf_size, callback_dma,
		    &hw_addr, BUS_DMA_NOWAIT);
		if (err) {
			device_printf(dev, "%s: bus_dmamap_load(%s[%d]) "
			    "hw_addr failed.\n", __func__, dir, i);
			return (err);
		}
		rp[i]->buf_hw_addr = hw_addr;

		rp[i]->rtag = -3;
		rp[i]->state = SUME_RIFFA_CHAN_STATE_IDLE;
	}

	return (0);
}

static int
sume_probe_riffa_buffers(struct sume_adapter *adapter)
{
	int error;

	error = sume_probe_riffa_buffer(adapter, &adapter->recv, "recv");
	if (error)
		return (error);

	error = sume_probe_riffa_buffer(adapter, &adapter->send, "send");

	return (error);
}

static int
get_modreg_value(struct nf_priv *nf_priv, struct sume_ifreq *sifr)
{
	int error;

	error = sume_module_reg_write(nf_priv, sifr, 0x00);
	if (error)
		return (error);

	error = sume_module_reg_read(nf_priv, sifr);
	if (error)
		return (error);

	return(0);
}

static void
sume_sysctl_init(struct sume_adapter *adapter)
{
	device_t dev = adapter->dev;
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(dev);
	struct sysctl_oid *tree = device_get_sysctl_tree(dev);
	struct sysctl_oid_list *child = SYSCTL_CHILDREN(tree);
	struct sysctl_oid *tmp_tree;
	int i;

	tree = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "sume", CTLFLAG_RW,
	    0, "SUME top-level tree");
	if (tree == NULL) {
		device_printf(dev, "SYSCTL_ADD_NODE failed.\n");
		return;
	}
	SYSCTL_ADD_INT(ctx, child, OID_AUTO, "debug", CTLFLAG_RW,
	    &adapter->sume_debug, 0, "debug int leaf");

	/* total RX error stats */
	SYSCTL_ADD_U64(ctx, child, OID_AUTO, "rx_epkts",
	    CTLFLAG_RD, &adapter->packets_err, 0, "rx errors");
	SYSCTL_ADD_U64(ctx, child, OID_AUTO, "rx_ebytes",
	    CTLFLAG_RD, &adapter->bytes_err, 0, "rx error bytes");

#define	IFC_NAME_LEN 4
	char namebuf[IFC_NAME_LEN];

	for (i = SUME_NPORTS - 1; i >= 0; i--) {
		struct ifnet *ifp = adapter->ifp[i];
		if (ifp == NULL)
			continue;

		struct nf_priv *nf_priv = ifp->if_softc;

		snprintf(namebuf, IFC_NAME_LEN, "nf%d", nf_priv->unit);
		tmp_tree = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, namebuf,
		    CTLFLAG_RW, 0, "SUME ifc tree");
		if (tmp_tree == NULL) {
			device_printf(dev, "SYSCTL_ADD_NODE failed.\n");
			return;
		}

		/* HW RX stats */
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tmp_tree), OID_AUTO,
		    "hw_rx_packets", CTLFLAG_RD, &nf_priv->stats.hw_rx_packets,
		    0, "hw_rx packets");

		/* HW TX stats */
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tmp_tree), OID_AUTO,
		    "hw_tx_packets", CTLFLAG_RD, &nf_priv->stats.hw_tx_packets,
		    0, "hw_tx packets");

		/* RX stats */
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tmp_tree), OID_AUTO,
		    "rx_bytes", CTLFLAG_RD, &nf_priv->stats.rx_bytes, 0,
		    "rx bytes");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tmp_tree), OID_AUTO,
		    "rx_dropped", CTLFLAG_RD, &nf_priv->stats.rx_dropped, 0,
		    "rx dropped");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tmp_tree), OID_AUTO,
		    "rx_packets", CTLFLAG_RD, &nf_priv->stats.rx_packets, 0,
		    "rx packets");

		/* TX stats */
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tmp_tree), OID_AUTO,
		    "tx_bytes", CTLFLAG_RD, &nf_priv->stats.tx_bytes, 0,
		    "tx bytes");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tmp_tree), OID_AUTO,
		    "tx_dropped", CTLFLAG_RD, &nf_priv->stats.tx_dropped, 0,
		    "tx dropped");
		SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tmp_tree), OID_AUTO,
		    "tx_packets", CTLFLAG_RD, &nf_priv->stats.tx_packets, 0,
		    "tx packets");
	}
}

static void
sume_local_timer(void *arg)
{
	struct sume_adapter *adapter = arg;

	taskqueue_enqueue(adapter->tq, &adapter->stat_task);
	callout_reset(&adapter->timer, 1 * hz, sume_local_timer, adapter);
}

static void
sume_get_stats(void *context, int pending)
{
	struct sume_adapter *adapter = context;
	int i;

	for (i = 0; i < SUME_NPORTS; i++) {
		struct ifnet *ifp = adapter->ifp[i];
		if (ifp->if_flags & IFF_UP) {
			struct nf_priv *nf_priv = ifp->if_softc;
			struct sume_ifreq sifr;

			/* Get RX counter. */
			sifr.addr = SUME_STAT_RX_ADDR(nf_priv->port);
			sifr.val = 0;

			if (!get_modreg_value(nf_priv, &sifr))
				nf_priv->stats.hw_rx_packets += sifr.val;

			/* Get TX counter. */
			sifr.addr = SUME_STAT_TX_ADDR(nf_priv->port);
			sifr.val = 0;

			if (!get_modreg_value(nf_priv, &sifr))
				nf_priv->stats.hw_tx_packets += sifr.val;
		}
	}
}

static int
sume_attach(device_t dev)
{
	struct sume_adapter *adapter = device_get_softc(dev);
	adapter->dev = dev;
	int error, i;

	mtx_init(&adapter->lock, "Global lock", NULL, MTX_DEF);

	adapter->running = 0;

	/* OK finish up RIFFA. */
	error = sume_probe_riffa_pci(adapter);
	if (error != 0)
		goto error;

	error = sume_probe_riffa_buffers(adapter);
	if (error != 0)
		goto error;

	/* Now do the network interfaces. */
	for (i = 0; i < SUME_NPORTS; i++) {
		error = sume_ifp_alloc(adapter, i);
		if (error != 0)
			goto error;
	}

	/*  Register stats and register sysctls. */
	sume_sysctl_init(adapter);

	/* Reset the HW. */
	read_reg(adapter, RIFFA_INFO_REG_OFF);

	/* Ready to go, "enable" IRQ. */
	adapter->running = 1;

	callout_init(&adapter->timer, 1);
	TASK_INIT(&adapter->stat_task, 0, sume_get_stats, adapter);

	adapter->tq = taskqueue_create_fast("sume_stats", M_NOWAIT,
	    taskqueue_thread_enqueue, &adapter->tq);
	taskqueue_start_threads(&adapter->tq, 1, PI_NET, "%s stattaskq",
	    device_get_nameunit(adapter->dev));

	callout_reset(&adapter->timer, 1 * hz, sume_local_timer, adapter);

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

	for (i = 0; i < SUME_RIFFA_CHANNELS; i++) {
		if (pp[i] == NULL)
			continue;

		if (pp[i]->buf_hw_addr != 0) {
			bus_dmamem_free(pp[i]->my_tag, pp[i]->buf_addr,
			    pp[i]->my_map);
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
	int i;
	struct nf_priv *nf_priv;

	KASSERT(mtx_initialized(&adapter->lock), ("SUME mutex not "
	    "initialized"));
	adapter->running = 0;

	for (i = 0; i < SUME_NPORTS; i++) {
		struct ifnet *ifp = adapter->ifp[i];
		if (ifp == NULL)
			continue;

		ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
		nf_priv = ifp->if_softc;
		if (nf_priv != NULL) {
			if (ifp->if_flags & IFF_UP)
				if_down(ifp);
			ifmedia_removeall(&nf_priv->media);
			free_unr(unr, nf_priv->unit);
		}

		ifp->if_flags &= ~IFF_UP;
		ether_ifdetach(ifp);
		if_free(ifp);

		if (nf_priv != NULL)
			free(nf_priv, M_SUME);
	}

	/* Drain the stats callout and task queue. */
	callout_drain(&adapter->timer);

	if (adapter->tq) {
		taskqueue_drain(adapter->tq, &adapter->stat_task);
		taskqueue_free(adapter->tq);
	}

	sume_remove_riffa_buffers(adapter);

	if (adapter->irq.tag)
		bus_teardown_intr(dev, adapter->irq.res, adapter->irq.tag);
	if (adapter->irq.res)
		bus_release_resource(dev, SYS_RES_IRQ, adapter->irq.rid,
		    adapter->irq.res);

	pci_release_msi(dev);

	if (adapter->bar0_addr)
		bus_release_resource(dev, SYS_RES_MEMORY, adapter->rid,
		    adapter->bar0_addr);

	mtx_destroy(&adapter->lock);

	return (0);
}

static int
mod_event(module_t mod, int cmd, void *arg)
{
	int rc = 0;

	switch (cmd) {
	case MOD_LOAD:
		unr = new_unrhdr(0, INT_MAX, NULL);
		break;

	case MOD_UNLOAD:
		delete_unrhdr(unr);
		break;
	}

	return (rc);
}
static devclass_t sume_devclass;

DRIVER_MODULE(sume, pci, sume_driver, sume_devclass, mod_event, 0);
MODULE_VERSION(sume, 1);