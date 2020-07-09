/*-
 * Copyright (c) 2015 Bjoern A. Zeeb
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
#include <machine/atomic.h>

#include <net/if_media.h>
#include <netinet/in.h>
#include <netinet/in_var.h>
#include <netinet/if_ether.h>

#include "adapter.h"

#define	PCI_VENDOR_ID_XILINX 0x10ee

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
*/

MALLOC_DECLARE(M_SUME);
MALLOC_DEFINE(M_SUME, "sume", "NetFPGA SUME device driver");

static unsigned int sume_nports __read_mostly = SUME_PORTS_MAX;
TUNABLE_INT("sume.nports", &sume_nports);

static int mod_event(module_t, int, void *);
void sume_intr_handler(void *);
static int sume_intr_filter(void *);
static int sume_if_ioctl(struct ifnet *, unsigned long, caddr_t);
static int sume_fill_bb_desc(struct sume_adapter *,
    struct riffa_chnl_dir *, uint64_t);
static void check_queues(struct sume_adapter *);
static inline uint32_t read_reg(struct sume_adapter *, int);
static inline void write_reg(struct sume_adapter *, int, uint32_t);

//#define	DEBUG // replace with sysctl variable

struct {
	uint16_t device;
	char *desc;
} sume_pciids[] = {
	{0x7028, "NetFPGA SUME"},
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

/* Building mbuf for packet received from SUME. We expect to receive 'len'
 * bytes of data (including metadata) written from the bouncebuffer address
 * buf_addr+3*sizeof(uint32_t). Metadata will tell us which SUME interface
 * received the packet (sport will be 1, 2, 4 or 8), the packet length (plen),
 * and the magic word needs to be 0xcafe. When we have the packet data, we
 * create an mbuf and copy the data to it using m_copyback() function, set the
 * correct interface to rcvif and send the packet to the OS with if_input.
 */
static int
sume_rx_build_mbuf(struct sume_adapter *adapter, int i, uint32_t len)
{
	struct mbuf *m;
	struct ifnet *ifp;
	struct nf_priv *nf_priv;
	int np;
	uint16_t sport, dport, plen, magic;
	uint8_t *indata = (uint8_t *) adapter->recv[i]->buf_addr + sizeof(struct
		nf_bb_desc);
	device_t dev = adapter->dev;
	struct nf_metadata *mdata = (struct nf_metadata *) indata;

	/* The metadata header is 16 bytes. */
	if (len < sizeof(struct nf_metadata)) {
		device_printf(dev, "%s: short frame (%d)\n",
		    __func__, len);
		return (EINVAL);
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
		return(ENOMEM);
	}

	/* We got the packet from one of the even bits */
	np = (ffs(dport & SUME_DPORT_MASK) >> 1) - 1;
	if (np > sume_nports) {
		device_printf(dev, "%s: invalid destination port 0x%04x"
		    "(%d)\n", __func__, dport, np);
		return (EINVAL);
	}
	ifp = adapter->ifp[np];

	/* If the interface is down, well, we are done. */
	nf_priv = ifp->if_softc;
	if (nf_priv->port_up == 0) {
		device_printf(dev, "Device not up.\n");
		return (ENETDOWN);
	}

#ifdef DEBUG
	printf("Building mbuf with length: %d\n", plen);
#endif
	m = m_getm(NULL, plen, M_NOWAIT, MT_DATA);
	if (m == NULL) {
		m_freem(m);
		return (ENOMEM);
	}

	/* Copy the data in at the right offset. */
	m_copyback(m, 0, plen, (void *) (indata + sizeof(struct nf_metadata)));
	m->m_pkthdr.rcvif = ifp;

	(*ifp->if_input)(ifp, m);

	return (0);
}

/* Packet to transmit. We take the packet data from the mbuf and copy it to the
 * bouncebuffer address buf_addr+3*sizeof(uint32_t)+16. The 16 bytes before the
 * packet data are for metadata: sport/dport (depending on our source
 * interface), packet length and magic 0xcafe. We tell the SUME about the
 * transfer, fill the first 3*sizeof(uint32_t) bytes of the bouncebuffer with
 * the informaton about the start and length of the packet and trigger the
 * transaction.
 */
static int
sume_start_xmit(struct ifnet *ifp, struct mbuf *m)
{
	struct sume_adapter *adapter;
	struct nf_priv *nf_priv;
	uint8_t *outbuf;
	int error, i, last, offset;
	device_t dev;
	struct nf_metadata *mdata;

	nf_priv = ifp->if_softc;
	adapter = nf_priv->adapter;
	i = nf_priv->riffa_channel;
	dev = adapter->dev;

#ifdef DEBUG
	printf("Sending %d bytes to nf%d\n", m->m_pkthdr.len, nf_priv->port);
#endif

	/* Small packets need to be padded and their len expanded? */

	KASSERT(mtx_owned(&adapter->lock), ("SUME lock not owned"));

	outbuf = (uint8_t *) adapter->send[i]->buf_addr + sizeof(struct
		nf_bb_desc);
	mdata = (struct nf_metadata *) outbuf;
	/*
	 * Check state. It's the best we can do for now.
	 */
	if (adapter->send[i]->state != SUME_RIFFA_CHAN_STATE_IDLE) {
		device_printf(dev, "%s: SUME not in IDLE state (state %d)\n",
		    __func__, adapter->send[i]->state);
		//m_freem(m);
		return (EBUSY);
	}
	/* Clear the recovery flag. */
	adapter->send[i]->flags &= ~SUME_CHAN_STATE_RECOVERY_FLAG;

	/* Make sure we fit with the 16 bytes nf_metadata. */
	if ((m->m_pkthdr.len + sizeof(struct nf_metadata)) >
	    adapter->sg_buf_size) {
		device_printf(dev, "%s: Packet too big for bounce buffer "
		    "(%d)\n", __func__, m->m_pkthdr.len);
		m_freem(m);
		return (ENOMEM);
	}

	bus_dmamap_sync(adapter->send[i]->my_tag, adapter->send[i]->my_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	/* Skip the first 16 bytes for the metadata. */
	m_copydata(m, 0, m->m_pkthdr.len, outbuf + sizeof(struct nf_metadata));
	adapter->send[i]->len = sizeof(struct nf_metadata) / 4;	/* words */
	adapter->send[i]->len += (m->m_pkthdr.len / 4) +
	    ((m->m_pkthdr.len % 4 == 0) ? 0 : 1);

	/* Fill in the metadata. */
	mdata->sport = htole16(
	    1 << (nf_priv->port * 2 + 1)); /* CPU(DMA) ports are odd. */
	mdata->dport = htole16(
	    1 << (nf_priv->port * 2)); /* MAC ports are even. */
	mdata->plen = htole16(m->m_pkthdr.len);
	mdata->magic = htole16(SUME_RIFFA_MAGIC);
	mdata->t1 = htole32(0);
	mdata->t2 = htole32(0);

	/* Let the FPGA know about the transfer. */
	offset = 0;
	last = 1;
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_OFFLAST_REG_OFF),
	    ((offset << 1) | (last & 0x01)));
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_LEN_REG_OFF),
	    adapter->send[i]->len);		/* words */

	/* Fill the S/G map. */
	error = sume_fill_bb_desc(adapter,
	    adapter->send[i], SUME_RIFFA_LEN(adapter->send[i]->len));
	if (error) {
		device_printf(dev, "%s: failed to map S/G buffer\n", __func__);
		return (ENOMEM);
	}

	/* Update the state before intiating the DMA to avoid races. */
	adapter->send[i]->state = SUME_RIFFA_CHAN_STATE_READY;

	bus_dmamap_sync(adapter->send[i]->my_tag, adapter->send[i]->my_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	/* DMA. */
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_SG_ADDR_LO_REG_OFF),
	    SUME_RIFFA_LO_ADDR(adapter->send[i]->buf_hw_addr));
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_SG_ADDR_HI_REG_OFF),
	    SUME_RIFFA_HI_ADDR(adapter->send[i]->buf_hw_addr));
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_SG_LEN_REG_OFF),
	    4 * adapter->send[i]->num_sg);

	bus_dmamap_sync(adapter->send[i]->my_tag, adapter->send[i]->my_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	/* We can free as long as we use the bounce buffer. */
	m_freem(m);

	return (0);
}

/* SUME interrupt handler for when we get a valid interrupt from the board.
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
void
sume_intr_handler(void *arg)
{
	struct sume_adapter *adapter = arg;
	uint32_t vect, vect0, vect1, len;
	int i, error, loops;
	device_t dev = adapter->dev;

	SUME_LOCK(adapter);

	vect0 = read_reg(adapter, RIFFA_IRQ_REG0_OFF);
	if((vect0 & 0xC0000000) != 0) {
		SUME_UNLOCK(adapter);
		return;
	}
	if (adapter->num_chnls > 6) {
		vect1 = read_reg(adapter, RIFFA_IRQ_REG1_OFF);
		if((vect1 & 0xC0000000) != 0) {
			SUME_UNLOCK(adapter);
			return;
		}
	} else
		vect1 = 0;

	/*
	 * We only have one interrupt for all channels and no way
	 * to quickly lookup for which channel(s) we got an interrupt?
	 */
	for (i = 0; i < adapter->num_chnls; i++) {
		if (i < 6)
			vect = vect0 >> (5 * i);
		else
			vect = vect1 >> (5 * i);

		loops = 0;
		while ((vect & (SUME_MSI_TXBUF | SUME_MSI_TXDONE)) &&
		    loops <= 5) {
#ifdef DEBUG
			device_printf(dev, "%s: TX ch %d state %u vect = "
			    "0x%08x\n", __func__, i, adapter->send[i]->state,
			    vect);
#endif
			switch (adapter->send[i]->state) {
			case SUME_RIFFA_CHAN_STATE_IDLE:
				break;
			case SUME_RIFFA_CHAN_STATE_READY:
				if (vect & SUME_MSI_TXBUF) {
					adapter->send[i]->state =
					    SUME_RIFFA_CHAN_STATE_READ;
					vect &= ~SUME_MSI_TXBUF;
				} else {
					device_printf(dev, "%s: ch %d "
					    "unexpected interrupt in send+3 "
					    "state %u: vect = 0x%08x\n",
					    __func__, i,
					    adapter->send[i]->state, vect);
					adapter->send[i]->flags |=
					    SUME_CHAN_STATE_RECOVERY_FLAG;
				}
				break;
			case SUME_RIFFA_CHAN_STATE_READ:
				if (vect & SUME_MSI_TXDONE) {
					adapter->send[i]->state =
					    SUME_RIFFA_CHAN_STATE_LEN;

					len = read_reg(adapter,
					    RIFFA_CHNL_REG(i,
					    RIFFA_RX_TNFR_LEN_REG_OFF));
					if (i == SUME_RIFFA_CHANNEL_DATA) {
						adapter->send[i]->state =
						    SUME_RIFFA_CHAN_STATE_IDLE;
						check_queues(adapter);
					} else if (i == SUME_RIFFA_CHANNEL_REG) {
						wakeup(&adapter->send[i]->event);
						printf("Wake up send "
						    "thread!\n");
					} else {
						device_printf(dev, "%s: "
						    "interrupt on ch %d "
						    "unexpected in send+4 "
						    "state %u: vect = "
						    "0x%08x\n", __func__, i,
						    adapter->send[i]->state,
						    vect);
						adapter->send[i]->flags |=
						    SUME_CHAN_STATE_RECOVERY_FLAG;
					}
					vect &= ~SUME_MSI_TXDONE;
				} else {
					device_printf(dev, "%s: ch %d "
					    "unexpected interrupt in send+4 "
					    "state %u: vect = 0x%08x\n",
					    __func__, i,
					    adapter->send[i]->state, vect);
					adapter->send[i]->flags |=
					    SUME_CHAN_STATE_RECOVERY_FLAG;
				}
				break;
			case SUME_RIFFA_CHAN_STATE_LEN:
				break;
			default:
				printf("WARNON!\n");
			}
			loops++;
		}

		if ((vect & (SUME_MSI_TXBUF | SUME_MSI_TXDONE)) &&
		    ((adapter->send[i]->flags & SUME_CHAN_STATE_RECOVERY_FLAG)
		    != 0))
			device_printf(dev, "%s: ignoring vect = 0x%08x during "
			    "TX; not in recovery; state = %d loops = %d\n",
			    __func__, vect, adapter->send[i]->state, loops);

		loops = 0;
		while ((vect & (SUME_MSI_RXQUE | SUME_MSI_RXBUF |
			SUME_MSI_RXDONE)) && loops < 5) {
#ifdef DEBUG
			device_printf(dev, "%s: RX ch %d state %u vect = "
			    "0x%08x\n", __func__, i, adapter->recv[i]->state,
			    vect);
#endif
			switch (adapter->recv[i]->state) {
			case SUME_RIFFA_CHAN_STATE_IDLE:
				if (vect & SUME_MSI_RXQUE) {
					uint32_t max_ptr;

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

					/* Boundary checks. */
					max_ptr = (uint32_t) ((char *)
					    adapter->recv[i]->buf_addr
					    + SUME_RIFFA_OFFSET(
					    adapter->recv[i]->offlast) +
					    SUME_RIFFA_LEN(
					    adapter->recv[i]->len) - 1);
					if (max_ptr < (uint32_t)
					    adapter->recv[i]->buf_addr) {
						device_printf(dev, "%s: "
						    "receive buffer "
						    "wrap-around overflow.\n",
						    __func__);
					}
					if ((SUME_RIFFA_OFFSET(
					    adapter->recv[i]->offlast) +
					    SUME_RIFFA_LEN(
					    adapter->recv[i]->len)) >
					    adapter->sg_buf_size) {
						device_printf(dev, "%s: "
						    "receive buffer too "
						    "small.\n", __func__);
					}

					/* Build and load S/G map. */
					error = sume_fill_bb_desc(adapter,
					    adapter->recv[i], SUME_RIFFA_LEN(
					    adapter->recv[i]->len));
					if (error != 0) {
						device_printf(dev, "%s: "
						    "Failed to build S/G "
						    "map.\n", __func__);
					}
					bus_dmamap_sync(
					    adapter->recv[i]->my_tag,
					    adapter->recv[i]->my_map,
					    BUS_DMASYNC_PREREAD |
					    BUS_DMASYNC_PREWRITE);
					write_reg(adapter,
					    RIFFA_CHNL_REG(i,
					    RIFFA_TX_SG_ADDR_LO_REG_OFF),
					    SUME_RIFFA_LO_ADDR(
					    adapter->recv[i]->buf_hw_addr));
					write_reg(adapter,
					    RIFFA_CHNL_REG(i,
					    RIFFA_TX_SG_ADDR_HI_REG_OFF),
					    SUME_RIFFA_HI_ADDR(
					    adapter->recv[i]->buf_hw_addr));
					write_reg(adapter,
					    RIFFA_CHNL_REG(i,
					    RIFFA_TX_SG_LEN_REG_OFF),
					    4 * adapter->recv[i]->num_sg);
					bus_dmamap_sync(
					    adapter->recv[i]->my_tag,
					    adapter->recv[i]->my_map,
					    BUS_DMASYNC_POSTREAD |
					    BUS_DMASYNC_POSTWRITE);

					adapter->recv[i]->state =
					    SUME_RIFFA_CHAN_STATE_READY;
					vect &= ~SUME_MSI_RXQUE;
				} else {
					device_printf(dev, "%s: ch %d "
					    "unexpected interrupt in recv+0 "
					    "state %u: vect = 0x%08x\n",
					    __func__, i,
					    adapter->recv[i]->state, vect);
					adapter->recv[i]->flags |=
					    SUME_CHAN_STATE_RECOVERY_FLAG;
				}
				break;
			case SUME_RIFFA_CHAN_STATE_READY:
				if (vect & SUME_MSI_RXBUF) {
					adapter->recv[i]->state =
					    SUME_RIFFA_CHAN_STATE_READ;
					vect &= ~SUME_MSI_RXBUF;
				} else {
					device_printf(dev, "%s: ch %d "
					    "unexpected interrupt in recv+1 "
					    "state %u: vect = 0x%08x\n",
					    __func__, i,
					    adapter->recv[i]->state, vect);
					adapter->recv[i]->flags |=
					    SUME_CHAN_STATE_RECOVERY_FLAG;
				}
				break;
			case SUME_RIFFA_CHAN_STATE_READ:
				if (vect & SUME_MSI_RXDONE) {
					len = read_reg(adapter,
					    RIFFA_CHNL_REG(i,
					    RIFFA_TX_TNFR_LEN_REG_OFF));

					/*
					 * Remember, len and recv[i]->len
					 * are words.
					 */
					if (i == SUME_RIFFA_CHANNEL_DATA) {
						error = sume_rx_build_mbuf(
						    adapter, i, len << 2);
						adapter->recv[i]->state =
						    SUME_RIFFA_CHAN_STATE_IDLE;
					} else if (i ==
					    SUME_RIFFA_CHANNEL_REG) {
						wakeup(&adapter->recv[i]->event);
						printf("Wake up recv "
						    "thread!\n");
					} else {
						device_printf(dev, "%s: "
						    "interrupt on ch %d "
						    "unexpected in recv+2 "
						    "state %u: vect = "
						    "0x%08x\n", __func__, i,
						    adapter->recv[i]->state,
						    vect);
						adapter->recv[i]->flags |=
						    SUME_CHAN_STATE_RECOVERY_FLAG;
					}
					vect &= ~SUME_MSI_RXDONE;

				} else {
					device_printf(dev, "%s: ch %d "
					    "unexpected interrupt in recv+2 "
					    "state %u: vect = 0x%08x\n",
					    __func__, i,
					    adapter->recv[i]->state, vect);
					adapter->recv[i]->flags |=
					    SUME_CHAN_STATE_RECOVERY_FLAG;
				}
				break;
			case SUME_RIFFA_CHAN_STATE_LEN:
				break;
			default:
				printf("WARNON!\n");
			}
			loops++;
		}

		if ((vect & (SUME_MSI_RXQUE | SUME_MSI_RXBUF |
		    SUME_MSI_RXDONE)) && ((adapter->recv[i]->flags &
		    SUME_CHAN_STATE_RECOVERY_FLAG) != 0))
			device_printf(dev, "%s: ignoring vect = 0x%08x "
			    "during RX; not in recovery; state = %d, loops = "
			    "%d\n", __func__, vect, adapter->recv[i]->state,
			    loops);
	}
	SUME_UNLOCK(adapter);
}

/* Filtering interrupts. We wait for the adapter to go into the 'running' state
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
	if (atomic_load_int(&adapter->running) == 0)
		return (FILTER_STRAY);

	return (FILTER_SCHEDULE_THREAD);
}

static int
sume_probe_riffa_pci(struct sume_adapter *adapter)
{
	device_t dev = adapter->dev;
	int error, count, capmem;
	uint32_t reg, devctl, linkctl;

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
	adapter->num_chnls =	SUME_RIFFA_CHANNELS;
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

static void
sume_if_down(void *arg)
{
	struct ifnet *ifp = arg;
	struct nf_priv *nf_priv = ifp->if_softc;

	if (!nf_priv->port_up)
		return; /* already down */

	if_down(ifp);
	nf_priv->port_up = 0;

	printf("SUME nf%d down.\n", nf_priv->port);
}

static void
sume_if_up(void *arg)
{
	struct ifnet *ifp = arg;
	struct nf_priv *nf_priv = ifp->if_softc;

	if (nf_priv->port_up)
		return; /* already up */

	if_up(ifp);
	nf_priv->port_up = 1;

	printf("SUME nf%d up.\n", nf_priv->port);
}

/* Helper functions. */
static int
sume_fill_bb_desc(struct sume_adapter *adapter, struct riffa_chnl_dir *p,
    uint64_t len)
{
	uint32_t *bouncebuf;

	bouncebuf = (uint32_t *) p->buf_addr;

	bouncebuf[0] = (p->buf_hw_addr + sizeof(struct nf_bb_desc));
	bouncebuf[1] = (p->buf_hw_addr + sizeof(struct nf_bb_desc)) >> 32;
	bouncebuf[2] = len >> 2;

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
	    adapter->send[i]->len);	/* words */

	/* Fill the S/G map. */
	error = sume_fill_bb_desc(adapter, adapter->send[i],
	    SUME_RIFFA_LEN(adapter->send[i]->len));
	if (error != 0) {
		device_printf(dev, "%s: failed to map S/G buffer\n", __func__);
		return (EFAULT);
	}

	/* Update the state before intiating the DMA to avoid races. */
	adapter->send[i]->state = SUME_RIFFA_CHAN_STATE_READY;

	bus_dmamap_sync(adapter->send[i]->my_tag, adapter->send[i]->my_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	/* DMA. */
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_SG_ADDR_LO_REG_OFF),
	    SUME_RIFFA_LO_ADDR(adapter->send[i]->buf_hw_addr));
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_SG_ADDR_HI_REG_OFF),
	    SUME_RIFFA_HI_ADDR(adapter->send[i]->buf_hw_addr));
	write_reg(adapter, RIFFA_CHNL_REG(i, RIFFA_RX_SG_LEN_REG_OFF),
	    4 * adapter->send[i]->num_sg);
	bus_dmamap_sync(adapter->send[i]->my_tag, adapter->send[i]->my_map,
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
sume_initiate_reg_write(struct nf_priv *nf_priv, struct sume_ifreq *sifr,
    uint32_t strb)
{
	struct sume_adapter *adapter;
	struct nf_regop_data *data;
	int error = 0, i;

	adapter = nf_priv->adapter;

	/*
	 * 1. Make sure the channel is free;  otherwise return EBUSY.
	 * 2. Prepare the memory in the bounce buffer (which we always
	 *    use for regs).
	 * 3. Start the DMA process.
	 * 4. Sleep and wait for result and return success or error.
	 */
	i = SUME_RIFFA_CHANNEL_REG;
	mtx_lock(&adapter->send[i]->send_sleep);

	if (adapter->send[i]->state != SUME_RIFFA_CHAN_STATE_IDLE) {
		mtx_unlock(&adapter->send[i]->send_sleep);
		return (EBUSY);
	}

	data = (struct nf_regop_data *) (adapter->send[i]->buf_addr +
		sizeof(struct nf_bb_desc));
	data->addr = htole32(sifr->addr);
	data->val = htole32(sifr->val);
	/* Tag to indentify request. */
	data->rtag = htole32(++adapter->send[i]->rtag);
	data->strb = htole32(strb);
	adapter->send[i]->len = 4;	/* Words */

	error = sume_reg_wr_locked(adapter, i);
	if (error) {
		mtx_unlock(&adapter->send[i]->send_sleep);
		return (EFAULT);
	}

	/* Timeout after 1s. */
	error = msleep(&adapter->send[i]->event,
	    &adapter->send[i]->send_sleep, 0, "Waiting recv finish", 1 * hz);

	if (error == EWOULDBLOCK) {
		device_printf(adapter->dev, "%s: wait error: %d\n", __func__,
		    error);
		mtx_unlock(&adapter->send[i]->send_sleep);
		return (EWOULDBLOCK);
	}

	/* This was a write so we are done; were interrupted, or timed out. */
	if (strb != 0x00 || error != 0 || error == EWOULDBLOCK) {
		adapter->send[i]->state = SUME_RIFFA_CHAN_STATE_IDLE;
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

	mtx_unlock(&adapter->send[i]->send_sleep);

	return (error);
}

static int
sume_read_reg_result(struct nf_priv *nf_priv, struct sume_ifreq *sifr)
{
	struct sume_adapter *adapter;
	struct nf_regop_data *data;
	int error, i;
	device_t dev;

	adapter = nf_priv->adapter;
	dev = adapter->dev;

	/*
	 * 0. Sleep waiting for result if needed (unless condition is
	 *    true already).
	 * 2. Read DMA results.
	 * 3. Update state on *TX* to IDLE to allow next read to start.
	 */
	i = SUME_RIFFA_CHANNEL_REG;

	/* We only need to be woken up at the end of the transaction. */
	/* Timeout after 1s. */
	mtx_lock(&adapter->recv[i]->recv_sleep);
	bus_dmamap_sync(adapter->recv[i]->my_tag, adapter->recv[i]->my_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	error = msleep(&adapter->recv[i]->event,
	    &adapter->recv[i]->recv_sleep, 0, "Waiting transaction finish",
	    1 * hz);

	if (error == EWOULDBLOCK) {
		device_printf(dev, "%s: wait error: %d\n", __func__, error);
		mtx_unlock(&adapter->recv[i]->recv_sleep);
		return (EWOULDBLOCK);
	}
	bus_dmamap_sync(adapter->recv[i]->my_tag, adapter->recv[i]->my_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	/*
	 * Read reply data and validate address and tag.
	 * Note: we do access the send side without lock but the state
	 * machine does prevent the data from changing.
	 */
	data = (struct nf_regop_data *) (adapter->recv[i]->buf_addr +
		sizeof(struct nf_bb_desc));
	if (le32toh(data->rtag) != adapter->send[i]->rtag) {
		device_printf(dev, "%s: rtag error: 0x%08x 0x%08x\n", __func__,
		    le32toh(data->rtag), adapter->send[i]->rtag);
	}
	sifr->val = le32toh(data->val);
	adapter->recv[i]->state = SUME_RIFFA_CHAN_STATE_IDLE;
	mtx_unlock(&adapter->recv[i]->recv_sleep);

	/* We are done. */
	adapter->send[i]->state = SUME_RIFFA_CHAN_STATE_IDLE;

	return (0);
}

static int
sume_if_ioctl(struct ifnet *ifp, unsigned long cmd, caddr_t data)
{
	struct nf_priv *nf_priv;
	struct ifreq *ifr = (struct ifreq *) data;
	struct ifaddr *ifa = (struct ifaddr *) data;
	struct sume_ifreq sifr;
	int error = 0;
	struct sume_adapter *adapter;

	nf_priv = ifp->if_softc;
	if (nf_priv == NULL || nf_priv->adapter == NULL)
		return (EINVAL);

	adapter = nf_priv->adapter;

	switch (cmd) {
	case SIOCSIFFLAGS:
		if (atomic_load_int(&adapter->running) == 0)
			break;
		SUME_LOCK(adapter);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				sume_if_up(ifp);
		} else if (ifp->if_drv_flags & IFF_DRV_RUNNING)
			sume_if_down(ifp);
		SUME_UNLOCK(adapter);
		break;

	case SIOCGIFXMEDIA:
		if (atomic_load_int(&adapter->running) == 0)
			break;
		SUME_LOCK(adapter);
		ifmedia_ioctl(ifp, ifr, &nf_priv->media, cmd);
		SUME_UNLOCK(adapter);
		break;

	case SIOCSIFADDR:
	case SIOCAIFADDR:
		if (atomic_load_int(&adapter->running) == 0)
			break;
		SUME_LOCK(adapter);
		if (!nf_priv->port_up) {
			ifp->if_flags |= IFF_UP;
			nf_priv->port_up = 1;
		}
		SUME_UNLOCK(adapter);
		arp_ifinit(ifp, ifa);
		break;

	case SUME_IOCTL_CMD_WRITE_REG:
		error = copyin(ifr_data_get_ptr(ifr), &sifr, sizeof(sifr));
		if (error) {
			error = EINVAL;
			break;
		}
		error = sume_initiate_reg_write(nf_priv, &sifr, 0x1f);
		break;

	case SUME_IOCTL_CMD_READ_REG:
		error = copyin(ifr_data_get_ptr(ifr), &sifr, sizeof(sifr));
		if (error) {
			error = EINVAL;
			break;
		}

		error = sume_initiate_reg_write(nf_priv, &sifr, 0x00);
		if (error)
			break;

		error = sume_read_reg_result(nf_priv, &sifr);
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

	if (ifm->ifm_cur->ifm_media == (IFM_ETHER | IFM_10G_SR) &&
	    nf_priv->port_up)
		ifmr->ifm_active = IFM_ETHER | IFM_10G_SR;
	else
		ifmr->ifm_active = ifm->ifm_cur->ifm_media;

	ifmr->ifm_status |= IFM_ACTIVE;

	return;
}

static int
sume_if_start_locked(struct ifnet *ifp)
{
	struct mbuf *m_head;
	struct nf_priv *nf_priv = ifp->if_softc;
	struct sume_adapter *adapter = nf_priv->adapter;

	if (!IFQ_DRV_IS_EMPTY(&ifp->if_snd)) {
		IFQ_DRV_DEQUEUE(&ifp->if_snd, m_head);
		if (m_head == NULL)
			return (1);
		/*
		*  Failed xmit means we need to try again later so requeue
		*/
		if (sume_start_xmit(ifp, m_head)) {
			if (m_head != NULL)
				IFQ_DRV_PREPEND(&ifp->if_snd, m_head);
		}

		adapter->last_ifc = nf_priv->port;
		return (0);
	}

	return (1);
}

static void
sume_if_start(struct ifnet *ifp)
{
	struct nf_priv *nf_priv = ifp->if_softc;
	struct sume_adapter *adapter = nf_priv->adapter;
	int i = nf_priv->riffa_channel;

	if ((ifp->if_drv_flags & (IFF_DRV_RUNNING|IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING)
		return;
	if (!nf_priv->port_up)
		return;

	SUME_LOCK(adapter);
	if (adapter->send[i]->state != SUME_RIFFA_CHAN_STATE_IDLE) {
		//device_printf(adapter->dev, "%s: SUME not in IDLE state "
			//"(state %d)\n", __func__, adapter->send[i]->state);
		SUME_UNLOCK(adapter);
		return;
	}

	sume_if_start_locked(ifp);
	SUME_UNLOCK(adapter);
}

static void
check_queues(struct sume_adapter *adapter)
{
	int i, last;

	KASSERT(mtx_owned(&adapter->lock), ("SUME lock not owned"));

	last = adapter->last_ifc;

	/* Check all interfaces */
	for (i = last+1; i < last + sume_nports + 1; i++)
		if (!sume_if_start_locked(adapter->ifp[i % sume_nports]))
			break;
}

static void
sume_qflush(struct ifnet *ifp)
{

	// dummy qflush
}

static int
sume_ifp_alloc(struct sume_adapter *adapter, uint32_t port)
{
	struct ifnet *ifp;	
	struct nf_priv *nf_priv = malloc(sizeof(struct nf_priv), M_SUME,
	    M_ZERO | M_WAITOK);
	device_t dev = adapter->dev;

	ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "Cannot allocate ifnet\n");
		return (ENOMEM);
	}

	adapter->ifp[port] = ifp;
	ifp->if_softc = nf_priv;

	if_initname(ifp, SUME_ETH_DEVICE_NAME, port);
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;

	ifp->if_start = sume_if_start;
	ifp->if_ioctl = sume_if_ioctl;
	//ifp->if_transmit = sume_start_xmit;
	ifp->if_qflush = sume_qflush;

	nf_priv->adapter = adapter;
	nf_priv->ifp = ifp;
	nf_priv->port = port;
	nf_priv->riffa_channel = SUME_RIFFA_CHANNEL_DATA;

	uint8_t hw_addr[ETHER_ADDR_LEN] = DEFAULT_ETHER_ADDRESS;
	hw_addr[ETHER_ADDR_LEN-1] = port;
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
	*p = (struct riffa_chnl_dir **) malloc(adapter->num_chnls *
	    sizeof(struct riffa_chnl_dir *), M_SUME, M_ZERO | M_WAITOK);
	if (*p == NULL) {
		device_printf(dev, "%s: malloc(%s) failed.\n", __func__, dir);
		return (error);
	}

	rp = *p;
	/* Allocate the chnl_dir structs themselves. */
	for (i = 0; i < adapter->num_chnls; i++) {
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

		/* Initialize state. */
		mtx_init(&rp[i]->send_sleep, "Send sleep", NULL, MTX_DEF);
		mtx_init(&rp[i]->recv_sleep, "Recv sleep", NULL, MTX_DEF);

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
sume_attach(device_t dev)
{
	struct sume_adapter *adapter = device_get_softc(dev);
	adapter->dev = dev;
	int error, i;

	/* Start with the safety check to avoid malfunctions further down. */
	if (sume_nports < 1 || sume_nports > SUME_PORTS_MAX) {
		device_printf(dev, "%s: sume_nports out of range: %d (1..%d). "
		    "Using max.\n", __func__, sume_nports, SUME_PORTS_MAX);
		sume_nports = SUME_PORTS_MAX;
	}

	mtx_init(&adapter->lock, "Global lock", NULL, MTX_DEF);

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
		error = sume_ifp_alloc(adapter, i);
		if (error != 0)
			goto error;
	}

	/* Reset the HW. */
	read_reg(adapter, RIFFA_INFO_REG_OFF);

	/* Ready to go, "enable" IRQ. */
	atomic_set_int(&adapter->running, 1);

	bus_generic_attach(dev);

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

		if (pp[i]->buf_hw_addr != 0) {
			bus_dmamem_free(pp[i]->my_tag, pp[i]->buf_addr,
			    pp[i]->my_map);
			pp[i]->buf_hw_addr = 0;
		}

		mtx_destroy(&pp[i]->recv_sleep);
		mtx_destroy(&pp[i]->send_sleep);
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
	struct nf_priv *nf_priv;

	sume_remove_riffa_buffers(adapter);

	for (i = 0; i < sume_nports; i++) {
		// XXX not really safe, fix this
		nf_priv = adapter->ifp[i]->if_softc;
		if (nf_priv->port_up)
			if_down(adapter->ifp[i]);
		ifmedia_removeall(&nf_priv->media);
		if (adapter->ifp[i] != NULL) {
			ether_ifdetach(adapter->ifp[i]);
		}
		free(nf_priv, M_SUME);
	}

	mtx_destroy(&adapter->lock);

	rc = bus_generic_detach(dev);
	if (rc)
		return (rc);

	if (adapter->irq.tag)
		bus_teardown_intr(dev, adapter->irq.res, adapter->irq.tag);
	if (adapter->irq.res)
		bus_release_resource(dev, SYS_RES_IRQ, adapter->irq.rid,
		    adapter->irq.res);

	device_delete_children(dev);

	pci_release_msi(dev);

	if (adapter->bar0_addr)
		bus_release_resource(dev, SYS_RES_MEMORY, adapter->rid,
		    adapter->bar0_addr);

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
