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

#define	DEFAULT_ETHER_ADDRESS		"\02SUME\00"
#define	SUME_ETH_DEVICE_NAME		"nf"

#ifndef SUME_PORTS_MAX
#define	SUME_PORTS_MAX			4
#endif

#define	SUME_IOCTL_CMD_WRITE_REG	(SIOCGPRIVATE_0)
#define	SUME_IOCTL_CMD_READ_REG		(SIOCGPRIVATE_1)

#define	SUME_LOCK(adapter)	\
    mtx_lock(&adapter->lock);
#define	SUME_UNLOCK(adapter)	\
    mtx_unlock(&adapter->lock);

/* Currently SUME only uses two fixed channels for all port traffic and regs. */
#define	SUME_RIFFA_CHANNEL_DATA		0
#define	SUME_RIFFA_CHANNEL_REG		1	/* See description at top. */
#define	SUME_RIFFA_CHANNELS		2

/* RIFFA constants. */
#define	RIFFA_MAX_CHNLS			12
#define	RIFFA_MAX_BUS_WIDTH_PARAM	4
#define	RIFFA_SG_BUF_SIZE		(4*1024)
#define	RIFFA_SG_ELEMS			200

/* RIFFA register offsets. */
#define	RIFFA_RX_SG_LEN_REG_OFF		0x0
#define	RIFFA_RX_SG_ADDR_LO_REG_OFF	0x1
#define	RIFFA_RX_SG_ADDR_HI_REG_OFF	0x2
#define	RIFFA_RX_LEN_REG_OFF		0x3
#define	RIFFA_RX_OFFLAST_REG_OFF	0x4
#define	RIFFA_TX_SG_LEN_REG_OFF		0x5
#define	RIFFA_TX_SG_ADDR_LO_REG_OFF	0x6
#define	RIFFA_TX_SG_ADDR_HI_REG_OFF	0x7
#define	RIFFA_TX_LEN_REG_OFF		0x8
#define	RIFFA_TX_OFFLAST_REG_OFF	0x9
#define	RIFFA_INFO_REG_OFF		0xA
#define	RIFFA_IRQ_REG0_OFF		0xB
#define	RIFFA_IRQ_REG1_OFF		0xC
#define	RIFFA_RX_TNFR_LEN_REG_OFF	0xD
#define	RIFFA_TX_TNFR_LEN_REG_OFF	0xE

#define	RIFFA_CHNL_REG(c, o)		((c << 4) + o)

/*
 * RIFFA state machine;
 * rather than using complex circular buffers for 1 transaction.
 */
#define	SUME_RIFFA_CHAN_STATE_IDLE	0x01
#define	SUME_RIFFA_CHAN_STATE_READY	0x02
#define	SUME_RIFFA_CHAN_STATE_READ	0x04
#define	SUME_RIFFA_CHAN_STATE_LEN	0x08

#define	SUME_CHAN_STATE_RECOVERY_FLAG	0x80000000

/* Various bits and pieces. */
#define	SUME_RIFFA_MAGIC		0xcafe

/* Accessor macros. */
#define	SUME_RIFFA_LAST(offlast)	((offlast) & 0x01)
#define	SUME_RIFFA_OFFSET(offlast)	((uint64_t)((offlast) >> 1) << 2)
#define	SUME_RIFFA_LEN(len)		((uint64_t)(len) << 2)

#define	SUME_RIFFA_LO_ADDR(addr)	(addr & 0xFFFFFFFF)
#define	SUME_RIFFA_HI_ADDR(addr)	((addr >> 32) & 0xFFFFFFFF)

#define	SUME_MSI_RXQUE			(1 << 0)
#define	SUME_MSI_RXBUF			(1 << 1)
#define	SUME_MSI_RXDONE			(1 << 2)
#define	SUME_MSI_TXBUF			(1 << 3)
#define	SUME_MSI_TXDONE			(1 << 4)

#define	SUME_DPORT_MASK			0xaa

struct irq {
	struct resource		*res;
	uint32_t		rid;
	void			*tag;
} __aligned(CACHE_LINE_SIZE);

struct riffa_chnl_dir {
	char			*buf_addr;	/* S/G addresses+len. */
	bus_addr_t		buf_hw_addr;	/* -- " -- mapped. */
	uint32_t		num_sg;
	uint32_t		state;
	uint32_t		flags;
	uint32_t		offlast;
	uint32_t		len;		/* words */
	uint32_t		rtag;

	bus_dma_tag_t		my_tag;
	bus_dmamap_t		my_map;

	/* Used only for register read/write */
	struct mtx		send_sleep;
	struct mtx		recv_sleep;
	uint32_t		event;
};

struct sume_ifreq {
	uint32_t		addr;
	uint32_t		val;
};

struct nf_priv {
	struct sume_adapter	*adapter;
	struct ifnet		*ifp;
	uint32_t		port;
	uint32_t		port_up;
	uint32_t		riffa_channel;
	struct ifmedia		media;
};

struct sume_adapter {
	device_t		dev;
	uint32_t		rid;
	struct resource		*bar0_addr;
	bus_size_t		bar0_len;
	bus_space_tag_t		bt;
	bus_space_handle_t	bh;
	struct irq		irq;
	uint32_t		num_chnls;
	uint32_t		num_sg;
	uint32_t		sg_buf_size;
	volatile int		running;
	struct ifnet		*ifp[4];
	struct mtx		lock;

	struct riffa_chnl_dir	**recv;
	struct riffa_chnl_dir	**send;

	uint32_t		last_ifc;
};

/* SUME metadata:
 * sport - not used for RX. For TX, set to 0x02, 0x08, 0x20, 0x80, depending on
 *     the sending interface (nf0, nf1, nf2 or nf3).
 * dport - For RX, is set to 0x02, 0x08, 0x20, 0x80, depending on the receiving
 *     interface (nf0, nf1, nf2 or nf3). For TX, set to 0x01, 0x04, 0x10, 0x40,
 *     depending on the sending HW interface (nf0, nf1, nf2 or nf3).
 * plen - length of the send/receive packet data (in bytes)
 * magic - SUME hardcoded magic number which should be 0xcafe
 * t1, t1 - could be used for timestamping by SUME
 */
struct nf_metadata {
	uint16_t		sport;
	uint16_t		dport;
	uint16_t		plen;
	uint16_t		magic;
	uint32_t		t1;
	uint32_t		t2;
};

/* Used for ioctl communication with the rwaxi program used to read/write SUME
 *    internally defined register data.
 * addr - address of the SUME module register to read/write
 * val - value to write/read to/from the register
 * rtag - returned on read: transaction tag, for syncronization
 * strb - 0x1f when writing, 0x00 for reading
 */
struct nf_regop_data {
	uint32_t		addr;
	uint32_t		val;
	uint32_t		rtag;
	uint32_t		strb;
};

/* Our bouncebuffer "descriptor". This holds our physical address (lower and
 * upper values) of the beginning of the DMA data to RX/TX. The len is number
 * of words to transmit.
 */
struct nf_bb_desc {
	uint32_t		lower;
	uint32_t		upper;
	uint32_t		len;
};
