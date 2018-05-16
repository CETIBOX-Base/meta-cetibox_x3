/*
 * Copyright (c) 2017 CETITEC GmbH
 *
 * This software is licensed under the terms of the GNU General Public License (GPL) Version 2.
 *
 * The terms of the license can be found in the main directory of the delivery:
 *
 * GPL2 License: LICENSE-GPL2.txt
 */
#ifndef SJA1105_H_
#define SJA1105_H_

#include <linux/kernel.h>

#define SWITCH_PORTS 5
#define SWITCH_MGMTROUTE_INDEX 0
#define SWITCH_MGMT_READ_RETRIES 10

#define PTP_ETHERTYPE 0x88F7
#define P8021AS_MULTICAST "\x01\x80\xC2\x00\x00\x0E"
#define PTP_MAC_VAL 0x0180C200000Eull

#define SJA1105_SPI_CMD(write, wordcount, address)   \
		(uint32_t) ((((write) ? 1:0) << 31)          \
		            | (((wordcount) & 0x3f) << 25)   \
		            | (((address) & 0x1FFFFF) << 4))

#define SJA1105_EGRESS_TIMESTAMPS 0xC0
#define SJA1105_CONFSTATUS 0x1
#define SJA1105_STATUS 0x3
#define SJA1105_PTPTSCLK 0x1B

#define SJA1105_L2_ADDR_LOOKUP_RECONF 0x20

#define SJA1105_L2_ADDR_LOOKUP_RECONF_VALID (1 << 31)
#define SJA1105_L2_ADDR_LOOKUP_RECONF_RDWRSET (1 << 30)
#define SJA1105_L2_ADDR_LOOKUP_RECONF_VALIDENT (1 << 27)
#define SJA1105_L2_ADDR_LOOKUP_RECONF_MGMTROUTE (1 << 26)

#define SJA1105_L2_ADDR_LOOKUP_RECONF_ENFPORT (1 << 30)

#define SJA1105_MAC_RECONF 0x36
#define SJA1105_MAC_RECONF_VALID (1<<31)
#define SJA1105_MAC_RECONF_SPEED_OFS 29
#define SJA1105_MAC_RECONF_PORT_OFS 24
#define SJA1105_MAC_RECONF_DRPDTAG (1<<23)
#define SJA1105_MAC_RECONF_DRPUNTAG (1<<22)
#define SJA1105_MAC_RECONF_RETAG (1<<21)
#define SJA1105_MAC_RECONF_DYN_LEARN  (1<<20)
#define SJA1105_MAC_RECONF_EGRESS (1<<19)
#define SJA1105_MAC_RECONF_INGRESS (1<<18)
#define SJA1105_MAC_RECONF_INGMIRR (1<<17)
#define SJA1105_MAC_RECONF_EGRMIRR (1<<16)
#define SJA1105_MAC_RECONF_VLANPRIO_OFS 12
#define SJA1105_MAC_RECONF_VLANID_OFS 0

#define SJA1105_MAC_RECONF_DEFAULT (SJA1105_MAC_RECONF_VALID | \
									SJA1105_MAC_RECONF_DYN_LEARN | \
									SJA1105_MAC_RECONF_EGRESS | \
									SJA1105_MAC_RECONF_INGRESS)

#define SJA1105_CGU_IDIV0 0x10000b
#define SJA1105_CGU_MII0_RGMII_TX_CLK 0x100016

#define SJA1105_CGU_CLKSRC_OFS 24
#define SJA1105_CGU_AUTOBLOCK (1<<11)
#define SJA1105_CGU_IDIV_OFS 2
#define SJA1105_CGU_PD (1<<0)

#define SJA1105_CLKSRC_PLL0 0xb
#define SJA1105_CLKSRC_IDIV0 0x11
#define SJA1105_CLKSRC_REF25 0xa

struct sja1105_egress_ts {
	struct completion received;
	uint64_t ts;
};

typedef void (*sja1105_spi_ts_completion)(struct list_head *list, uint64_t ts,
                                          void *ctx);

struct sja1105_phy;

struct sja1105_poll_config {
	uint32_t mac_config[5];
	bool poll_enable;
};

#define SJA1105_IOC_POLL _IOW(SPI_IOC_MAGIC, 255, struct sja1105_poll_config)

struct sja1105_spi
{
	uint8_t *rxbuf;
	uint8_t *txbuf;
	dma_addr_t rxdma;
	dma_addr_t txdma;
	int users;
	int devnum;
	struct device *chardev;

	spinlock_t spi_lock;
	struct spi_device *dev;
	struct task_struct *poller;
	unsigned mgmt_index;

	struct sja1105_egress_ts egress_ts[SWITCH_PORTS][2];

	spinlock_t ts_complete_lock;
	sja1105_spi_ts_completion ts_complete_cb;
	struct list_head ts_complete_list;
	void *ts_complete_ctx;
	struct mutex buf_lock;

	bool switch_confd;
	struct sja1105_poll_config poll_config;
	struct sja1105_phy *phys;
	unsigned num_phys;
};

/* Recreate real timestamp according to AH1402 */
static inline uint64_t sja1105_recreate_ts(uint64_t clkval, uint32_t tsval)
{
	uint64_t clkval_upper = clkval >> 24;
	if ((clkval & 0xFFFFFF) < tsval) {
		clkval_upper--;
	}
	/* Multiply by 8ns because that's the switch's resolution */
	return ((clkval_upper << 24) | tsval) * 8;
}


void sja1105_spi_next_clkval_async(struct sja1105_spi *sja,
                                   struct list_head *list_entry,
                                   sja1105_spi_ts_completion cb, void *cb_ctx);

int sja1105_spi_prepare_egress_ptp(struct sja1105_spi *sja, uint8_t portnumber,
                                   bool timestamp, unsigned tsreg);

int sja1105_spi_wait_egress_ts(struct sja1105_spi *sja, uint8_t portnumber,
                               uint64_t *timestamp, int timeout_ms,
                               unsigned tsreg);

int sja1105_find_by_spi(unsigned bus, unsigned chip_select);
struct sja1105_spi *sja1105_spi_get(int minor);
void sja1105_spi_put(struct sja1105_spi *sja);
#endif
