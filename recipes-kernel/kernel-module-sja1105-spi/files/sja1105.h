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

// 24 bits, 8 ns unit
#define SWITCH_EPOCH_MS 134

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

#define SJA1105_MAC_RECONF_SPEED_MASK (0x3<<29)

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

struct sja1105_phy;

struct sja1105_poll_config {
	uint32_t mac_config[5];
	bool poll_enable;
};

#define SJA1105_IOC_POLL _IOW(SPI_IOC_MAGIC, 255, struct sja1105_poll_config)
#define SJA1105_IOC_SWITCHID _IOW(SPI_IOC_MAGIC, 254, uint8_t)

struct sja1105_spi
{
	struct kref ref;
	int devnum;
	struct device *chardev;

	spinlock_t spi_lock;
	struct spi_device *dev;
	struct task_struct *poller;
	unsigned mgmt_index;

	u8 *phy_spi_buf, *api_spi_buf, *poll_spi_buf, *user_spi_buf;
	struct mutex phy_lock, api_lock, user_buf_lock, poll_lock;

	uint64_t clkval;
	bool clkval_valid;

	bool switch_confd;
	struct sja1105_poll_config poll_config;
	struct sja1105_phy *phys;
	unsigned num_phys;
	bool enable_ptp;
#ifdef SPI_FAKE_SPEED
	uint32_t fake_speed;
#endif
	uint8_t switch_id;
};

bool sja1105_spi_is_clkval_valid(struct sja1105_spi *sja);

static inline uint64_t sja1105_recreate_ts(struct sja1105_spi *sja, uint32_t tsval)
{
	uint64_t clkval = READ_ONCE(sja->clkval);
	uint64_t clkval_upper = clkval >> 24;
	uint32_t clkval_lower = clkval & ((1<<24)-1);
	const uint32_t half_epoch = (1<<23)-1;
	int delta = tsval - clkval_lower;

	if (abs(delta) > half_epoch) {
		if (clkval_lower < tsval) {
			// clkval has wrapped since tsval was captured
			--clkval_upper;
		} else {
			// the clock has wrapped since clkval was captured
			++clkval_upper;
		}
	}

	/* Multiply by 8ns because that's the switch's resolution */
	return ((clkval_upper << 24) | tsval) * 8;
}

int sja1105_spi_prepare_egress_ptp(struct sja1105_spi *sja, uint8_t portnumber,
                                   bool timestamp, unsigned tsreg);
int sja1105_spi_prepare_egress_ptp_multiport(struct sja1105_spi *sja, uint8_t *portmask,
                                   bool timestamp, unsigned tsreg);

int sja1105_spi_wait_egress_ts(struct sja1105_spi *sja, uint8_t portmask,
                               uint64_t *timestamp, int timeout_ms,
                               unsigned tsreg);

struct sja1105_spi* sja1105_get_by_spi(unsigned bus, unsigned chip_select);
struct sja1105_spi* sja1105_get_by_of_node(struct device_node *np);
void sja1105_spi_put(struct sja1105_spi *sja);

int sja1105_get_switch_id(struct sja1105_spi *sja);

int sja1105_spi_set_ptp_state(struct sja1105_spi *sja, bool enabled_ptp);
#endif
