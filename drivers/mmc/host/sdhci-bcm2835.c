/*
 * BCM2835 SDHCI
 * Copyright (C) 2012 Stephen Warren
 * Based on U-Boot's MMC driver for the BCM2835 by Oleksandr Tymoshenko & me
 * Portions of the code there were obviously based on the Linux kernel at:
 * git://github.com/raspberrypi/linux.git rpi-3.6.y
 * commit f5b930b "Main bcm2708 linux port" signed-off-by Dom Cobley.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mmc/host.h>
#include "sdhci-pltfm.h"

struct bcm2835_sdhci_host {
	u32 shadow_cmd;
	u32 shadow_blk;
#ifdef CONFIG_MMC_SDHCI_BCM2835_VERIFY_WORKAROUND
	int previous_reg;
#endif
};

#define REG_OFFSET_IN_BITS(reg) ((reg) << 3 & 0x18)

static inline u32 bcm2835_sdhci_readl(struct sdhci_host *host, int reg)
{
	u32 val = readl(host->ioaddr + reg);

	if (reg == SDHCI_CAPABILITIES)
		val |= SDHCI_CAN_VDD_330;

	return val;
}

static u16 bcm2835_sdhci_readw(struct sdhci_host *host, int reg)
{
	u32 val = bcm2835_sdhci_readl(host, (reg & ~3));
	u16 word = val >> REG_OFFSET_IN_BITS(reg) & 0xffff;
	return word;
}

static u8 bcm2835_sdhci_readb(struct sdhci_host *host, int reg)
{
	u32 val = bcm2835_sdhci_readl(host, (reg & ~3));
	u8 byte = val >> REG_OFFSET_IN_BITS(reg) & 0xff;
	return byte;
}

static inline void bcm2835_sdhci_writel(struct sdhci_host *host,
						u32 val, int reg)
{
#ifdef CONFIG_MMC_SDHCI_BCM2835_VERIFY_WORKAROUND
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct bcm2835_sdhci_host *bcm2835_host = pltfm_host->priv;

	if (bcm2835_host->previous_reg == reg) {
		if ((reg != SDHCI_HOST_CONTROL)
			&& (reg != SDHCI_CLOCK_CONTROL)) {
			dev_err(mmc_dev(host->mmc),
			"back-to-back write to 0x%x\n", reg);
		}
	}
	bcm2835_host->previous_reg = reg;
#endif

	writel(val, host->ioaddr + reg);
}

/*
 * The Arasan has a bugette whereby it may lose the content of successive
 * writes to the same register that are within two SD-card clock cycles of
 * each other (a clock domain crossing problem). The data
 * register does not have this problem, which is just as well - otherwise we'd
 * have to nobble the DMA engine too.
 *
 * This wouldn't be a problem with the code except that we can only write the
 * controller with 32-bit writes.  So two different 16-bit registers are
 * written back to back creates the problem.
 *
 * In reality, this only happens when SDHCI_BLOCK_SIZE and SDHCI_BLOCK_COUNT
 * are written followed by SDHCI_TRANSFER_MODE and SDHCI_COMMAND.
 * The BLOCK_SIZE and BLOCK_COUNT are meaningless until a command issued so
 * the work around can be further optimized. We can keep shadow values of
 * BLOCK_SIZE, BLOCK_COUNT, and TRANSFER_MODE until a COMMAND is issued.
 * Then, write the BLOCK_SIZE+BLOCK_COUNT in a single 32-bit write followed
 * by the TRANSFER+COMMAND in another 32-bit write.
 */
static void bcm2835_sdhci_writew(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct bcm2835_sdhci_host *bcm2835_host = pltfm_host->priv;
	u32 word_shift = REG_OFFSET_IN_BITS(reg);
	u32 mask = 0xffff << word_shift;
	u32 oldval, newval;

	if (reg == SDHCI_COMMAND) {
		/* Write the block now as we are issuing a command */
		if (bcm2835_host->shadow_blk != 0) {
			bcm2835_sdhci_writel(host, bcm2835_host->shadow_blk,
				SDHCI_BLOCK_SIZE);
			bcm2835_host->shadow_blk = 0;
		}
		oldval = bcm2835_host->shadow_cmd;
	} else if (reg == SDHCI_BLOCK_SIZE || reg == SDHCI_BLOCK_COUNT) {
		/* Block size and count are stored in shadow reg */
		oldval = bcm2835_host->shadow_blk;
	} else {
		/* Read reg, all other registers are not shadowed */
		oldval = readl(host->ioaddr + (reg & ~3));
	}
	newval = (oldval & ~mask) | (val << word_shift);

	if (reg == SDHCI_TRANSFER_MODE) {
		/* Save the transfer mode until the command is issued */
		bcm2835_host->shadow_cmd = newval;
	} else if (reg == SDHCI_BLOCK_SIZE || reg == SDHCI_BLOCK_COUNT) {
		/* Save the block info until the command is issued */
		bcm2835_host->shadow_blk = newval;
	} else {
		/* Command or other regular 32-bit write */
		bcm2835_sdhci_writel(host, newval, reg & ~3);
	}
}

static void bcm2835_sdhci_writeb(struct sdhci_host *host, u8 val, int reg)
{
	u32 oldval = readl(host->ioaddr + (reg & ~3));
	u32 byte_shift = REG_OFFSET_IN_BITS(reg);
	u32 mask = 0xff << byte_shift;
	u32 newval = (oldval & ~mask) | (val << byte_shift);

	bcm2835_sdhci_writel(host, newval, reg & ~3);
}

static const struct sdhci_ops bcm2835_sdhci_ops = {
	.read_l = bcm2835_sdhci_readl,
	.read_w = bcm2835_sdhci_readw,
	.read_b = bcm2835_sdhci_readb,
	.write_l = bcm2835_sdhci_writel,
	.write_w = bcm2835_sdhci_writew,
	.write_b = bcm2835_sdhci_writeb,
	.set_clock = sdhci_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static const struct sdhci_pltfm_data bcm2835_sdhci_pdata = {
	.quirks = SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12 |
		  SDHCI_QUIRK_BROKEN_CARD_DETECTION |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK,
	.ops = &bcm2835_sdhci_ops,
};

static int bcm2835_sdhci_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct bcm2835_sdhci_host *bcm2835_host;
	struct sdhci_pltfm_host *pltfm_host;
	int ret;

	host = sdhci_pltfm_init(pdev, &bcm2835_sdhci_pdata, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);

	bcm2835_host = devm_kzalloc(&pdev->dev, sizeof(*bcm2835_host),
					GFP_KERNEL);
	if (!bcm2835_host) {
		dev_err(mmc_dev(host->mmc),
			"failed to allocate bcm2835_sdhci\n");
		return -ENOMEM;
	}

	pltfm_host = sdhci_priv(host);
	pltfm_host->priv = bcm2835_host;

	pltfm_host->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pltfm_host->clk)) {
		ret = PTR_ERR(pltfm_host->clk);
		goto err;
	}

	return sdhci_add_host(host);

err:
	sdhci_pltfm_free(pdev);
	return ret;
}

static int bcm2835_sdhci_remove(struct platform_device *pdev)
{
	return sdhci_pltfm_unregister(pdev);
}

static const struct of_device_id bcm2835_sdhci_of_match[] = {
	{ .compatible = "brcm,bcm2835-sdhci" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2835_sdhci_of_match);

static struct platform_driver bcm2835_sdhci_driver = {
	.driver = {
		.name = "sdhci-bcm2835",
		.of_match_table = bcm2835_sdhci_of_match,
		.pm = SDHCI_PLTFM_PMOPS,
	},
	.probe = bcm2835_sdhci_probe,
	.remove = bcm2835_sdhci_remove,
};
module_platform_driver(bcm2835_sdhci_driver);

MODULE_DESCRIPTION("BCM2835 SDHCI driver");
MODULE_AUTHOR("Stephen Warren");
MODULE_LICENSE("GPL v2");
