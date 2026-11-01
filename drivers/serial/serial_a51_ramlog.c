// SPDX-License-Identifier: GPL-2.0
/*
 * Write-only "console" that appends to the ramoops console-zone ring buffer
 * (Linux persistent_ram_buffer: "DBGC" signature, start, size, then the
 * ring; no ECC) uniLoader's own lib/ramlog writes to before jumping here, so
 * u-boot's log chains onto the loader's and is readable as
 * /sys/fs/pstore/console-ramoops-0 from whichever kernel boots next.
 *
 * The header is left as uniLoader wrote it: a valid signature at probe means
 * continue the same ring, matching lib/ramlog/ramlog.c's own self-healing
 * check so this behaves the same whether or not uniLoader ran first.
 *
 * MMU/dcache are on here (unlike uniLoader), and the zone must survive an
 * abrupt watchdog reset with no clean shutdown, so every write is flushed
 * out of the cache immediately.
 */

#include <dm.h>
#include <malloc.h>
#include <serial.h>
#include <serial_a51_ramlog.h>
#include <linux/compiler.h>
#include <asm/cache.h>
#include <asm/io.h>

#define RAMOOPS_SIG	0x43474244u	/* "DBGC" */
#define RAMLOG_HDR	12
#define BOOT_START_UNSET	0xffffffffu

/*
 * Ring position when u-boot first probed the console (before relocation),
 * i.e. where this u-boot's own output starts. Initialised so it lives in
 * .data, which relocation copies with its current value.
 */
static u32 boot_start = BOOT_START_UNSET;

struct a51_ramlog_priv {
	void __iomem *hdr;	/* u32[3]: sig, start, size */
	void __iomem *data;
	u32 cap;		/* ring capacity, i.e. (region size - RAMLOG_HDR) */
};

static void a51_ramlog_putc_raw(struct a51_ramlog_priv *priv, char c)
{
	u32 start = readl(priv->hdr + 4);
	u32 size = readl(priv->hdr + 8);

	writeb(c, priv->data + start);
	flush_dcache_range((unsigned long)(priv->data + start),
			    (unsigned long)(priv->data + start) + ARCH_DMA_MINALIGN);

	start = (start + 1) % priv->cap;
	if (size < priv->cap)
		size++;
	writel(start, priv->hdr + 4);
	writel(size, priv->hdr + 8);
	flush_dcache_range((unsigned long)priv->hdr,
			    (unsigned long)priv->hdr + RAMLOG_HDR);
}

static int a51_ramlog_putc(struct udevice *dev, const char ch)
{
	a51_ramlog_putc_raw(dev_get_priv(dev), ch);

	return 0;
}

static ssize_t a51_ramlog_puts(struct udevice *dev, const char *s, size_t len)
{
	struct a51_ramlog_priv *priv = dev_get_priv(dev);
	size_t i;

	for (i = 0; i < len; i++)
		a51_ramlog_putc_raw(priv, s[i]);

	return len;
}

static int a51_ramlog_getc(struct udevice *dev)
{
	return -EAGAIN;
}

static int a51_ramlog_pending(struct udevice *dev, bool input)
{
	return 0;
}

static int a51_ramlog_probe(struct udevice *dev)
{
	struct a51_ramlog_priv *priv = dev_get_priv(dev);
	fdt_size_t size;
	fdt_addr_t addr = dev_read_addr_size(dev, &size);

	if (addr == FDT_ADDR_T_NONE || size <= RAMLOG_HDR)
		return -EINVAL;

	priv->hdr = (void __iomem *)(uintptr_t)addr;
	priv->data = priv->hdr + RAMLOG_HDR;
	priv->cap = (u32)size - RAMLOG_HDR;

	if (readl(priv->hdr) != RAMOOPS_SIG ||
	    readl(priv->hdr + 4) >= priv->cap ||
	    readl(priv->hdr + 8) > priv->cap) {
		writel(RAMOOPS_SIG, priv->hdr);
		writel(0, priv->hdr + 4);
		writel(0, priv->hdr + 8);
		flush_dcache_range((unsigned long)priv->hdr,
				    (unsigned long)priv->hdr + RAMLOG_HDR);
	}
	if (boot_start == BOOT_START_UNSET)
		boot_start = readl(priv->hdr + 4);

	return 0;
}

char *a51_ramlog_since_boot(void)
{
	struct a51_ramlog_priv *priv;
	struct udevice *dev;
	u32 start, len, i;
	char *buf;

	if (boot_start == BOOT_START_UNSET ||
	    uclass_get_device_by_driver(UCLASS_SERIAL,
					DM_DRIVER_GET(serial_a51_ramlog), &dev))
		return NULL;

	priv = dev_get_priv(dev);
	start = readl(priv->hdr + 4);
	len = (start + priv->cap - boot_start) % priv->cap;
	buf = malloc(len + 1);
	if (!buf)
		return NULL;

	for (i = 0; i < len; i++)
		buf[i] = readb(priv->data + (boot_start + i) % priv->cap);
	buf[len] = '\0';

	return buf;
}

static const struct dm_serial_ops a51_ramlog_ops = {
	.putc = a51_ramlog_putc,
	.puts = a51_ramlog_puts,
	.getc = a51_ramlog_getc,
	.pending = a51_ramlog_pending,
};

static const struct udevice_id a51_ramlog_ids[] = {
	{ .compatible = "samsung,a51-ramlog-console" },
	{ }
};

U_BOOT_DRIVER(serial_a51_ramlog) = {
	.name	= "serial_a51_ramlog",
	.id	= UCLASS_SERIAL,
	.of_match = a51_ramlog_ids,
	.probe	= a51_ramlog_probe,
	.ops	= &a51_ramlog_ops,
	.priv_auto = sizeof(struct a51_ramlog_priv),
	.flags	= DM_FLAG_PRE_RELOC,
};
