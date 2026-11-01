// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Galaxy A51 (SM-A515F, Exynos 9611): S-BOOT reset reason and boot
 * target, ported from uniLoader's board-a51.c / drivers/samsung/sboot.c.
 *
 * RST_STAT bits per downstream sec_debug_reset_reason.c. S-BOOT turns an
 * INFORM3 of 0x1234567X into its persistent REBOOT_MODE (low nibble), then
 * clears INFORM3; after a watchdog reset INFORM3 holds S-BOOT's own "dinf"
 * magic 0x25b7ce71 and the stored REBOOT_MODE (the last slot) is reused.
 *
 * The DECON trigger is set before relocation so the simple-framebuffer
 * console shows on the panel.
 *
 * The reset registers are read first thing, logged at the end of init and
 * exported as env vars (rst_stat, reset_reason). "sboot <target>" writes
 * INFORM3 and does a PMU software reset, like "reboot <target>" on stock.
 *
 * The power key is dead, so a hang cannot be cleared by hand. Ported from
 * uniLoader (drivers/samsung/exynos-wdt.c, lib/failsafe):
 * - EXYNOS_MOBILE_A51_WDT arms WDT_CPUCL0 as early as u-boot has a hook;
 *   the kernel takes it over (s3c2410_wdt.tmr_atboot=1).
 * - EXYNOS_MOBILE_A51_FAILSAFE, for u-boot started by S-BOOT directly: after
 *   a watchdog reset of this slot, or MAX_BOOTS boots without the kernel's
 *   init clearing the boot counter (scratch + 4), reset into the other slot;
 *   S-BOOT itself just boots the same slot again after a watchdog reset.
 *   Same scratch layout as uniLoader, so mainline init and a uniLoader in
 *   the other slot interoperate.
 */

#include <command.h>
#include <dm.h>
#include <env.h>
#include <event.h>
#include <malloc.h>
#include <mapmem.h>
#include <serial_a51_ramlog.h>
#include <time.h>
#include <video.h>
#include <video_console.h>
#include <vsprintf.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <linux/delay.h>

#define PMU_BASE		0x11860000
#define PMU_SWRESET		(PMU_BASE + 0x0400)
#define PMU_RST_STAT		(PMU_BASE + 0x0404)
#define PMU_WDT_DISABLE		(PMU_BASE + 0x0408)
#define PMU_WDT_MASK_RESET	(PMU_BASE + 0x040c)
#define PMU_INFORM2		(PMU_BASE + 0x0808)
#define PMU_INFORM3		(PMU_BASE + 0x080c)

/* WDT_CPUCL0: prescaler 256, divider 128 on the 26 MHz OSCCLK, 16-bit */
#define WDT_BASE		0x10050000
#define WTCON			(WDT_BASE + 0x00)
#define WTDAT			(WDT_BASE + 0x04)
#define WTCNT			(WDT_BASE + 0x08)
#define WTCON_PRESCALE(x)	((x) << 8)
#define WTCON_DIV128		(3 << 3)
#define WTCON_ENABLE		BIT(5)
#define WTCON_RSTEN		BIT(0)
#define WDT_HZ			(26000000 / 256 / 128)
#define WDT_PMU_BIT		BIT(23)	/* S-BOOT leaves it set in both regs */

#define FAILSAFE_MAGIC		0x5afeb007
#define FAILSAFE_SLOT_MARK	0x5afe5100	/* | slot */
#define SBOOT_NORMAL		0x12345670
#define SBOOT_RECOVERY		0x12345674

#define RST_STAT_CL1_WDT	BIT(23)
#define RST_STAT_CL0_WDT	BIT(24)
#define RST_STAT_SWRESET	BIT(29)
#define RST_STAT_PINRESET	BIT(16)

#define SBOOT_DINF_MAGIC	0x25b7ce71

/*
 * S-BOOT leaves DECON in a trigger mode that does not refresh the panel
 * from the framebuffer @0xca000000; 0x1281 (from uniLoader) makes writes
 * to it show up. Mainline simpledrm relies on this too.
 */
#define DECON_F_BASE		0x148b0000
#define HW_SW_TRIG_CONTROL	(DECON_F_BASE + 0x70)
#define DECON_TRIG_FB_UPDATE	0x1281

static const struct {
	const char *name;
	u32 inform3;
} sboot_targets[] = {
	{ "normal",	0x12345670 },
	{ "download",	0x12345671 },
	{ "upload",	0x12345672 },
	{ "recovery",	0x12345674 },
};

static const char *a51_reset_reason(u32 rst)
{
	if (rst & (RST_STAT_CL0_WDT | RST_STAT_CL1_WDT))
		return "wdt";
	if (rst & RST_STAT_SWRESET)
		return "swreset";
	if (rst & RST_STAT_PINRESET)
		return "pinreset";
	return "other";
}

/* Written before relocation: keep them out of .bss, which is not set up yet */
static u32 boot_rst_stat __section(".data");
static u32 boot_inform2 __section(".data");
static u32 boot_inform3 __section(".data");
static u32 wdt_sec __section(".data");

static void a51_sboot_reset(const char *name, u32 inform3)
{
	printf("a51: reset into %s (INFORM3=%08x)\n", name, inform3);
	writel(inform3, PMU_INFORM3);
	writel(1, PMU_SWRESET);
	while (1)
		;
}

static unsigned int a51_wdt_arm(unsigned int sec)
{
	unsigned long count = sec ? sec * WDT_HZ : 0xffff;

	if (count > 0xffff)
		count = 0xffff;

	writel(0, WTCON);
	clrbits_le32(PMU_WDT_DISABLE, WDT_PMU_BIT);
	clrbits_le32(PMU_WDT_MASK_RESET, WDT_PMU_BIT);
	writel(count, WTDAT);
	writel(count, WTCNT);
	writel(WTCON_PRESCALE(255) | WTCON_DIV128 | WTCON_ENABLE | WTCON_RSTEN,
	       WTCON);

	return count / WDT_HZ;
}

/*
 * Earliest board hook in board_init_f (before arch_cpu_init, DM, timer and
 * console); EVT_FSP_INIT_F fires on every architecture.
 */
static int a51_early(void)
{
	boot_rst_stat = readl(PMU_RST_STAT);
	boot_inform2 = readl(PMU_INFORM2);
	boot_inform3 = readl(PMU_INFORM3);
	if (IS_ENABLED(CONFIG_EXYNOS_MOBILE_A51_WDT))
		wdt_sec = a51_wdt_arm(CONFIG_EXYNOS_MOBILE_A51_WDT_SEC);

	return 0;
}
EVENT_SPY_SIMPLE(EVT_FSP_INIT_F, a51_early);

#ifdef CONFIG_EXYNOS_MOBILE_A51_FAILSAFE
#define SELF		CONFIG_EXYNOS_MOBILE_A51_FAILSAFE_SLOT
#define OTHER		(1 - SELF)
#define OTHER_NAME	(OTHER == 0 ? "normal" : "recovery")
#define OTHER_INFORM3	(OTHER == 0 ? SBOOT_NORMAL : SBOOT_RECOVERY)

/* 0: magic, 1: boot counter (cleared by the kernel's init), 2: slot mark */
static void scratch_set(int i, u32 val)
{
	ulong base = CONFIG_EXYNOS_MOBILE_A51_FAILSAFE_SCRATCH;

	writel(val, base + 4 * i);
	flush_dcache_range(base, base + ARCH_DMA_MINALIGN);
}

static u32 scratch_get(int i)
{
	return readl((ulong)CONFIG_EXYNOS_MOBILE_A51_FAILSAFE_SCRATCH + 4 * i);
}

static void a51_escape(const char *why)
{
	printf("failsafe: %s, reset into slot %d in 3 s\n", why, OTHER);
	mdelay(3000);
	a51_sboot_reset(OTHER_NAME, OTHER_INFORM3);
}

static void a51_failsafe(void)
{
	u32 count;

	if (boot_rst_stat & (RST_STAT_CL0_WDT | RST_STAT_CL1_WDT)) {
		scratch_set(1, 0);
		if (scratch_get(0) == FAILSAFE_MAGIC &&
		    scratch_get(2) == (FAILSAFE_SLOT_MARK | OTHER))
			printf("failsafe: watchdog reset of slot %d, booting this one\n",
			       OTHER);
		else
			a51_escape("last reset was the watchdog");
	}

	count = scratch_get(0) == FAILSAFE_MAGIC ? scratch_get(1) + 1 : 1;
	scratch_set(0, FAILSAFE_MAGIC);
	scratch_set(1, count);
	printf("failsafe: slot %d, boot counter %u (escape slot %d)\n",
	       SELF, count, OTHER);
	if (count >= 2) {
		printf("failsafe: boot %u without reaching init\n", count);
		if (count >= CONFIG_EXYNOS_MOBILE_A51_FAILSAFE_MAX_BOOTS) {
			scratch_set(1, 0);
			a51_escape("boot loop");
		}
	}
	scratch_set(2, FAILSAFE_SLOT_MARK | SELF);

	/* S-BOOT honours INFORM3 after anything but a watchdog reset: a panic
	 * that resets lands in the other slot */
	writel(OTHER_INFORM3, PMU_INFORM3);
}
#else
static void a51_failsafe(void) {}
#endif

static void a51_wdt_test(void)
{
	ulong start = get_timer(0);

	printf("a51: watchdog test, waiting for the reset (%u s)\n", wdt_sec);
	while (get_timer(start) < 3 * wdt_sec * 1000)
		;
	printf("a51: watchdog test: no reset after %u s\n", 3 * wdt_sec);
#ifdef CONFIG_EXYNOS_MOBILE_A51_FAILSAFE
	a51_escape("no watchdog reset");
#endif
	while (1)
		;
}

/* Console (ramlog) and timer are up here, still before relocation */
static int a51_misc_init_f(void)
{
	writel(DECON_TRIG_FB_UPDATE, HW_SW_TRIG_CONTROL);
	if (IS_ENABLED(CONFIG_EXYNOS_MOBILE_A51_WDT))
		printf("a51: watchdog armed, %u s\n", wdt_sec);
	a51_failsafe();
	if (IS_ENABLED(CONFIG_EXYNOS_MOBILE_A51_WDT_TEST))
		a51_wdt_test();

	return 0;
}
EVENT_SPY_SIMPLE(EVT_MISC_INIT_F, a51_misc_init_f);

static int a51_log_reset(void)
{
	u32 rst = boot_rst_stat;
	u32 inform2 = boot_inform2;
	u32 inform3 = boot_inform3;
	const char *reason = a51_reset_reason(rst);

	printf("a51: RST_STAT=%08x (%s) INFORM2=%08x INFORM3=%08x%s\n",
	       rst, reason, inform2, inform3,
	       inform3 == SBOOT_DINF_MAGIC ? " (S-BOOT dinf)" : "");
	env_set_hex("rst_stat", rst);
	env_set("reset_reason", reason);

	return 0;
}

/*
 * Drop ANSI escapes (the CLI's terminal-size probe) and stray controls, and
 * apply \r and \b like a terminal would: "Loading Environment from ...\r"
 * is overwritten by its "... OK" line, the autoboot countdown backspaces.
 */
static void a51_strip_escapes(char *s)
{
	char *out = s, *line = s;

	while (*s) {
		if (*s == '\r' && s[1] != '\n') {
			out = line;
			s++;
			continue;
		}
		if (*s == '\b') {
			if (out > line)
				out--;
			s++;
			continue;
		}
		if (*s == '\e') {
			s++;
			if (*s == '[') {
				s++;
				while (*s && (*s < 0x40 || *s > 0x7e))
					s++;
			}
			if (*s)
				s++;
			continue;
		}
		if (*s >= 0x20 || *s == '\n' || *s == '\t') {
			*out++ = *s;
			if (*s == '\n')
				line = out;
		}
		s++;
	}
	*out = '\0';
}

/*
 * The video console only receives output once console_init_r() switches
 * stdout to it, after relocation; the banner, model and DRAM lines went to
 * the ramlog alone. Repaint the panel with everything u-boot logged so far.
 */
static void a51_replay_on_video(void)
{
	struct udevice *con, *vid;
	char *log;

	if (!IS_ENABLED(CONFIG_A51_RAMLOG_SERIAL) || !IS_ENABLED(CONFIG_VIDEO))
		return;
	if (uclass_first_device_err(UCLASS_VIDEO_CONSOLE, &con))
		return;
	log = a51_ramlog_since_boot();
	if (!log)
		return;

	a51_strip_escapes(log);
	vid = dev_get_parent(con);
	video_clear(vid);
	if (CONFIG_IS_ENABLED(VIDEO_LOGO))
		video_bmp_display(vid, map_to_sysmem(video_get_u_boot_logo()),
				  -4, 4, true);
	vidconsole_position_cursor(con, 0, 0);
	vidconsole_put_string(con, log);
	video_sync(vid, true);
	free(log);
}

static int a51_last_stage(void)
{
	a51_log_reset();
	a51_replay_on_video();

	return 0;
}
EVENT_SPY_SIMPLE(EVT_LAST_STAGE_INIT, a51_last_stage);

static int do_sboot(struct cmd_tbl *cmdtp, int flag, int argc,
		    char *const argv[])
{
	int i;

	if (argc != 2)
		return CMD_RET_USAGE;

	for (i = 0; i < ARRAY_SIZE(sboot_targets); i++)
		if (!strcmp(argv[1], sboot_targets[i].name))
			a51_sboot_reset(sboot_targets[i].name,
					sboot_targets[i].inform3);

	return CMD_RET_USAGE;
}

U_BOOT_CMD(sboot, 2, 0, do_sboot,
	   "reset into an S-BOOT target",
	   "<normal|download|upload|recovery>\n"
	   "    normal = BOOT, recovery = RECOVERY, through PMU INFORM3 + SWRESET");
