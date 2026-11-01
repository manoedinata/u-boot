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
 * Read-only at boot: the reason is logged and exported as env vars
 * (rst_stat, reset_reason); nothing acts on it yet. "sboot <target>" writes
 * INFORM3 and does a PMU software reset, like "reboot <target>" on stock.
 */

#include <command.h>
#include <dm.h>
#include <env.h>
#include <event.h>
#include <malloc.h>
#include <mapmem.h>
#include <serial_a51_ramlog.h>
#include <video.h>
#include <video_console.h>
#include <vsprintf.h>
#include <asm/io.h>

#define PMU_BASE		0x11860000
#define PMU_SWRESET		(PMU_BASE + 0x0400)
#define PMU_RST_STAT		(PMU_BASE + 0x0404)
#define PMU_INFORM2		(PMU_BASE + 0x0808)
#define PMU_INFORM3		(PMU_BASE + 0x080c)

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

static int a51_decon_trigger(void)
{
	writel(DECON_TRIG_FB_UPDATE, HW_SW_TRIG_CONTROL);

	return 0;
}
EVENT_SPY_SIMPLE(EVT_MISC_INIT_F, a51_decon_trigger);

static int a51_log_reset(void)
{
	u32 rst = readl(PMU_RST_STAT);
	u32 inform2 = readl(PMU_INFORM2);
	u32 inform3 = readl(PMU_INFORM3);
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

	for (i = 0; i < ARRAY_SIZE(sboot_targets); i++) {
		if (strcmp(argv[1], sboot_targets[i].name))
			continue;
		printf("a51: reset into %s (INFORM3=%08x)\n",
		       sboot_targets[i].name, sboot_targets[i].inform3);
		writel(sboot_targets[i].inform3, PMU_INFORM3);
		writel(1, PMU_SWRESET);
		while (1)
			;
	}

	return CMD_RET_USAGE;
}

U_BOOT_CMD(sboot, 2, 0, do_sboot,
	   "reset into an S-BOOT target",
	   "<normal|download|upload|recovery>\n"
	   "    normal = BOOT, recovery = RECOVERY, through PMU INFORM3 + SWRESET");
