/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "msm_fb.h"

#include <linux/memory.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include "linux/proc_fs.h"

#include <linux/delay.h>

#include <mach/hardware.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <mach/vreg.h>
#include <mach/board_lge.h>

#define MSM_FB_LCDC_VREG_OP(name, op, level)			\
do { \
	vreg = vreg_get(0, name); \
	vreg_set_level(vreg, level); \
	if (vreg_##op(vreg)) \
		printk(KERN_ERR "%s: %s vreg operation failed \n", \
			(vreg_##op == vreg_enable) ? "vreg_enable" \
				: "vreg_disable", name); \
} while (0)

static char *msm_fb_vreg[] = {
	"gp1",
	"gp2",
};

#define QVGA_WIDTH        240
#define QVGA_HEIGHT       320

static void *DISP_CMD_PORT;
static void *DISP_DATA_PORT;

#define EBI2_WRITE16C(x, y) outpw(x, y)
#define EBI2_WRITE16D(x, y) outpw(x, y)
#define EBI2_READ16(x) inpw(x)

static boolean disp_initialized = FALSE;
struct msm_fb_panel_data tovis_qvga_panel_data;

/* For some reason the contrast set at init time is not good. Need to do
* it again
*/
static boolean display_on = TRUE; // FALSE;

static void tovis_qvga_disp_init(struct platform_device *pdev);
#if 0 /* not defined */
static void tovis_qvga_disp_set_rect(int x, int y, int xres, int yres);

static int tovis_qvga_disp_off(struct platform_device *pdev);
static int tovis_qvga_disp_on(struct platform_device *pdev);
#endif

static unsigned int te_lines = 0xab;
static unsigned int mactl = 0x98;
#ifdef TUNING_INITCODE
module_param(te_lines, uint, 0644);
module_param(mactl, uint, 0644);
#endif

static void tovis_qvga_disp_init(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	if (disp_initialized)
		return;

	mfd = platform_get_drvdata(pdev);

	DISP_CMD_PORT = mfd->cmd_port;
	DISP_DATA_PORT = mfd->data_port;

	disp_initialized = TRUE;
}

static int ebi2_power_save_on = 1;
static void msm_fb_ebi2_power_save(int on)
{
	struct vreg *vreg;
	int flag_on = !!on;

	if (ebi2_power_save_on == flag_on)
		return;

 	ebi2_power_save_on = flag_on;

	if (on) {
//		MSM_FB_LCDC_VREG_OP(msm_fb_vreg[0], enable, 1800);
		MSM_FB_LCDC_VREG_OP(msm_fb_vreg[1], enable, 2800);
	} else {
//		MSM_FB_LCDC_VREG_OP(msm_fb_vreg[0], disable, 0);
		MSM_FB_LCDC_VREG_OP(msm_fb_vreg[1], disable, 0);
	}
}

static int ilitek_qvga_disp_off(struct platform_device *pdev)
{
	if (!disp_initialized)
		tovis_qvga_disp_init(pdev);

	if (display_on) {
		// perform lcd deep sleep instead of power off
		EBI2_WRITE16C(DISP_CMD_PORT, 0x28);
		mdelay(50);
		EBI2_WRITE16C(DISP_CMD_PORT, 0x10); // SPLIN
		mdelay(120);
		msm_fb_ebi2_power_save(0);
		display_on = FALSE;
	}

	return 0;
}

static inline void disp_set_rect(uint16_t csp, uint16_t cep, uint16_t psp, uint16_t pep) {
	EBI2_WRITE16C(DISP_CMD_PORT, 0x2a);
	EBI2_WRITE16D(DISP_DATA_PORT,(csp >> 8));
	EBI2_WRITE16D(DISP_DATA_PORT,csp & 0xFF);
	EBI2_WRITE16D(DISP_DATA_PORT,cep >> 8);
	EBI2_WRITE16D(DISP_DATA_PORT,cep & 0xFF);
	EBI2_WRITE16C(DISP_CMD_PORT, 0x2b);
	EBI2_WRITE16D(DISP_DATA_PORT,psp >> 8);
	EBI2_WRITE16D(DISP_DATA_PORT,psp & 0xFF);
	EBI2_WRITE16D(DISP_DATA_PORT,pep >> 8);
	EBI2_WRITE16D(DISP_DATA_PORT,pep & 0xFF);
}

static void ilitek_qvga_disp_set_rect(int x, int y, int xres, int yres) // xres = width, yres - height
{
	if (!disp_initialized)
		return;

	/* printk(KERN_INFO "%s: entered.\n", __func__); */
	disp_set_rect(x, x+xres-1, y, y+yres-1);
	EBI2_WRITE16C(DISP_CMD_PORT,0x2c); // Write memory start
}

static void do_ilitek_init(struct platform_device *pdev)
{
	EBI2_WRITE16C(DISP_CMD_PORT, 0xc0);
	EBI2_WRITE16D(DISP_DATA_PORT,0x2f); // 1

	EBI2_WRITE16C(DISP_CMD_PORT, 0xc1);
	EBI2_WRITE16D(DISP_DATA_PORT,0x11); // 1

	EBI2_WRITE16C(DISP_CMD_PORT, 0xc5);
	EBI2_WRITE16D(DISP_DATA_PORT,0x38); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x50); // 2

	EBI2_WRITE16C(DISP_CMD_PORT, 0xcb);
	EBI2_WRITE16D(DISP_DATA_PORT,0x39); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x2c); // 2
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 3
	EBI2_WRITE16D(DISP_DATA_PORT,0x34); // 4
	EBI2_WRITE16D(DISP_DATA_PORT,0x02); // 5

	EBI2_WRITE16C(DISP_CMD_PORT, 0xcf);
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0xaa); // 2
	EBI2_WRITE16D(DISP_DATA_PORT,0xb0); // 3

	/* Driver timing control */
	EBI2_WRITE16C(DISP_CMD_PORT, 0xe8);
	EBI2_WRITE16D(DISP_DATA_PORT,0x8a); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x01); // 2
	EBI2_WRITE16D(DISP_DATA_PORT,0x78); // 3

	EBI2_WRITE16C(DISP_CMD_PORT, 0xea);
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 2

	/* Power on sequence control */
	EBI2_WRITE16C(DISP_CMD_PORT, 0xed);
	EBI2_WRITE16D(DISP_DATA_PORT,0x67); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x03); // 2
	EBI2_WRITE16D(DISP_DATA_PORT,0x12); // 3
	EBI2_WRITE16D(DISP_DATA_PORT,0x81); // 4

	/* Pump ratio control */
	EBI2_WRITE16C(DISP_CMD_PORT, 0xf7);
	EBI2_WRITE16D(DISP_DATA_PORT,0x20); // 1

	/* Display mode setting */
	EBI2_WRITE16C(DISP_CMD_PORT, 0x13);

	/* Memory access control */
	EBI2_WRITE16C(DISP_CMD_PORT, 0x36);
	EBI2_WRITE16D(DISP_DATA_PORT,mactl); // 1

	EBI2_WRITE16C(DISP_CMD_PORT, 0x3a);
	EBI2_WRITE16D(DISP_DATA_PORT,0x05); // 1

	EBI2_WRITE16C(DISP_CMD_PORT, 0xb1);
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x17); // 2

	EBI2_WRITE16C(DISP_CMD_PORT, 0xb4);
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 1

	EBI2_WRITE16C(DISP_CMD_PORT, 0xb5);
	EBI2_WRITE16D(DISP_DATA_PORT,0x0a); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x06); // 2
	EBI2_WRITE16D(DISP_DATA_PORT,0x0a); // 3
	EBI2_WRITE16D(DISP_DATA_PORT,0x14); // 4

	EBI2_WRITE16C(DISP_CMD_PORT, 0xb6);
	EBI2_WRITE16D(DISP_DATA_PORT,0x0a); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x02); // 2
	EBI2_WRITE16D(DISP_DATA_PORT,0x27); // 3
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 4

	EBI2_WRITE16C(DISP_CMD_PORT, 0x35);
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 1

	EBI2_WRITE16C(DISP_CMD_PORT, 0x44); // Tearing effect Control
	EBI2_WRITE16D(DISP_DATA_PORT,te_lines>>8); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,te_lines&0xFF); // 2

  /* Positive Gamma Correction */
	EBI2_WRITE16C(DISP_CMD_PORT, 0xe0);
	EBI2_WRITE16D(DISP_DATA_PORT,0x0d); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x32); // 2
	EBI2_WRITE16D(DISP_DATA_PORT,0x2d); // 3
	EBI2_WRITE16D(DISP_DATA_PORT,0x0c); // 4
	EBI2_WRITE16D(DISP_DATA_PORT,0x0e); // 5
	EBI2_WRITE16D(DISP_DATA_PORT,0x05); // 6
	EBI2_WRITE16D(DISP_DATA_PORT,0x4d); // 6
	EBI2_WRITE16D(DISP_DATA_PORT,0x55); // 8
	EBI2_WRITE16D(DISP_DATA_PORT,0x34); // 9
	EBI2_WRITE16D(DISP_DATA_PORT,0x02); // 10
	EBI2_WRITE16D(DISP_DATA_PORT,0x0d); // 11
	EBI2_WRITE16D(DISP_DATA_PORT,0x02); // 12
	EBI2_WRITE16D(DISP_DATA_PORT,0x12); // 13
	EBI2_WRITE16D(DISP_DATA_PORT,0x12); // 14
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 15

  /* Negative Gamma Correction */
	EBI2_WRITE16C(DISP_CMD_PORT, 0xe1);
	EBI2_WRITE16D(DISP_DATA_PORT,0x0a); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x0f); // 2
	EBI2_WRITE16D(DISP_DATA_PORT,0x14); // 3
	EBI2_WRITE16D(DISP_DATA_PORT,0x02); // 4
	EBI2_WRITE16D(DISP_DATA_PORT,0x0f); // 5
	EBI2_WRITE16D(DISP_DATA_PORT,0x05); // 6
	EBI2_WRITE16D(DISP_DATA_PORT,0x34); // 6
	EBI2_WRITE16D(DISP_DATA_PORT,0x44); // 8
	EBI2_WRITE16D(DISP_DATA_PORT,0x4d); // 9
	EBI2_WRITE16D(DISP_DATA_PORT,0x08); // 10
	EBI2_WRITE16D(DISP_DATA_PORT,0x10); // 11
	EBI2_WRITE16D(DISP_DATA_PORT,0x0c); // 12
	EBI2_WRITE16D(DISP_DATA_PORT,0x2f); // 13
	EBI2_WRITE16D(DISP_DATA_PORT,0x2f); // 14
	EBI2_WRITE16D(DISP_DATA_PORT,0x17); // 15

	EBI2_WRITE16C(DISP_CMD_PORT,0x2a); // Set_column_address
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 2
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 3
	EBI2_WRITE16D(DISP_DATA_PORT,0xef); // 4

	EBI2_WRITE16C(DISP_CMD_PORT,0x2b); // Set_Page_address
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x00); // 2
	EBI2_WRITE16D(DISP_DATA_PORT,0x01); // 3
	EBI2_WRITE16D(DISP_DATA_PORT,0x3f); // 4

	EBI2_WRITE16C(DISP_CMD_PORT,0xe8); // Charge Sharing Control
	EBI2_WRITE16D(DISP_DATA_PORT,0x8a); // 1
	EBI2_WRITE16D(DISP_DATA_PORT,0x01); // 2
	EBI2_WRITE16D(DISP_DATA_PORT,0x78); // 3

	EBI2_WRITE16C(DISP_CMD_PORT,0x11); // Exit Sleep

	mdelay(80);

/*-- bootlogo is displayed at oemsbl
	EBI2_WRITE16C(DISP_CMD_PORT,0x2c); // Write memory start
	for(y = 0; y < 320; y++) {
		int pixel = 0x0;
		for(x= 0; x < 240; x++) {
			EBI2_WRITE16D(DISP_DATA_PORT,pixel); // 1
		}
	}

	mdelay(50);
*/
	EBI2_WRITE16C(DISP_CMD_PORT,0x29); // Display On
}

static int ilitek_qvga_disp_on(struct platform_device *pdev)
{
	if (!disp_initialized)
		tovis_qvga_disp_init(pdev);

	if (!display_on) {
		msm_fb_ebi2_power_save(1);
		mdelay(10);
		gpio_set_value(102, 1);
		mdelay(1);
		gpio_set_value(102, 0);
		mdelay(1);
		gpio_set_value(102, 1);
		mdelay(120);
		display_on = TRUE;

		do_ilitek_init(pdev);
	}
	return 0;
}

ssize_t tovis_qvga_show_onoff(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", display_on);
}

ssize_t tovis_qvga_store_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int onoff;
	struct msm_fb_panel_data *pdata = dev_get_platdata(dev);
	struct platform_device *pd = to_platform_device(dev);

	sscanf(buf, "%d", &onoff);

	if (onoff) {
		pdata->on(pd);
	} else {
		pdata->off(pd);
	}

	return count;
}

DEVICE_ATTR(lcd_onoff, 0666, tovis_qvga_show_onoff, tovis_qvga_store_onoff);

static int __init tovis_qvga_probe(struct platform_device *pdev)
{
	int ret;

	if (pdev->id == 0) {
		return 0;
	}

	msm_fb_add_device(pdev);

	ret = device_create_file(&pdev->dev, &dev_attr_lcd_onoff);
	if (ret) {
		printk("tovis_qvga_probe device_creat_file failed!!!\n");
	}

	return 0;
}

struct msm_fb_panel_data tovis_qvga_panel_data = {
	.on = ilitek_qvga_disp_on,
	.off = ilitek_qvga_disp_off,
	.set_backlight = NULL,
	.set_rect = ilitek_qvga_disp_set_rect,
};

static struct platform_device this_device = {
	.name   = "ebi2_tovis_qvga",
	.id	= 1,
	.dev	= {
		.platform_data = &tovis_qvga_panel_data,
	}
};

static struct platform_driver this_driver = {
	.probe  = tovis_qvga_probe,
	.driver = {
		.name   = "ebi2_tovis_qvga",
	},
};

static int __init tovis_qvga_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	ret = platform_driver_register(&this_driver);
	if (!ret) {
		pinfo = &tovis_qvga_panel_data.panel_info;
		pinfo->xres = 240;
		pinfo->yres = 320;
		pinfo->type = EBI2_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->wait_cycle = 0x808000;  // ebi2 write timing reduced by bongkyu.kim

		pinfo->bpp = 16;
		pinfo->fb_num = 2;
		pinfo->lcd.vsync_enable = TRUE;
		pinfo->lcd.refx100 = 6000;
		pinfo->lcd.v_back_porch = 0x06;
		pinfo->lcd.v_front_porch = 0x0a;
		pinfo->lcd.v_pulse_width = 2;
		pinfo->lcd.hw_vsync_mode = TRUE;
		pinfo->lcd.vsync_notifier_period = 0;

		ret = platform_device_register(&this_device);
		if (ret)
			platform_driver_unregister(&this_driver);
	}

	return ret;
}

module_init(tovis_qvga_init);
