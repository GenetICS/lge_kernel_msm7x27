/* linux/arch/arm/mach-msm/lge/devices_lge.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2009 LGE.
 * Author: SungEun Kim <cleaneye.kim@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/setup.h>
#include <asm/mach/mmc.h>
#include <mach/msm_memtypes.h>
#include <asm/mach-types.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/pmic.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <linux/android_pmem.h>
#ifdef CONFIG_USB_FUNCTION
#include <linux/usb/mass_storage_function.h>
#include <linux/usb/android_composite.h>
#endif
#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
#include <asm/setup.h>
#endif
#include <mach/board_lge.h>
#include "../board-msm7627-regulator.h"
#include "../devices.h"
#include "../pm.h"
#include <mach/socinfo.h>
#include <linux/bootmem.h>

/* setting board revision information */
int lge_bd_rev;

static int __init board_revno_setup(char *rev_info)
{
	char *rev_str[] = { "evb", "rev_a", "rev_b", "rev_c", "rev_d", "rev_e", "rev_f",
#if defined(CONFIG_MACH_MSM7X27_PECAN)
		"rev_g",
#endif
		"rev_10","rev_11","rev_12","rev_13",};
	int i;

	lge_bd_rev = LGE_REV_TOT_NUM;
	
	for (i = 0; i < LGE_REV_TOT_NUM; i++) 
		if (!strncmp(rev_info, rev_str[i], 6)) {
			lge_bd_rev = i;
			break;
		}

	printk(KERN_INFO"BOARD: LGE %s\n", rev_str[lge_bd_rev]);

	return 1;
}

__setup("lge.rev=", board_revno_setup);

/* setting whether uart console is enalbed or disabled */
static int uart_console_mode = 0;

int __init lge_get_uart_mode(void)
{
	return uart_console_mode;
}

static int __init lge_uart_mode(char *uart_mode)
{
	if (!strncmp("enable", uart_mode, 5)) {
		printk(KERN_INFO"UART CONSOLE : enable\n");
		uart_console_mode = 1;
	} 
	else 
		printk(KERN_INFO"UART CONSOLE : disable\n");

	return 1;
}

__setup("uart_console=", lge_uart_mode);

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource[] = {
	{
		.name = "ram_console",
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = ARRAY_SIZE(ram_console_resource),
	.resource = ram_console_resource,
};

void __init lge_add_ramconsole_devices(void)
{
	struct resource *res = ram_console_resource;
	struct membank *bank = &meminfo.bank[0];
	res->start = MSM7X27_EBI1_CS0_BASE + bank->size;
	res->end = res->start + LGE_RAM_CONSOLE_SIZE - 1;
	printk("RAM CONSOLE START ADDR : %d\n", res->start);
	printk("RAM CONSOLE END ADDR   : %d\n", res->end);
	
	platform_device_register(&ram_console_device);
}

__WEAK struct lge_panic_handler_platform_data panic_handler_data;

static struct platform_device panic_handler_device = {
	.name = "panic-handler",
	.dev    = {
		.platform_data = &panic_handler_data,
	}
};

void __init lge_add_panic_handler_devices(void)
{
	platform_device_register(&panic_handler_device);
}

static struct platform_device ers_kernel = {
	.name = "ers-kernel",
};

void __init lge_add_ers_devices(void)
{
	platform_device_register(&ers_kernel);
}

#endif

/* setting frame buffer device */
static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;

	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
		if (!strcmp(name, "lcdc_gordon_vga"))
			ret = 0;
		else
			ret = -ENODEV;
	}

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static void *fb_copy_virt;
void *lge_get_fb_copy_virt_addr(void)
{
	return (void *)fb_copy_virt;
}
static size_t fb_copy_phys;
static size_t fb_copy_size;

void *lge_get_fb_copy_phys_addr(void)
{
	return (void *)fb_copy_phys;
}
static void __init lge_make_fb_pmem(void)
{
	struct membank *bank = &meminfo.bank[0];
	unsigned *temp;

	fb_copy_phys = MSM7X27_EBI1_CS0_BASE + bank->size + LGE_RAM_CONSOLE_SIZE;
	/* Modify HIDDEN_RSEET_FB_SIZE defined in board_lge.h */
	fb_copy_size = HIDDEN_RESET_FB_SIZE;

	fb_copy_virt = ioremap(fb_copy_phys, fb_copy_size);

	temp = fb_copy_virt;
	*temp = 0x0;

	printk("FB START PHYS ADDR : %x\n", fb_copy_phys);
	printk("FB START VIRT ADDR : %x\n", (unsigned)fb_copy_virt);
	printk("FB SIZE : %x\n", fb_copy_size);

	return;
}
void __init msm_add_fb_device(void) 
{
	lge_make_fb_pmem();
	platform_device_register(&msm_fb_device);
}

/* kgsl moved to mach-msm/devices-msm7x27.c since .38 */


static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};


static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};


static struct platform_device *pmem_devices[] __initdata = {
	&android_pmem_device,
	&android_pmem_adsp_device,
};

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	pmem_fb_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_fb_size", fb_size_setup);

void __init msm_msm7x2x_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
			size, addr, __pa(addr));
}

static struct memtype_reserve msm7x27_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_mdp_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x27_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	msm7x27_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
}

static void __init msm7x27_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x27_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7x27_reserve_info __initdata = {
	.memtype_reserve_table = msm7x27_reserve_table,
	.calculate_reserve_sizes = msm7x27_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x27_paddr_to_memtype,
};

void __init msm7x27_reserve(void)
{
	reserve_info = &msm7x27_reserve_info;
	msm_reserve();
}

void __init msm7x27_init_early(void)
{
	msm_msm7x2x_allocate_memory_regions();
}

void __init msm_add_pmem_devices(void)
{
	platform_add_devices(pmem_devices, ARRAY_SIZE(pmem_devices));
}

/* setting power management configuration of msm7x25 */
__WEAK struct msm_pm_platform_data msm7x25_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
};

/* setting power management configuration of msm7x27 */
__WEAK struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
	.idle_supported = 1,
	.suspend_supported = 1,
	.suspend_enabled = 1,
	.idle_enabled = 1,
	.latency = 16000,
	.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
	.idle_supported = 1,
	.suspend_supported = 1,
	.suspend_enabled = 1,
	.idle_enabled = 1,
	.latency = 12000,
	.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
	.idle_supported = 1,
	.suspend_supported = 1,
	.suspend_enabled = 1,
	.idle_enabled = 1,
	.latency = 2000,
	.residency = 0,
	}
};

#ifdef CONFIG_USB_G_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

#endif

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	if (on)
		msm_hsusb_vbus_powerup();
	else
		msm_hsusb_vbus_shutdown();
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_65NM),
};
static void __init msm7x2x_init_host(void)
{
	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa())
		return;

	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int msm_hsusb_ldo_init(int init)
{
	static struct regulator *reg_hsusb;
	int rc;
	if (init) {
		reg_hsusb = regulator_get(NULL, "usb");
		if (IS_ERR(reg_hsusb)) {
			rc = PTR_ERR(reg_hsusb);
			pr_err("%s: could not get regulator: %d\n",
					__func__, rc);
			goto out;
		}

		rc = regulator_set_voltage(reg_hsusb, 3300000, 3300000);
		if (rc < 0) {
			pr_err("%s: could not set voltage: %d\n",
					 __func__, rc);
			goto usb_reg_fail;
		}

		rc = regulator_enable(reg_hsusb);
		if (rc < 0) {
			pr_err("%s: could not enable regulator: %d\n",
					__func__, rc);
			goto usb_reg_fail;
		}

		/*
		 * PHY 3.3V analog domain(VDDA33) is powered up by
		 * an always enabled power supply (LP5900TL-3.3).
		 * USB VREG default source is VBUS line. Turning
		 * on USB VREG has a side effect on the USB suspend
		 * current. Hence USB VREG is explicitly turned
		 * off here.
		 */

		rc = regulator_disable(reg_hsusb);
		if (rc < 0) {
			pr_err("%s: could not disable regulator: %d\n",
					__func__, rc);
			goto usb_reg_fail;
		}

		regulator_put(reg_hsusb);
	}

	return 0;
usb_reg_fail:
	regulator_put(reg_hsusb);
out:
	return rc;
}

static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		ret = msm_pm_app_rpc_init(callback);
	} else {
		msm_pm_app_rpc_deinit(callback);
		ret = 0;
	}
	return ret;
}

static int msm_otg_rpc_phy_reset(void __iomem *regs)
{
	return msm_hsusb_phy_reset();
}

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect    	= hsusb_rpc_connect,
#ifdef CONFIG_ARCH_MSM7X27
	/* LGE_CHANGE
	 * To reset USB LDO, use RPC(only msm7x27).
	 * 2011-01-12, hyunhui.park@lge.com
	 */
	.phy_reset			= msm_otg_rpc_phy_reset,
#endif
	.pmic_vbus_notif_init	= msm_hsusb_pmic_notif_init,
	.chg_vbus_draw      = hsusb_chg_vbus_draw,
	.chg_connected      = hsusb_chg_connected,
	.chg_init        	= hsusb_chg_init,
#ifdef CONFIG_USB_EHCI_MSM_72K
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.ldo_init       = msm_hsusb_ldo_init,
	.pclk_required_during_lpm = 1,
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;
#endif
#endif

static struct platform_device *usb_devices[] __initdata = {
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
	/* LGE_CHANGE
	 * Add platform data and device for cdrom storage function.
	 * It will be used in Autorun feature.
	 * 2011-03-02, hyunhui.park@lge.com
	 */
	&usb_cdrom_storage_device,
#endif
#ifdef CONFIG_USB_G_ANDROID
	&android_usb_device,
#endif

};

static void usb_mpp_init(void)
{
	unsigned rc;
	unsigned mpp_usb = 7;

	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
		rc = mpp_config_digital_out(mpp_usb,
				MPP_CFG(MPP_DLOGIC_LVL_VDD,
					MPP_DLOGIC_OUT_CTRL_HIGH));
		if (rc)
			pr_err("%s: configuring mpp pin"
					"to enable 3.3V LDO failed\n", __func__);
	}
}

void __init msm_add_usb_devices(void) 
{
	usb_mpp_init();

#ifdef CONFIG_USB_MSM_OTG_72K
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	if (machine_is_msm7x25_surf() || machine_is_msm7x25_ffa()) {
		msm_otg_pdata.pemp_level =
			PRE_EMPHASIS_WITH_20_PERCENT;
		msm_otg_pdata.drv_ampl = HS_DRV_AMPLITUDE_5_PERCENT;
		msm_otg_pdata.cdr_autoreset = CDR_AUTO_RESET_ENABLE;
		msm_otg_pdata.phy_reset = msm_otg_rpc_phy_reset;
	}
	if (machine_is_msm7x27_surf() || machine_is_msm7x27_ffa()) {
		msm_otg_pdata.pemp_level =
			PRE_EMPHASIS_WITH_10_PERCENT;
		msm_otg_pdata.drv_ampl = HS_DRV_AMPLITUDE_5_PERCENT;
		msm_otg_pdata.cdr_autoreset = CDR_AUTO_RESET_DISABLE;
		msm_otg_pdata.phy_reset_sig_inverted = 1;
	}

#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
		msm7x27_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
	msm_gadget_pdata.is_phy_status_timer_on = 1;
#endif
#endif

	platform_add_devices(usb_devices, ARRAY_SIZE(usb_devices));

#ifdef CONFIG_USB_EHCI_MSM_72K
	msm7x2x_init_host();
#endif
}


/* setting msm i2c device */
static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
		gpio_scl = 95;
		gpio_sda = 96;
	} else {
		gpio_scl = 60;
		gpio_sda = 61;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 400000,
	.rmutex  = 0,
	.pri_clk = 60,
	.pri_dat = 61,
	.aux_clk = 95,
	.aux_dat = 96,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static struct platform_device msm_proccomm_regulator_dev = {
	.name   = PROCCOMM_REGULATOR_DEV_NAME,
	.id     = -1,
	.dev    = {
	  .platform_data = &msm7627_proccomm_regulator_data
	}
};

void __init msm7627_init_regulators(void)
{
	int rc = platform_device_register(&msm_proccomm_regulator_dev);
	if (rc)
	  pr_err("%s: could not register regulator device: %d\n",
	     __func__, rc);
}

void __init msm_device_i2c_init(void)
{
	if (gpio_request(60, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(61, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");
	if (gpio_request(95, "i2c_sec_clk"))
		pr_err("failed to request gpio i2c_sec_clk\n");
	if (gpio_request(96, "i2c_sec_dat"))
		pr_err("failed to request gpio i2c_sec_dat\n");

	msm_i2c_pdata.pm_lat =
		msm7x27_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_B_SYNC      GPIO_CFG(87, 5, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_DATA      GPIO_CFG(86, 3, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(85, 3, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(84, 4, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_B_SYNC, .label =  "tsif_sync", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_clk = "core_clk",
	.tsif_pclk = "iface_clk",
	.tsif_ref_clk = "ref_clk",
};

void __init lge_add_tsif_devices(void)
//TODO: this needs merging into msm7x2x_init in device board files
{
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
	platform_device_register(&msm_device_tsif);
}
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */


/* lge gpio i2c device */
#define MAX_GPIO_I2C_DEV_NUM	10
#define LOWEST_GPIO_I2C_BUS_NUM	2

static gpio_i2c_init_func_t *i2c_init_func[MAX_GPIO_I2C_DEV_NUM] __initdata;
static int i2c_dev_num __initdata = 0;

void __init lge_add_gpio_i2c_device(gpio_i2c_init_func_t *init_func)
{
	i2c_init_func[i2c_dev_num] = init_func;
	i2c_dev_num++;
}

void __init lge_add_gpio_i2c_devices(void)
{
	int index;
	gpio_i2c_init_func_t *init_func_ptr;

	for (index = 0;index < i2c_dev_num;index++) {
		init_func_ptr = i2c_init_func[index];
		(*init_func_ptr)(LOWEST_GPIO_I2C_BUS_NUM + index);
	}
}

int init_gpio_i2c_pin(struct i2c_gpio_platform_data *i2c_adap_pdata,
		struct gpio_i2c_pin gpio_i2c_pin,
		struct i2c_board_info *i2c_board_info_data)
{
	i2c_adap_pdata->sda_pin = gpio_i2c_pin.sda_pin;
	i2c_adap_pdata->scl_pin = gpio_i2c_pin.scl_pin;

	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.sda_pin, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.scl_pin, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(gpio_i2c_pin.sda_pin, 1);
	gpio_set_value(gpio_i2c_pin.scl_pin, 1);

	if (gpio_i2c_pin.reset_pin) {
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.reset_pin, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(gpio_i2c_pin.reset_pin, 1);
	}

	if (gpio_i2c_pin.irq_pin) {
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.irq_pin, 0, GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		i2c_board_info_data->irq =
			MSM_GPIO_TO_INT(gpio_i2c_pin.irq_pin);
	}

	return 0;
}

#ifdef CONFIG_MACH_MSM7X27_PECAN
int init_gpio_i2c_pin_touch(struct i2c_gpio_platform_data *i2c_adap_pdata,
    struct gpio_i2c_pin gpio_i2c_pin,
    struct i2c_board_info *i2c_board_info_data)
{
  i2c_adap_pdata->sda_pin = gpio_i2c_pin.sda_pin;
  i2c_adap_pdata->scl_pin = gpio_i2c_pin.scl_pin;
  {
    gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.sda_pin, 0, GPIO_CFG_OUTPUT,
      GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.scl_pin, 0, GPIO_CFG_OUTPUT,
      GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

  }
  gpio_set_value(gpio_i2c_pin.sda_pin, 1);
  gpio_set_value(gpio_i2c_pin.scl_pin, 1);

  if (gpio_i2c_pin.reset_pin) {
    gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.reset_pin, 0, GPIO_CFG_OUTPUT,
      GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(gpio_i2c_pin.reset_pin, 1);
  }

  if (gpio_i2c_pin.irq_pin) {
    gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.irq_pin, 0, GPIO_CFG_INPUT,
      GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    i2c_board_info_data->irq =
      MSM_GPIO_TO_INT(gpio_i2c_pin.irq_pin);
  }

  return 0;
}
#endif

__WEAK void __init lge_add_camera_devices(void)
{
}

__WEAK void __init lge_add_input_devices(void)
{
}

__WEAK void __init lge_add_lcd_devices(void)
{
}

__WEAK void __init lge_add_btpower_devices(void)
{
}

__WEAK void __init lge_add_mmc_devices(void)
{
}

__WEAK void __init lge_add_misc_devices(void)
{
}

__WEAK void __init lge_add_pm_devices(void)
{
}
