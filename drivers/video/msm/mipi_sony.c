/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
#include "mipi_dsi.h"
#include "mipi_sony.h"
#include <mach/gpio.h>
#include <asm/irq.h>
#include <asm/system.h>

#define GPIO_HIGH_VALUE 1
#define GPIO_LOW_VALUE  0

#define NOP()	do {asm volatile ("NOP");} while(0);
#define DELAY_3NS() do { \
    asm volatile ("NOP"); \
    asm volatile ("NOP"); \
    asm volatile ("NOP");} while(0);


#define FEATURE_SKY_BACKLIGHT_AAT1403  


#ifdef FEATURE_SKY_BACKLIGHT_AAT1403
#define LCD_BL_EN      21
#define BL_MAX         32
#endif

#define LCD_RESET      50
#define LCD_POWER      49
#define LCD_DEBUG_MSG

#ifdef LCD_DEBUG_MSG
#define ENTER_FUNC()        printk(KERN_INFO "[SKY_LCD] +%s \n", __FUNCTION__);
#define EXIT_FUNC()         printk(KERN_INFO "[SKY_LCD] -%s \n", __FUNCTION__);
#define ENTER_FUNC2()       printk(KERN_ERR "[SKY_LCD] +%s\n", __FUNCTION__);
#define EXIT_FUNC2()        printk(KERN_ERR "[SKY_LCD] -%s\n", __FUNCTION__);
#define PRINT(fmt, args...) printk(KERN_INFO fmt, ##args)
#define DEBUG_EN 1
#else
#define PRINT(fmt, args...)
#define ENTER_FUNC2()
#define EXIT_FUNC2()
#define ENTER_FUNC()
#define EXIT_FUNC()
#define DEBUG_EN 0
#endif

static struct msm_panel_common_pdata *mipi_sony_pdata;

static struct dsi_buf sony_tx_buf;
static struct dsi_buf sony_rx_buf;

static uint32_t lcd_gpio_init_table[] = {
	GPIO_CFG(LCD_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(LCD_POWER, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#ifdef 	FEATURE_SKY_BACKLIGHT_AAT1403
	GPIO_CFG(LCD_BL_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif
};

struct lcd_state_type {
    boolean disp_powered_up;
    boolean disp_initialized;
    boolean disp_on;
};

static struct lcd_state_type sony_state = { 0, };

static void lcd_gpio_init(uint32_t *table, int len, unsigned enable)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n],
				enable ? GPIO_CFG_ENABLE : GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, table[n], rc);
			break;
		}
	}
}


char sleep_out[2]   = {0x11, 0x00};
char display_ctl[2]  = {0x36, 0x80};
char disp_on[2]     = {0x29, 0x00};
char sleep_in[2]    = {0x10, 0x00};
char disp_off[2]    = {0x28, 0x00};

static struct dsi_cmd_desc sony_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(disp_off), disp_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(sleep_in), sleep_in}
};

static struct dsi_cmd_desc sony_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 140, sizeof(sleep_out), sleep_out},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(display_ctl), display_ctl},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(disp_on), disp_on}
};

/*
 0. sony_display_init_cmds
 1. sony_display_veil_init0_cmds
 2. sony_display_veil_lut_cmds
 3. sony_display_veil_init1_cmds
 4. sony_display_veil_tex_cmds
 5. sony_display_veil_colormap_cmds
 6. sony_display_veil_init2_cmds
 7. dsi_cmd_desc sony_display_on_cmds
 */

static int mipi_sony_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

    ENTER_FUNC2();

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;	
	
	//mutex_lock(&mfd->dma->ov_mutex);
	if (sony_state.disp_initialized == false) {
		//PRINT("[LIVED] LCD RESET!!\n");
		gpio_set_value(LCD_RESET, GPIO_LOW_VALUE);
		usleep(10);//msleep(1);
		gpio_set_value(LCD_RESET, GPIO_HIGH_VALUE);
		usleep(10);//msleep(120);
		sony_state.disp_initialized = true;
	}
	
	mipi_dsi_cmds_tx(&sony_tx_buf, sony_display_on_cmds,
			ARRAY_SIZE(sony_display_on_cmds));
	sony_state.disp_on = true;
	//mutex_unlock(&mfd->dma->ov_mutex);

	EXIT_FUNC2();
	return 0;
}

static int mipi_sony_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

    ENTER_FUNC2();

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

    //mutex_lock(&mfd->dma->ov_mutex); 
	if (sony_state.disp_on == true) {
		gpio_set_value(LCD_RESET, GPIO_LOW_VALUE);
		usleep(10);//msleep(1);
		gpio_set_value(LCD_RESET, GPIO_HIGH_VALUE);
		usleep(10);//msleep(120);

		mipi_dsi_cmds_tx(&sony_tx_buf, sony_display_off_cmds,
				ARRAY_SIZE(sony_display_off_cmds));
		sony_state.disp_on = false;
		sony_state.disp_initialized = false;
	}
    //mutex_unlock(&mfd->dma->ov_mutex);    
    EXIT_FUNC2();
	return 0;
}

static void mipi_sony_set_backlight(struct msm_fb_data_type *mfd)
{
	//static int first_enable = 0;
	static int prev_bl_level = 0;
	int cnt, bl_level;
	//int count = 0;
	unsigned long flags;
	bl_level = mfd->bl_level;

	if (bl_level == prev_bl_level || sony_state.disp_on == 0) {
		PRINT("[LIVED] same! or not disp_on\n");
	} else {
		if (bl_level == 0) {
			gpio_set_value(LCD_BL_EN ,GPIO_LOW_VALUE);
			usleep(500);
		} else {
			cnt = BL_MAX - bl_level;

			//PRINT("[LIVED] prev_bl_level =%d, bl_level =%d, cnt=%d\n", prev_bl_level, bl_level, cnt);
			do {
				local_save_flags(flags);
				local_irq_disable();
				gpio_set_value(LCD_BL_EN ,GPIO_LOW_VALUE);
				udelay(1);	// T LO
				//count++;
				gpio_set_value(LCD_BL_EN ,GPIO_HIGH_VALUE);
				udelay(1);	// T HI
				local_irq_restore(flags);
			} while (cnt--);
			udelay(500);      // latch

			//PRINT("[LIVED] count=%d\n", count);
		}
		prev_bl_level = bl_level;
	}
}

static int __devinit mipi_sony_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_sony_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_sony_lcd_probe,
	.driver = {
		.name   = "mipi_sony",
	},
};

static struct msm_fb_panel_data sony_panel_data = {
	.on             = mipi_sony_lcd_on,
	.off            = mipi_sony_lcd_off,
	.set_backlight  = mipi_sony_set_backlight,
};

static int ch_used[3];

int mipi_sony_device_register(struct msm_panel_info *pinfo,
		u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_sony", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	sony_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &sony_panel_data,
		sizeof(sony_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_sony_lcd_init(void)
{
    ENTER_FUNC2();

    lcd_gpio_init(lcd_gpio_init_table, ARRAY_SIZE(lcd_gpio_init_table), 1);

    sony_state.disp_powered_up = true;

    mipi_dsi_buf_alloc(&sony_tx_buf, DSI_BUF_SIZE);
    mipi_dsi_buf_alloc(&sony_rx_buf, DSI_BUF_SIZE);

    EXIT_FUNC2();

    return platform_driver_register(&this_driver);
}

module_init(mipi_sony_lcd_init);
