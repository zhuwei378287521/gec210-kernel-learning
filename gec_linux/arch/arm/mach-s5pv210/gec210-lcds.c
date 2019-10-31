/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/gpio.h>

#include <plat/fb.h>
#include <plat/ctouch.h>
#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <generated/autoconf.h>
#include <../../../drivers/video/samsung/s3cfb.h>


/* s3cfb configs for supported LCD */

static struct s3cfb_lcd xvga = {
#if 0
	.width = 1440,
	.height = 900,

	.bpp = 32,
	.freq = 60,

	.timing = {
		.h_fp	= 50,
		.h_bp	= 50,
		.h_sw	= 250,
		.v_fp	= 3,
		.v_fpe	= 3,
		.v_bp	= 29,
		.v_bpe	= 29,
		.v_sw	= 6,
	},
	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},

#endif


#if 1
	.width = 1024,
	.height = 768,
	.bpp = 32,
	.freq = 60,

	.timing = {
		.h_fp	= 24,
		.h_bp	= 100,
		.h_sw	= 136,
		.v_fp	= 3,
		.v_fpe	= 3,
		.v_bp	= 29,
		.v_bpe	= 29,
		.v_sw	= 6,
	},
	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
#endif


#if 0
	.width = 1024,
	.height = 768,
	.p_width = 309,
	.p_height = 230,
	.bpp = 32,
	.freq = 65,

	.timing = {
		.h_fp	= 10,
		.h_bp	= 10,
		.h_sw	= 200,
		.v_fp	= 10,
		.v_fpe	= 1,
		.v_bp	= 20,
		.v_bpe	= 1,
		.v_sw	= 12,
	},
	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
#endif
};

static struct s3cfb_lcd wvga_s70 = {

	.width = 800,
	.height = 480,
	.p_width = 154,
	.p_height = 96,
	.bpp = 16,
	.freq = 65,

	.timing = {
		.h_fp = 80,
		.h_bp = 36,
		.h_sw = 10,
		.v_fp = 22,
		.v_fpe = 1,
		.v_bp = 15,
		.v_bpe = 1,
		.v_sw = 8,
	},
	.polarity = {
		.rise_vclk = 0,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

static struct s3cfb_lcd wvga_h43 = {
	.width = 480,
	.height = 272,
	.p_width = 96,
	.p_height = 54,
	.bpp = 32,
	.freq = 65,

	.timing = {
		.h_fp =  5,
		.h_bp = 40,
		.h_sw =  2,
		.v_fp =  8,
		.v_fpe = 1,
		.v_bp =  8,
		.v_bpe = 1,
		.v_sw =  2,
	},
	.polarity = {
		.rise_vclk = 0,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};


/* VGAs */
static struct s3cfb_lcd vga = {

#if 1
	.width= 800,
	.height = 600,
	.p_width = 112,
	.p_height = 84,
	.bpp = 32,
	.freq = 65,

	.timing = {
		.h_fp = 16,
		.h_bp = 100,
		.h_sw = 110,
		.v_fp = 32,
		.v_fpe = 1,
		.v_bp = 11,
		.v_bpe = 1,
		.v_sw = 2,
	},
	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
#endif

#if 0
	.width = 800,
	.height = 600,
	.bpp = 32,
	.freq = 60,

	.timing = {
		.h_fp	= 50,
		.h_bp	= 10,
		.v_fp = 10,
		.v_bp = 20,

		.h_sw = 70,
		.v_fpe = 1,
		.v_bpe = 1,
		.v_sw = 12,
	},
	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
#endif
};


/* HDMI */
static struct s3cfb_lcd hdmi_def = {
	.width = 1920,
	.height = 1080,
	.p_width = 480,
	.p_height = 320,
	.bpp = 32,
	.freq = 62,

	.timing = {
		.h_fp = 12,
		.h_bp = 12,
		.h_sw = 4,
		.v_fp = 8,
		.v_fpe = 1,
		.v_bp = 8,
		.v_bpe = 1,
		.v_sw =  4,
	},
	.polarity = {
		.rise_vclk = 0,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

static struct hdmi_config {
	char *name;
	int width;
	int height;
} gec210_hdmi_config[] = {
	{ "HDMI1080P60",	1920, 1080 },
	{ "HDMI1080I60",	1920, 1080 },
	{ "HDMI1080P30",	1920, 1080 },

	{ "HDMI1080P60D",	 960,  536 },
	{ "HDMI1080I60D",	 960,  536 },
	{ "HDMI1080P30D",	 960,  536 },

	{ "HDMI720P60",		1280,  720 },
	{ "HDMI720P60D",	 640,  360 },

	{ "HDMI576P16X9",	 720,  576 },
	{ "HDMI576P16X9D",	 720,  576 },
	{ "HDMI576P4X3",	 720,  576 },
	{ "HDMI576P4X3D",	 720,  576 },

	{ "HDMI480P16X9",	 720,  480 },
	{ "HDMI480P16X9D",	 720,  480 },
	{ "HDMI480P4X3",	 720,  480 },
	{ "HDMI480P4X3D",	 720,  480 },
};


/* Try to guess LCD panel by kernel command line, or
 * using *W50* as default */

static struct {
	char *name;
	struct s3cfb_lcd *lcd;
	int ctp;
} gec210_lcd_config[] = {
	{ "XVGA",  &xvga,  1 },
	{ "S70",  &wvga_s70,  1 },
	{ "H43",  &wvga_h43,  1 },
	{ "VGA",  &vga,  1 },
	{ "HDM",  &hdmi_def,  0 },	/* Pls keep it at last */
};

static int lcd_idx;

static int __init gec210_setup_lcd(char *str)
{
	int i;

	if (!strncasecmp("HDMI", str, 4)) {
		struct hdmi_config *cfg = &gec210_hdmi_config[0];
		struct s3cfb_lcd *lcd;
		lcd_idx = ARRAY_SIZE(gec210_lcd_config) - 1;
		lcd = gec210_lcd_config[lcd_idx].lcd;
		lcd->args = lcd_idx;

		for (i = 0; i < ARRAY_SIZE(gec210_hdmi_config); i++, cfg++) {
			if (!strcasecmp(cfg->name, str)) {
				lcd->width = cfg->width;
				lcd->height = cfg->height;
				goto __ret;
			}
		}
	}

#ifdef CONFIG_LOGO_LINUX_GEC480272
			lcd_idx = 2;
			gec210_lcd_config[lcd_idx].lcd->args = lcd_idx;
			printk("========480x272=========\n");
#endif

#ifdef CONFIG_LOGO_LINUX_GEC800480
			lcd_idx = 1;
			gec210_lcd_config[lcd_idx].lcd->args = lcd_idx;
			printk("========800x480=========\n");
#endif

#ifdef CONFIG_LOGO_LINUX_GEC800600
			lcd_idx = 3;
			gec210_lcd_config[lcd_idx].lcd->args = lcd_idx;
			printk("========800x600=========\n");
#endif

#ifdef CONFIG_LOGO_LINUX_GEC1024768
			lcd_idx = 0;
			gec210_lcd_config[lcd_idx].lcd->args = lcd_idx;
			printk("========1024x768=========\n");
#endif
__ret:
	printk("==========GEC210: %s selected===========\n", gec210_lcd_config[lcd_idx].name);
	return 0;
}
early_param("lcd", gec210_setup_lcd);


struct s3cfb_lcd *gec210_get_lcd(void)
{

#ifdef CONFIG_LOGO_LINUX_GEC480272
			lcd_idx = 2;
			gec210_lcd_config[lcd_idx].lcd->args = lcd_idx;
#endif

#ifdef CONFIG_LOGO_LINUX_GEC800480
			lcd_idx = 1;
			gec210_lcd_config[lcd_idx].lcd->args = lcd_idx;
#endif

#ifdef CONFIG_LOGO_LINUX_GEC800600
			lcd_idx = 3;
			gec210_lcd_config[lcd_idx].lcd->args = lcd_idx;
#endif

#ifdef CONFIG_LOGO_LINUX_GEC1024768
			lcd_idx = 0;
			gec210_lcd_config[lcd_idx].lcd->args = lcd_idx;
#endif

	printk("==========GEC210: %s selected ID=%d===========\n", gec210_lcd_config[lcd_idx].name,lcd_idx);
	return gec210_lcd_config[lcd_idx].lcd;
}

void gec210_get_lcd_res(int *w, int *h)
{
	struct s3cfb_lcd *lcd = gec210_lcd_config[lcd_idx].lcd;

	if (w)
		*w = lcd->width;
	if (h)
		*h = lcd->height;

	return;
}
EXPORT_SYMBOL(gec210_get_lcd_res);

#if 1

#if defined(CONFIG_TOUCHSCREEN_GOODIX) || \
	defined(CONFIG_TOUCHSCREEN_FT5X0X)
static unsigned int ctp_type = CTP_NONE;

static int __init gec210_set_ctp(char *str)
{
	unsigned int val;
	char *p = str, *end;

	val = simple_strtoul(p, &end, 10);
	if (end <= p) {
		return 1;
	}

	if (val < CTP_MAX && gec210_lcd_config[lcd_idx].ctp) {
		ctp_type = val;
	}

	return 1;
}
__setup("ctp=", gec210_set_ctp);

unsigned int gec210_get_ctp(void)
{
	return ctp_type;
}
EXPORT_SYMBOL(gec210_get_ctp);
#endif

#endif

