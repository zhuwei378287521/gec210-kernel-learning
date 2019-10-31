/* linux/drivers/input/touchscreen/s3c-ts.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (c) 2004 Arnaud Patard <arnaud.patard@rtp-net.org>
 * iPAQ H1940 touchscreen support
 *
 * ChangeLog
 *
 * 2004-09-05: Herbert Potzl <herbert@13thfloor.at>
 *	- added clock (de-)allocation code
 *
 * 2005-03-06: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - h1940_ -> s3c24xx (this driver is now also used on the n30
 *        machines :P)
 *      - Debug messages are now enabled with the config option
 *        TOUCHSCREEN_S3C_DEBUG
 *      - Changed the way the value are read
 *      - Input subsystem should now work
 *      - Use ioremap and readl/writel
 *
 * 2005-03-23: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Make use of some undocumented features of the touchscreen
 *        controller
 *
 * 2006-09-05: Ryu Euiyoul <ryu.real@gmail.com>
 *      - added power management suspend and resume code
 *
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>

#include <mach/regs-adc.h>
#include <mach/ts-s3c.h>
#include <mach/irqs.h>
#include <mach/map.h>

#define CONFIG_CPU_S5PV210_EVT1

#ifdef CONFIG_CPU_S5PV210_EVT1
#define X_COOR_MIN	0//100
#define X_COOR_MAX	4096//3980
#define X_COOR_FUZZ	32
#define Y_COOR_MIN	0//200
#define Y_COOR_MAX	4096//3760
#define Y_COOR_FUZZ	32
#endif

#define	SAM_CNT	       9
#define   THRESHOLD   10
static int xy[2][SAM_CNT];
static int count;
static int tmp_x[SAM_CNT];
static int tmp_y[SAM_CNT];

//中值滤波
static void dot_filter(int *x_dot,int *y_dot)
{
	int i,j,k,tmp,x_sum=0,y_sum=0;
	//for x
	for(j=0;j<SAM_CNT-1;j++){
		for(i=0;i<SAM_CNT-j;i++)
		{
			if(tmp_x[i] > tmp_x[i+1]){
				tmp = tmp_x[i];
				tmp_x[i] = tmp_x[i+1];
				tmp_x[i+1] = tmp;
			}
		}

	}

	//for y
	for(j=0;j<SAM_CNT-1;j++){
		for(i=0;i<SAM_CNT-j;i++)
		{
			if(tmp_y[i] > tmp_y[i+1]){
				tmp = tmp_y[i];
				tmp_y[i] = tmp_y[i+1];
				tmp_y[i+1] = tmp;
			}
		}

	}
	
#if 0
	for(k=0;k<SAM_CNT;k++)
		printk("==tmp_x[%d]=%d tmp_y[%d]=%d==\n",k,tmp_x[k],k,tmp_y[k]);
#endif

	for(i=1;i<SAM_CNT-1;i++){
		x_sum += tmp_x[i];
		y_sum += tmp_y[i];
	}
	*x_dot = x_sum/(SAM_CNT - 2);
	*y_dot = y_sum/(SAM_CNT - 2);

	
}



static void do_dot_filter(int *x, int *y)
{
	int i,j,x_m0,x_m1,x_m2,xm[3];
	int y_m0,y_m1,y_m2,ym[3];
	//for x 
	xm[0] = (tmp_x[0] + tmp_x[1] + tmp_x[2])/3;
	xm[1] = (tmp_x[3] + tmp_x[4] + tmp_x[5])/3;
	xm[2] = (tmp_x[6] + tmp_x[7] + tmp_x[8])/3;
	x_m0 = (xm[0] - xm[1]) > 0 ? (xm[0] -xm[1] ) : (xm[1] - xm[0]);
	x_m1 = (xm[1] - xm[2]) > 0 ? (xm[1] - xm[2]) : (xm[2] - xm[1]);
	x_m2 = (xm[2] - xm[0]) > 0 ? (xm[2] - xm[0]) : (xm[0] - xm[2]);
	if(x_m0 > THRESHOLD || x_m1 > THRESHOLD || x_m2 > THRESHOLD){
		return;
	}
	if(xm[0] < xm[1]){
		if(xm[2] < xm[0])
			*x = (xm[0] + xm[2])/2;
		else
			*x = (xm[0] + xm[1])/2;
	}
	else if(xm[2] < xm[1])
		*x = (xm[0] + xm[2])/2;
		else
			*x = (xm[1] + xm[2])/2;
	//for y
	ym[0] = (tmp_y[0] + tmp_y[1] + tmp_y[2])/3;
	ym[1] = (tmp_y[3] + tmp_y[4] + tmp_y[5])/3;
	ym[2] = (tmp_y[6] + tmp_y[7] + tmp_y[8])/3;
	y_m0 = (ym[0] - ym[1]) > 0 ? (ym[0] -ym[1] ) : (ym[1] - ym[0]);
	y_m1 = (ym[1] - ym[2]) > 0 ? (ym[1] - ym[2]) : (ym[2] - ym[1]);
	y_m2 = (ym[2] - ym[0]) > 0 ? (ym[2] - ym[0]) : (ym[0] - ym[2]);
	if(y_m0 > THRESHOLD || y_m1 > THRESHOLD || y_m2 > THRESHOLD){
		return;
	}
	if(ym[0] < ym[1]){
		if(ym[2] < ym[0])
			*y = (ym[0] + ym[2])/2;
		else
			*y = (ym[0] + ym[1])/2;
	}
	else if(ym[2] < ym[1])
		*y = (ym[0] + ym[2])/2;
		else
			*y = (ym[1] + ym[2])/2;
		
}



/* For ts->dev.id.version */
#define S3C_TSVERSION	0x0101

#define WAIT4INT(x)	(((x)<<8) | \
			S3C_ADCTSC_YM_SEN | S3C_ADCTSC_YP_SEN | \
			S3C_ADCTSC_XP_SEN | S3C_ADCTSC_XY_PST(3))

#define AUTOPST	(S3C_ADCTSC_YM_SEN | S3C_ADCTSC_YP_SEN | \
		 S3C_ADCTSC_XP_SEN | S3C_ADCTSC_AUTO_PST | \
		 S3C_ADCTSC_XY_PST(0))

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ts_early_suspend(struct early_suspend *h);
static void ts_late_resume(struct early_suspend *h);
#endif

/* Touchscreen default configuration */
struct s3c_ts_mach_info s3c_ts_default_cfg __initdata = {
	.delay			= 10000,
	.presc			= 49,
	.oversampling_shift	= 2,
	.resol_bit		= 10
};

/*
 * Definitions & global arrays.
 */
static char *s3c_ts_name = "S5P TouchScreen";
static void __iomem		*ts_base;
static struct resource		*ts_mem;
static struct resource		*ts_irq;
static struct clk		*ts_clock;
static struct s3c_ts_info	*ts;

#if 1
static struct ts_temp_st
{
	int x[3];
	int y[3];
	unsigned char count;
}ts_temp = {.count=0};

#endif

static void touch_timer_fire(unsigned long data)
{
	unsigned long data0;
	unsigned long data1;
	int updown;
	int x, y;

	data0 = readl(ts_base+S3C_ADCDAT0);
	data1 = readl(ts_base+S3C_ADCDAT1);

	updown = (!(data0 & S3C_ADCDAT0_UPDOWN)) &&
		 (!(data1 & S3C_ADCDAT1_UPDOWN));

	if (updown) {
		if (ts->count) {
			x = (int)ts->xp/ts->count;
			y = (int)ts->yp/ts->count;

#ifdef CONFIG_FB_S3C_LTE480WV
			y = 4000 - y;
#endif		
			y = 4000 - y;
	//		printk("======x=%d y=%d\n",x,y);
			
	//		tmp_x[count] = x;
	//		tmp_y[count] = y;
	//		count++;
	//		printk("count=%d\n",count);
		//	if(count == SAM_CNT)
			{
			//	dot_filter(&x,&y);
			//	do_dot_filter(&x,&y);
				input_report_abs(ts->dev, ABS_X, x);
				input_report_abs(ts->dev, ABS_Y, y);
				input_report_abs(ts->dev, ABS_Z, 0);
				input_report_key(ts->dev, BTN_TOUCH, 1);
				input_report_abs(ts->dev, ABS_PRESSURE, 1);
				input_sync(ts->dev);
				count = 0;
			}
		}

		ts->xp = 0;
		ts->yp = 0;
		ts->count = 0;

		writel(S3C_ADCTSC_PULL_UP_DISABLE | AUTOPST,
				ts_base + S3C_ADCTSC);
		writel(readl(ts_base+S3C_ADCCON) | S3C_ADCCON_ENABLE_START,
				ts_base + S3C_ADCCON);
	} else {
		ts->count = 0;
		input_report_abs(ts->dev, ABS_X, ts->xp);
		input_report_abs(ts->dev, ABS_Y, ts->yp);
		input_report_abs(ts->dev, ABS_Z, 0);
		input_report_key(ts->dev, BTN_TOUCH, 0);
		input_report_abs(ts->dev, ABS_PRESSURE, 0);
		input_sync(ts->dev);

		writel(WAIT4INT(0), ts_base+S3C_ADCTSC);
	}
}

static struct timer_list touch_timer =
		TIMER_INITIALIZER(touch_timer_fire, 0, 0);

static irqreturn_t stylus_updown(int irqno, void *param)
{
	unsigned long data0;
	unsigned long data1;
	int updown;

	data0 = readl(ts_base + S3C_ADCDAT0);
	data1 = readl(ts_base + S3C_ADCDAT1);

	updown = (!(data0 & S3C_ADCDAT0_UPDOWN)) &&
		 (!(data1 & S3C_ADCDAT1_UPDOWN));

	/* TODO we should never get an interrupt with updown set while
	 * the timer is running, but maybe we ought to verify that the
	 * timer isn't running anyways. */

	if (updown)
		touch_timer_fire(0);

	if (ts->s3c_adc_con == ADC_TYPE_2) {
		__raw_writel(0x0, ts_base + S3C_ADCCLRWK);
		__raw_writel(0x0, ts_base + S3C_ADCCLRINT);
	}

	return IRQ_HANDLED;
}


int evaluate(int *num)
{
	int result;
	int diff[3];
	diff[0] = num[0] - num[1];
	diff[1] = num[1] - num[2];
	diff[2] = num[2] - num[0];	
	diff[0] = diff[0] >0? diff[0] : -diff[0];
	diff[1] = diff[1] >0? diff[1] : -diff[1];	
	diff[2] = diff[2] >0? diff[2] : -diff[2];	
	if(diff[0] < diff[1])
	{
		result = num[0] + ((diff[2] < diff[0])? num[2]:num[1]);
	}
	else
	{
		result = num[2] + ((diff[2] < diff[1])? num[0]:num[1]);	
	}
	result >>= 1;
	return result;
}


static int s3c_filter_ts(int x[], int y[]) /* 软件过滤 */

{

#define ERR_LIMIT 10      /* 经验值，容差值 */   



    int avr_x, avr_y;

    int det_x, det_y;
    
    avr_x = (x[0] + x[1])/2;

    avr_y = (y[0] + y[1])/2;



    det_x = (x[2] > avr_x) ? (x[2] - avr_x) : (avr_x - x[2]);

    det_y = (y[2] > avr_y) ? (y[2] - avr_y) : (avr_y - y[2]);



    if ((det_x > ERR_LIMIT) || (det_y > ERR_LIMIT))

        return 0;

    

    avr_x = (x[1] + x[2])/2;

    avr_y = (y[1] + y[2])/2;



    det_x = (x[3] > avr_x) ? (x[3] - avr_x) : (avr_x - x[3]);

    det_y = (y[3] > avr_y) ? (y[3] - avr_y) : (avr_y - y[3]);



    if ((det_x > ERR_LIMIT) || (det_y > ERR_LIMIT))

        return 1;

}


//ADC值转换成功，读取ADC的值
static irqreturn_t stylus_action(int irqno, void *param)
{
	unsigned long data0;
	unsigned long data1;
	int updown;

	data0 = readl(ts_base + S3C_ADCDAT0);
	data1 = readl(ts_base + S3C_ADCDAT1);

	if (ts->resol_bit == 12) {
#if defined(CONFIG_TOUCHSCREEN_NEW)
		ts->yp += S3C_ADCDAT0_XPDATA_MASK_12BIT -
			(data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT);
		ts->xp += S3C_ADCDAT1_YPDATA_MASK_12BIT -
			(data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT);
#else
 //		ts->xp += S3C_ADCDAT0_XPDATA_MASK_12BIT -
 //			(data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT);
//XJM =====================================================
		updown = (!(data0 & S3C_ADCDAT0_UPDOWN)) &&
			 (!(data1 & S3C_ADCDAT1_UPDOWN));
		//如果触摸笔还是处于按下的状态，则笔尖不是按下状态，则该坐标值无效
		if(updown)  
		{
		#if 0
			ts_temp.x[ts_temp.count] =  (data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT);
			ts_temp.y[ts_temp.count] =  data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT;
			ts_temp.count++;	
			if(ts_temp.count == 3)
			{
				ts_temp.count = 0;
				ts->xp += evaluate(ts_temp.x);
				ts->yp += evaluate(ts_temp.y);		

				
				ts->count++;	
			}	
		#endif
		#if 1
	/* 优化措施3:
         * 多次测量取平均值
         */
			ts_temp.x[ts_temp.count] =  (data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT);
			ts_temp.y[ts_temp.count] =  data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT;
			ts_temp.count++;	
			if(ts_temp.count == 4)
			{
				ts_temp.count = 0;	
				s3c_filter_ts(ts_temp.x,ts_temp.y);
				ts->xp +=( ts_temp.x[0] + ts_temp.x[1] + ts_temp.x[2] + ts_temp.x[3]  )/4;
				ts->yp +=( ts_temp.y[0] + ts_temp.y[1] + ts_temp.y[2] + ts_temp.y[3]  )/4;
				ts->count++;	
			}		
		#endif	
	//		ts->xp += (data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT);
	//		ts->yp += data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT;
		}
#endif
	} else {
#if defined(CONFIG_TOUCHSCREEN_NEW)
		ts->yp += S3C_ADCDAT0_XPDATA_MASK -
			(data0 & S3C_ADCDAT0_XPDATA_MASK);
		ts->xp += S3C_ADCDAT1_YPDATA_MASK -
			(data1 & S3C_ADCDAT1_YPDATA_MASK);
#else
		ts->xp += data0 & S3C_ADCDAT0_XPDATA_MASK;
		ts->yp += data1 & S3C_ADCDAT1_YPDATA_MASK;
#endif
	}

//	ts->count++;

	if (ts->count < (1<<ts->shift)) {
		writel(S3C_ADCTSC_PULL_UP_DISABLE | AUTOPST,
				ts_base + S3C_ADCTSC);
		writel(readl(ts_base + S3C_ADCCON) | S3C_ADCCON_ENABLE_START,
				ts_base + S3C_ADCCON);
	} else {
		mod_timer(&touch_timer, jiffies + 1);
		writel(WAIT4INT(1), ts_base + S3C_ADCTSC);
	}

	if (ts->s3c_adc_con == ADC_TYPE_2) {
		__raw_writel(0x0, ts_base+S3C_ADCCLRWK);
		__raw_writel(0x0, ts_base+S3C_ADCCLRINT);
	}
	return IRQ_HANDLED;
}

static struct s3c_ts_mach_info *s3c_ts_get_platdata(struct device *dev)
{
	if (dev->platform_data != NULL)
		return (struct s3c_ts_mach_info *)dev->platform_data;

	return &s3c_ts_default_cfg;
}

/*
 * The functions for inserting/removing us as a module.
 */
static int __init s3c_ts_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev;
	struct input_dev *input_dev;
	struct s3c_ts_mach_info *s3c_ts_cfg;
	void __iomem * ts_base0;
	void __iomem * ts_base1;
	int ret, size;
	int irq_flags = 0;

	dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "no memory resource specified\n");
		return -ENOENT;
	}

	size = (res->end - res->start) + 1;
	ts_mem = request_mem_region(res->start, size, pdev->name);
	if (ts_mem == NULL) {
		dev_err(dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

	ts_base = ioremap(res->start, size);
	if (ts_base == NULL) {
		dev_err(dev, "failed to ioremap() region\n");
		ret = -EINVAL;
		goto err_map;
	}

	ts_clock = clk_get(&pdev->dev, "adc");
	if (IS_ERR(ts_clock)) {
		dev_err(dev, "failed to find watchdog clock source\n");
		ret = PTR_ERR(ts_clock);
		goto err_clk;
	}

	clk_enable(ts_clock);

	s3c_ts_cfg = s3c_ts_get_platdata(&pdev->dev);
	if ((s3c_ts_cfg->presc & 0xff) > 0)
		writel(S3C_ADCCON_PRSCEN |
				S3C_ADCCON_PRSCVL(s3c_ts_cfg->presc & 0xFF),
				ts_base+S3C_ADCCON);
	else
		writel(0, ts_base + S3C_ADCCON);

	/* Initialise registers */
	if ((s3c_ts_cfg->delay & 0xffff) > 0)
		writel(s3c_ts_cfg->delay | 0xffff, ts_base + S3C_ADCDLY);

	if (s3c_ts_cfg->resol_bit == 12) {
		switch (s3c_ts_cfg->s3c_adc_con) {
		case ADC_TYPE_2:
			writel(readl(ts_base+S3C_ADCCON)|S3C_ADCCON_RESSEL_12BIT, ts_base+S3C_ADCCON);
			
			ts_base0 = ioremap(S3C_PA_ADC, SZ_4K);
			writel(readl(ts_base0 + S3C_ADCCON) | (0x1 << 17), ts_base0 + S3C_ADCCON);
			
			ts_base1 = ioremap(S3C_PA_ADC1, SZ_4K);
			writel(readl(ts_base1 + S3C_ADCCON) | (0x1 << 17), ts_base1 + S3C_ADCCON);

			iounmap(ts_base0);
			iounmap(ts_base1);
			break;

		case ADC_TYPE_1:
			writel(readl(ts_base+S3C_ADCCON)|S3C_ADCCON_RESSEL_12BIT_1, ts_base+S3C_ADCCON);
			break;

		default:
			dev_err(dev, "this type of AP isn't supported !\n");
			break;
		}
	}

	writel(WAIT4INT(0), ts_base + S3C_ADCTSC);

	ts = kzalloc(sizeof(struct s3c_ts_info), GFP_KERNEL);

	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	ts->dev = input_dev;

	ts->dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	if (s3c_ts_cfg->resol_bit == 12) {
		input_set_abs_params(ts->dev,
				ABS_X, X_COOR_MIN, X_COOR_MAX, X_COOR_FUZZ, 0);
		input_set_abs_params(ts->dev,
				ABS_Y, Y_COOR_MIN, Y_COOR_MAX, Y_COOR_FUZZ, 0);
	} else {
		input_set_abs_params(ts->dev, ABS_X, 0, 0x3FF, 0, 0);
		input_set_abs_params(ts->dev, ABS_Y, 0, 0x3FF, 0, 0);
	}

	input_set_abs_params(ts->dev, ABS_PRESSURE, 0, 1, 0, 0);

	sprintf(ts->phys, "input(ts)");

	ts->dev->name = s3c_ts_name;
	ts->dev->phys = ts->phys;
	ts->dev->id.bustype = BUS_RS232;
	ts->dev->id.vendor = 0xDEAD;
	ts->dev->id.product = 0xBEEF;
	ts->dev->id.version = S3C_TSVERSION;

	ts->shift = s3c_ts_cfg->oversampling_shift;
	ts->resol_bit = s3c_ts_cfg->resol_bit;
	ts->s3c_adc_con = s3c_ts_cfg->s3c_adc_con;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ts_early_suspend;
	ts->early_suspend.resume = ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	/* For IRQ_PENDUP */
	ts_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (ts_irq == NULL) {
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_irq;
	}

	ret = request_irq(ts_irq->start, stylus_updown, irq_flags,
			"s3c_updown", ts);
	if (ret != 0) {
		dev_err(dev, "s3c_ts.c: Could not allocate ts IRQ_PENDN !\n");
		ret = -EIO;
		goto err_irq;
	}

	/* For IRQ_ADC */
	ts_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (ts_irq == NULL) {
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_irq;
	}

	ret = request_irq(ts_irq->start, stylus_action, irq_flags,
			"s3c_action", ts);
	if (ret != 0) {
		dev_err(dev, "s3c_ts.c: Could not allocate ts IRQ_ADC !\n");
		ret =  -EIO;
		goto err_irq;
	}

	printk(KERN_INFO "%s got loaded successfully : %d bits\n",
			s3c_ts_name, s3c_ts_cfg->resol_bit);

	/* All went ok, so register to the input system */
	ret = input_register_device(ts->dev);
	if (ret) {
		dev_err(dev, "Could not register input device(touchscreen)!\n");
		ret = -EIO;
		goto fail;
	}

	return 0;

fail:
	free_irq(ts_irq->start, ts->dev);
	free_irq(ts_irq->end, ts->dev);

err_irq:
	input_free_device(input_dev);
	kfree(ts);

err_alloc:
	clk_disable(ts_clock);
	clk_put(ts_clock);

err_clk:
	iounmap(ts_base);

err_map:
	release_resource(ts_mem);
	kfree(ts_mem);

err_req:
	return ret;
}

static int s3c_ts_remove(struct platform_device *dev)
{
	printk(KERN_INFO "s3c_ts_remove() of TS called !\n");

	disable_irq(IRQ_ADC1);
	disable_irq(IRQ_PENDN1);

	free_irq(IRQ_PENDN1, ts->dev);
	free_irq(IRQ_ADC1, ts->dev);

	if (ts_clock) {
		clk_disable(ts_clock);
		clk_put(ts_clock);
		ts_clock = NULL;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
     unregister_early_suspend(&ts->early_suspend);
#endif

	input_unregister_device(ts->dev);
	iounmap(ts_base);

	return 0;
}

#ifdef CONFIG_PM
static unsigned int adccon, adctsc, adcdly;

static int s3c_ts_suspend(struct platform_device *dev, pm_message_t state)
{
	adccon = readl(ts_base+S3C_ADCCON);
	adctsc = readl(ts_base+S3C_ADCTSC);
	adcdly = readl(ts_base+S3C_ADCDLY);

	disable_irq(IRQ_ADC1);
	disable_irq(IRQ_PENDN1);

	clk_disable(ts_clock);

	return 0;
}

static int s3c_ts_resume(struct platform_device *pdev)
{
	clk_enable(ts_clock);

	writel(adccon, ts_base+S3C_ADCCON);
	writel(adctsc, ts_base+S3C_ADCTSC);
	writel(adcdly, ts_base+S3C_ADCDLY);
	writel(WAIT4INT(0), ts_base+S3C_ADCTSC);

	enable_irq(IRQ_ADC1);
	enable_irq(IRQ_PENDN1);

	return 0;
}
#else
#define s3c_ts_suspend NULL
#define s3c_ts_resume  NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ts_early_suspend(struct early_suspend *h)
{
	struct s3c_ts_info *ts;
	ts = container_of(h, struct s3c_ts_info, early_suspend);
	s3c_ts_suspend(NULL, PMSG_SUSPEND);
}

static void ts_late_resume(struct early_suspend *h)
{
	struct s3c_ts_info *ts;
	ts = container_of(h, struct s3c_ts_info, early_suspend);
	s3c_ts_resume(NULL);
}
#endif

static struct platform_driver s3c_ts_driver = {
       .probe          = s3c_ts_probe,
       .remove         = s3c_ts_remove,
       .suspend        = s3c_ts_suspend,
       .resume         = s3c_ts_resume,
       .driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-ts",
	},
};

static char banner[] __initdata = KERN_INFO \
		"S5P Touchscreen driver, (c) 2008 Samsung Electronics\n";

static int __init s3c_ts_init(void)
{
	printk(banner);
	return platform_driver_register(&s3c_ts_driver);
}

static void __exit s3c_ts_exit(void)
{
	platform_driver_unregister(&s3c_ts_driver);
}

module_init(s3c_ts_init);
module_exit(s3c_ts_exit);

MODULE_AUTHOR("Samsung AP");
MODULE_DESCRIPTION("S5P touchscreen driver");
MODULE_LICENSE("GPL");
