/*
 * drivers/input/touchscreen/tsc2007.c
 *
 * Copyright (c) 2008 MtekVision Co., Ltd.
 *	Kwangwoo Lee <kwlee@mtekvision.com>
 *
 * Using code from:
 *  - ads7846.c
 *	Copyright (c) 2005 David Brownell
 *	Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/tsc2007.h>
#include <asm/delay.h>


#include <mach/irqs.h>
#include <plat/irqs.h>
#include <plat/gpio-cfg.h>  //for s3c_gpio_cfgpin
#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/delay.h>
#include <mach/irqs.h>

#define TS_POLL_DELAY			2 /* ms delay between samples */
#define TS_POLL_PERIOD			2 /* ms delay between samples */

#define TSC2007_MEASURE_TEMP0		(0x0 << 4)
#define TSC2007_MEASURE_AUX		(0x2 << 4)
#define TSC2007_MEASURE_TEMP1		(0x4 << 4)
#define TSC2007_ACTIVATE_XN		(0x8 << 4)
#define TSC2007_ACTIVATE_YN		(0x9 << 4)
#define TSC2007_ACTIVATE_YP_XN		(0xa << 4)
#define TSC2007_SETUP			(0xb << 4)
#define TSC2007_MEASURE_X		(0xc << 4)
#define TSC2007_MEASURE_Y		(0xd << 4)
#define TSC2007_MEASURE_Z1		(0xe << 4)
#define TSC2007_MEASURE_Z2		(0xf << 4)

#define TSC2007_POWER_OFF_IRQ_EN	(0x0 << 2)
#define TSC2007_ADC_ON_IRQ_DIS0		(0x1 << 2)
#define TSC2007_ADC_OFF_IRQ_EN		(0x2 << 2)
#define TSC2007_ADC_ON_IRQ_DIS1		(0x3 << 2)

#define TSC2007_12BIT			(0x0 << 1)
#define TSC2007_8BIT			(0x1 << 1)

#define	MAX_12BIT			((1 << 12) - 1)

#define ADC_ON_12BIT	(TSC2007_12BIT | TSC2007_ADC_ON_IRQ_DIS0)

#define READ_Y		(ADC_ON_12BIT | TSC2007_MEASURE_Y)
#define READ_Z1		(ADC_ON_12BIT | TSC2007_MEASURE_Z1)
#define READ_Z2		(ADC_ON_12BIT | TSC2007_MEASURE_Z2)
#define READ_X		(ADC_ON_12BIT | TSC2007_MEASURE_X)
#define PWRDOWN		(TSC2007_12BIT | TSC2007_POWER_OFF_IRQ_EN)

#define SEND      1
#define RECV      2


struct ts_event {
	u16	x;
	u16	y;
	u16	z1, z2;
};

struct tsc2007 {
	struct input_dev	*input;
	char			phys[32];
	struct delayed_work	work;

	struct i2c_client	*client;

	u16			model;
	u16			x_plate_ohms;

	bool			pendown;
	int			irq;

	int			(*get_pendown_state)(void);
	void			(*clear_penirq)(void);
};

#define	SAM_CNT	       8
#define   THRESHOLD   10
static int xy[2][SAM_CNT];
static int count;
static int tmp_x[9];
static int tmp_y[9];

static inline  int tsc2007_xfer(struct tsc2007 *tsc, unsigned char cmd,unsigned short *data, int dir)
{
	int ret;
	unsigned char d[2]={0};
	if( dir == SEND ){
		ret = i2c_master_send(tsc->client,&cmd,1);
		if(ret < 0){
		//	printk("tsc2007 send data failed\n");
			goto error;
		}
		udelay(20);
	}
	if( dir == RECV ){
		ret = i2c_master_recv(tsc->client,d,2);
		if(ret < 0){
		//	printk("tsc2007 recv data failed\n");
			goto error;

		}
		*data = ( d[0] << 4 ) ;
		*data += ( d[1] >> 4 );
		udelay(20);

	}
	memset(d,0,2);
	return ret;

	/* The protocol and raw data format from i2c interface:
	 * S Addr Wr [A] Comm [A] S Addr Rd [A] [DataLow] A [DataHigh] NA P
	 * Where DataLow has [D11-D4], DataHigh has [D3-D0 << 4 | Dummy 4bit].
	 */
error:
	return -ENODEV;
//	dev_dbg(&tsc->client->dev, "data: 0x%x, val: 0x%x\n", data);

}



static void tsc2007_read_values(struct tsc2007 *tsc, struct ts_event *tc)
{
	

	/* y- still on; turn on only y+ (and ADC) */
// int tsc2007_xfer(struct tsc2007 *tsc, u8 cmd,int *data, int dir)
	int tmp;
	 tsc2007_xfer(tsc,READ_X ,NULL,SEND);
	 tsc2007_xfer(tsc,0,&tc->x,RECV);

	 tsc2007_xfer(tsc, READ_Y,NULL,SEND);
	 tsc2007_xfer(tsc,0,&tc->y,RECV);
	 

	 tsc2007_xfer(tsc, READ_Z1,NULL,SEND);
	 tsc2007_xfer(tsc,0,&tc->z1,RECV);

	 tsc2007_xfer(tsc, READ_Z2,NULL,SEND);
	 tsc2007_xfer(tsc,0,&tc->z2,RECV);

	/* power down */
	 tsc2007_xfer(tsc, PWRDOWN,NULL,SEND);

	return;

}

static u32 tsc2007_calculate_pressure(struct tsc2007 *tsc, struct ts_event *tc)
{
	u32 rt = 0;

	/* range filtering */
	if (tc->x == MAX_12BIT)
		tc->x = 0;

	if (likely(tc->x && tc->z1)) {
		/* compute touch pressure resistance using equation #1 */
		rt = tc->z2 - tc->z1;
		rt *= tc->x;
		rt *= tsc->x_plate_ohms;
		rt /= tc->z1;
		rt = (rt + 2047) >> 12;
	}

	return rt;
}

static void tsc2007_send_up_event(struct tsc2007 *tsc)
{
	struct input_dev *input = tsc->input;

	dev_dbg(&tsc->client->dev, "UP\n");

	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);
}
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

static void tsc2007_work(struct work_struct *work)
{
	struct tsc2007 *ts =
		container_of(to_delayed_work(work), struct tsc2007, work);
	struct ts_event tc={0};
	u32 rt;

	/*虽然tsc2007有压力传感功能，但是我们不能仅靠压力来判断笔按下模式
	当笔抬起的时候，压力传感的值会波动一会。
	 * NOTE: We can't rely on the pressure to determine the pen down
	 * state, even though this controller has a pressure sensor.
	 * The pressure value can fluctuate for quite a while after
	 * lifting the pen and in some cases may not even settle at the
	 * expected value.
	 *安全的方法是在中断下半部的时候读取该中断管脚的状态，但是不幸的是
	 这种方式不是总是有效的，在这种情况下还是需要依靠查看压力的值
	 * The only safe way to check for the pen up condition is in the
	 * work function by reading the pen signal state (it's a GPIO
	 * and IRQ). Unfortunately such callback is not always available,
	 * in that case we have rely on the pressure anyway.
	 */
	if (ts->get_pendown_state) {
		if (unlikely(!ts->get_pendown_state())) {
			tsc2007_send_up_event(ts);
			ts->pendown = false;
			goto out;
		}

		dev_dbg(&ts->client->dev, "pen is still down\n");
	}

	tsc2007_read_values(ts, &tc);

	rt = tsc2007_calculate_pressure(ts, &tc);
//	printk("========pressure=%d=========\n",rt);
	if (rt > MAX_12BIT) {
		/*
		 * Sample found inconsistent by debouncing or pressure is
		 * beyond the maximum. Don't report it to user space,
		 * repeat at least once more the measurement.
		 */
		dev_dbg(&ts->client->dev, "ignored pressure %d\n", rt);
		goto out;

	}
	//检测压力值，用于去抖动
//	if (rt < 250) {
	if(rt < 600) {
		struct input_dev *input = ts->input;

		if (!ts->pendown) {
		//	dev_dbg(&ts->client->dev, "DOWN\n");

			input_report_key(input, BTN_TOUCH, 1);
			ts->pendown = true;
		}

		tmp_x[count] = tc.x;
		tmp_y[count] = tc.y;
		count++;
//		printk("count=%d\n",count);
		if(count == SAM_CNT)
		{
			dot_filter(&tc.x,&tc.y);
		//	do_dot_filter(&tc.x,&tc.y);
	//		printk("====x=%d      y=%d=====\n",tc.x,tc.y);
		//上报坐标值
			input_report_abs(input, ABS_X, tc.y);
			input_report_abs(input, ABS_Y, tc.x);
			input_report_abs(input, ABS_PRESSURE, rt);

			input_sync(input);
			count = 0;
		}
		
		dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
			tc.x, tc.y, rt);

	} else if (!ts->get_pendown_state && ts->pendown) {
		/*
		 * We don't have callback to check pendown state, so we
		 * have to assume that since pressure reported is 0 the
		 * pen was lifted up.
		 */
		tsc2007_send_up_event(ts);
		ts->pendown = false;
	}

 out:
	if (ts->pendown)
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_PERIOD));
	else
		enable_irq(ts->irq);
}

static irqreturn_t tsc2007_irq(int irq, void *handle)
{
	struct tsc2007 *ts = handle;
//	int *key4 = handle;
//	if(  *key4 = 19)
//		printk("tsc2007 interrupt value is %x\n", irq);
#if 1
	if (!ts->get_pendown_state || likely(ts->get_pendown_state())) {
		disable_irq_nosync(ts->irq);
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_DELAY));
	}

	if (ts->clear_penirq)
		ts->clear_penirq();

#endif

	return IRQ_HANDLED;
}

static void tsc2007_free_irq(struct tsc2007 *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}

static int __devinit tsc2007_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct tsc2007 *ts;
	struct tsc2007_platform_data *pdata = pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}
	
#if 0
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;
#endif

	ts = kzalloc(sizeof(struct tsc2007), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	client->flags &= ~I2C_M_TEN; //for 7 bits address mode
	INIT_DELAYED_WORK(&ts->work, tsc2007_work);

	ts->model             = pdata->model;
	ts->x_plate_ohms      = pdata->x_plate_ohms;
	ts->get_pendown_state = pdata->get_pendown_state;
	ts->clear_penirq      = pdata->clear_penirq;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev->name = "TSC2007 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);

//	if (pdata->init_platform_hw)
//		pdata->init_platform_hw();
	printk("config GPH1-6 to EINT14\n");
	if( gpio_request(S5PV210_GPH1(6), "GPH1_6") )
			printk("GPIO_REQUEST FOR  tsc2007 failed\n");;
	gpio_direction_output(S5PV210_GPH1(6), 1);
	udelay(10);
	gpio_free(S5PV210_GPH1(6));
	printk("EINT  the value is %d\n",gpio_get_value( S5PV210_GPH1(6)));
	
	s3c_gpio_cfgpin(S5PV210_GPH1(6), S3C_GPIO_SFN(0XF));
	s3c_gpio_setpull(S5PV210_GPH1(6), S3C_GPIO_PULL_NONE);
	
	ts->irq   = gpio_to_irq(S5PV210_GPH1(6));
		

	err = request_irq(ts->irq, tsc2007_irq, 0,
			client->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	/* Prepare for touch readings - power down ADC and enable PENIRQ 配置触摸屏的模式为开中断，待机模式*/
	err = tsc2007_xfer(ts, PWRDOWN,NULL,SEND);
	if (err < 0){
		printk("tsc2007_xfer(ts, PWRDOWN,NULL,SEND)\n");
	//	goto err_free_irq;
	}

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	i2c_set_clientdata(client, ts);

	printk("--------->Now tsc2007 probed OK<-----------\n");
	return 0;

 err_free_irq:
	tsc2007_free_irq(ts);
	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit tsc2007_remove(struct i2c_client *client)
{
	struct tsc2007	*ts = i2c_get_clientdata(client);
	struct tsc2007_platform_data *pdata = client->dev.platform_data;

	tsc2007_free_irq(ts);

	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();

	input_unregister_device(ts->input);
	kfree(ts);
	printk("======tsc2007_remove=======\n");
	return 0;
}

static const struct i2c_device_id tsc2007_idtable[] = {
	{ "tsc2007", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tsc2007_idtable);

static struct i2c_driver tsc2007_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tsc2007"
	},
	.id_table	= tsc2007_idtable,
	.probe		= tsc2007_probe,
	.remove		= __devexit_p(tsc2007_remove),
};

//for test interrupt
#if 0
static irqreturn_t EINT_FUNC(int irq,void *dev_id)
{
	int eint15=0,eint19=1;
	eint19=gpio_get_value ( S5PV210_GPH2(3) ) ;
	eint15=gpio_get_value( S5PV210_GPH1(7) );
	printk("Eint_19 interrupt eint19=%d  eint15=%d\n",eint19,eint15);
	return IRQ_HANDLED;
}


int irq,key4=19;

#endif

static int __init tsc2007_init(void)
{
#if 0
	//FOR KEY4
	int ret;

	s3c_gpio_cfgpin(S5PV210_GPH2(3), S3C_GPIO_SFN(0XF));
	s3c_gpio_setpull(S5PV210_GPH2(3), S3C_GPIO_PULL_NONE);

	gpio_request(S5PV210_GPH2(3), "GPGPH2_3");
	gpio_direction_output(S5PV210_GPH2(3), 1);
	udelay(10);
	gpio_free(S5PV210_GPH2(3));
	
	irq = gpio_to_irq(S5PV210_GPH2(3));
	ret = request_irq(irq,EINT_FUNC,IRQ_TYPE_EDGE_FALLING,"key4",(void*)&key4);
	if(ret){
		printk("request irq failed\n");
	}

#endif

	return i2c_add_driver(&tsc2007_driver);
}

static void __exit tsc2007_exit(void)
{
//	free_irq(irq,(void*)&key4);
	i2c_del_driver(&tsc2007_driver);
}

module_init(tsc2007_init);
module_exit(tsc2007_exit);

MODULE_AUTHOR("Kwangwoo Lee <kwlee@mtekvision.com>");
MODULE_DESCRIPTION("TSC2007 TouchScreen Driver");
MODULE_LICENSE("GPL");
