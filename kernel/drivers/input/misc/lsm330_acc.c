/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
 *
 * File Name		: lsm330_acc.c
 * Authors		: MSH - Motion Mems BU - Application Team
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Denis Ciocca (denis.ciocca@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
* Version		: V.1.0.2
* Date			: 2012/Oct/15
 * Description		: LSM330 accelerometer driver
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
Version History.
	V 1.0.0		First Release
	V 1.0.2		I2C address bugfix
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/input/lsm330.h>

#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include <mach/irqs.h>
#include "cust_eint.h"

//#define	DEBUG

#define G_MAX			23920640	/* ug */
#define	I2C_RETRY_DELAY		5		/* Waiting for signals [ms] */
#define	I2C_RETRIES		5		/* Number of retries */
#define	I2C_AUTO_INCREMENT	0x80		/* Autoincrement i2c address */

#define SENSITIVITY_2G		60		/* ug/LSB	*/
#define SENSITIVITY_4G		120		/* ug/LSB	*/
#define SENSITIVITY_6G		180		/* ug/LSB	*/
#define SENSITIVITY_8G		240		/* ug/LSB	*/
#define SENSITIVITY_16G		730		/* ug/LSB	*/

#define	LSM330_ACC_FS_MASK	(0x38)

/* Output Data Rates ODR */
#define	LSM330_ODR_MASK		0XF0
#define LSM330_PM_OFF		0x00		/* OFF */
#define	LSM330_ODR3_125		0x10		/*    3.125 Hz */
#define	LSM330_ODR6_25		0x20		/*    6.25  Hz */
#define	LSM330_ODR12_5		0x30		/*   12.5   Hz */
#define	LSM330_ODR25		0x40		/*   25     Hz */
#define	LSM330_ODR50		0x50		/*   50     Hz */
#define	LSM330_ODR100		0x60		/*  100     Hz */
#define	LSM330_ODR400		0x70		/*  400     Hz */
#define	LSM330_ODR800		0x80		/*  800     Hz */
#define	LSM330_ODR1600		0x90		/* 1600     Hz */

/* Registers configuration Mask and settings */
/* CTRLREG1 */
#define LSM330_INTEN_MASK		0x01
#define LSM330_INTEN_OFF		0x00
#define LSM330_INTEN_ON			0x01

/* CTRLREG2 */
#define LSM330_HIST1_MASK		0xE0
#define LSM330_SM1INT_PIN_MASK		0x08
#define LSM330_SM1INT_PINB		0x08
#define LSM330_SM1INT_PINA		0x00
#define LSM330_SM1_EN_MASK		0x01
#define LSM330_SM1_EN_ON		0x01
#define LSM330_SM1_EN_OFF		0x00
/* */

/* CTRLREG3 */
#define LSM330_HIST2_MASK		0xE0
#define LSM330_SM2INT_PIN_MASK		0x08
#define LSM330_SM2INT_PINB		0x08
#define LSM330_SM2INT_PINA		0x00
#define LSM330_SM2_EN_MASK		0x01
#define LSM330_SM2_EN_ON		0x01
#define LSM330_SM2_EN_OFF		0x00
/* */

/* CTRLREG4 */
#define LSM330_INT_ACT_MASK		(0x01 << 6)
#define LSM330_INT_ACT_H		(0x01 << 6)
#define LSM330_INT_ACT_L		0x00

#define LSM330_INT2_EN_MASK		(0x01 << 4)
#define LSM330_INT2_EN_ON		(0x01 << 4)
#define LSM330_INT2_EN_OFF		0x00

#define LSM330_INT1_EN_MASK		(0x01 << 3)
#define LSM330_INT1_EN_ON		(0x01 << 3)
#define LSM330_INT1_EN_OFF		0x00
/* */

#define	OUT_AXISDATA_REG		LSM330_OUTX_L
#define WHOAMI_LSM330_ACC		0x40	/* Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define	LSM330_WHO_AM_I			0x0F	/* WhoAmI register Address */

#define	LSM330_OUTX_L			0x28	/* Output X LSByte */
#define	LSM330_OUTX_H			0x29	/* Output X MSByte */
#define	LSM330_OUTY_L			0x2A	/* Output Y LSByte */
#define	LSM330_OUTY_H			0x2B	/* Output Y MSByte */
#define	LSM330_OUTZ_L			0x2C	/* Output Z LSByte */
#define	LSM330_OUTZ_H			0x2D	/* Output Z MSByte */
#define	LSM330_LC_L			0x16	/* LSByte Long Counter Status */
#define	LSM330_LC_H			0x17	/* MSByte Long Counter Status */

#define	LSM330_STATUS_REG		0x27	/* Status */

#define	LSM330_CTRL_REG1		0x21	/* control reg 1 */
#define	LSM330_CTRL_REG2		0x22	/* control reg 2 */
#define	LSM330_CTRL_REG3		0x23	/* control reg 3 */
#define	LSM330_CTRL_REG4		0x20	/* control reg 4 */
#define	LSM330_CTRL_REG5		0x24	/* control reg 3 */
#define	LSM330_CTRL_REG6		0x25	/* control reg 4 */

#define	LSM330_OFF_X			0x10	/* Offset X Corr */
#define	LSM330_OFF_Y			0x11	/* Offset Y Corr */
#define	LSM330_OFF_Z			0x12	/* Offset Z Corr */

#define	LSM330_CS_X			0x13	/* Const Shift X */
#define	LSM330_CS_Y			0x14	/* Const Shift Y */
#define	LSM330_CS_Z			0x15	/* Const Shift Z */

#define	LSM330_VFC_1			0x1B	/* Vect Filter Coeff 1 */
#define	LSM330_VFC_2			0x1C	/* Vect Filter Coeff 2 */
#define	LSM330_VFC_3			0x1D	/* Vect Filter Coeff 3 */
#define	LSM330_VFC_4			0x1E	/* Vect Filter Coeff 4 */

/*	end CONTROL REGISTRES	*/


/* RESUME STATE INDICES */
#define	LSM330_RES_LC_L				0
#define	LSM330_RES_LC_H				1

#define	LSM330_RES_CTRL_REG1			2
#define	LSM330_RES_CTRL_REG2			3
#define	LSM330_RES_CTRL_REG3			4
#define	LSM330_RES_CTRL_REG4			5
#define	LSM330_RES_CTRL_REG5			6

#define	LSM330_RES_TIM4_1			20
#define	LSM330_RES_TIM3_1			21
#define	LSM330_RES_TIM2_1_L			22
#define	LSM330_RES_TIM2_1_H			23
#define	LSM330_RES_TIM1_1_L			24
#define	LSM330_RES_TIM1_1_H			25

#define	LSM330_RES_THRS2_1			26
#define	LSM330_RES_THRS1_1			27
#define	LSM330_RES_SA_1				28
#define	LSM330_RES_MA_1				29
#define	LSM330_RES_SETT_1			30

#define	LSM330_RES_TIM4_2			31
#define	LSM330_RES_TIM3_2			32
#define	LSM330_RES_TIM2_2_L			33
#define	LSM330_RES_TIM2_2_H			34
#define	LSM330_RES_TIM1_2_L			35
#define	LSM330_RES_TIM1_2_H			36

#define	LSM330_RES_THRS2_2			37
#define	LSM330_RES_THRS1_2			38
#define	LSM330_RES_DES_2			39
#define	LSM330_RES_SA_2				40
#define	LSM330_RES_MA_2				41
#define	LSM330_RES_SETT_2			42

#define	LSM330_RESUME_ENTRIES			43



#define	LSM330_STATE_PR_SIZE			16
/* end RESUME STATE INDICES */

/* STATE PROGRAMS ENABLE CONTROLS */
#define	LSM330_SM1_DIS_SM2_DIS			0x00
#define	LSM330_SM1_DIS_SM2_EN			0x01
#define	LSM330_SM1_EN_SM2_DIS			0x02
#define	LSM330_SM1_EN_SM2_EN			0x03

/* INTERRUPTS ENABLE CONTROLS */
#define	LSM330_INT1_DIS_INT2_DIS		0x00
#define	LSM330_INT1_DIS_INT2_EN			0x01
#define	LSM330_INT1_EN_INT2_DIS			0x02
#define	LSM330_INT1_EN_INT2_EN			0x03

#if 0
#define pr_debug(format, arg...)   printk(KERN_EMERG format , ## arg)
#define pr_err(format, arg...)   printk(KERN_EMERG format , ## arg)
#define pr_info(format, arg...)   printk(KERN_EMERG format , ## arg)

#define dev_err(dev, format, arg...)   printk(KERN_EMERG format , ## arg)
#define dev_info(dev, format, arg...)   printk(KERN_EMERG format , ## arg)
#define dev_dbg(dev, format, arg...)   printk(KERN_EMERG format , ## arg)
#define dev_warn(dev, format, arg...)   printk(KERN_EMERG format , ## arg)
#define dev_notice(dev, format, arg...)   printk(KERN_EMERG format , ## arg)
#endif

#ifdef LSM330_STEPCOUNTER_EN
u8 pedosm_walking_default[] = {
	/*address, value*/
	0X20,0X67,
	0X23,0X4C,
	0X24,0X8,
	0X25,0X10,
	0X27,0XFF,
	0X10,0X0,
	0X11,0X0,
	0X12,0X0,
	0X13,0X0,
	0X14,0X0,
	0X15,0X0,
	0X16,0XFF,
	0X17,0X7F,
	0X1B,0X7D,
	0X1C,0X72,
	0X1D,0X4C,
	0X1E,0X26,
	0X1F,0X0,
	0X2E,0X0,
	0X19,0X0,
	0X1A,0X0,
	0X50,0X0,
	0X51,0X0,
	0X52,0X30,                //SM1 T2
	0X53,0X0,
	0X54,0X46,                //SM1 T1
	0X55,0X0,
	0X56,0X0,
	0X57,0X4,                 //SM1 THS1 +-4G
	0X59,0X0,
	0X5A,0X3,
	0X5B,0X21,
	0X5C,0X0,
	0X5D,0X22,
	0X5E,0X0,
	0X5F,0X0,
	0X70,0X0,
	0X71,0X00,
	0X72,0X16,                //SM2 T2
	0X73,0X0,
	0X74,0X17,                //SM2 T1
	0X75,0X0,
	0X76,0X0,
	0X77,0X16,                //SM  THS1 +-4G
	0X78,0X0,
	0X79,0X0,
	0X7A,0X3,
	0X7B,0X21,
	0X7C,0X0,
	0X7D,0X10,
	0X7E,0X0,
	0X7F,0X0,
	0X40,0X15,
	0X41,0X2,
	0X42,0X15,
	0X43,0X2,
	0X44,0X15,
	0X45,0X2,
	0X46,0X15,
	0X47,0X2,
	0X48,0XBB,
	0X49,0XBB,
	0X4A,0XBB,
	0X4B,0X15,
	0X4C,0X2,
	0X4D,0X22,
	0X4E,0X57,
	0X4F,0XAA,
	0X60,0X15,                //SM2 S0
	0X61,0X02,                //SM2 S1
	0X62,0X15,                //SM2 S2
	0X63,0X02,                //SM2 S3
	0X64,0X15,                //SM2 S4
	0X65,0X02,                //SM2 S5
	0X66,0X15,                //SM2 S6
	0X67,0X11,                //SM2 S7
	0X68,0X00,                //SM2 S8
	0X69,0X00,                //SM2 S9
	0X6A,0X00,                //SM2 S10
	0X6B,0X00,                //SM2 S11
	0X6C,0X00,                //SM2 S12
	0X6D,0X00,                //SM2 S13
	0X6E,0X00,                //SM2 S14
	0X6F,0X00,                //SM2 S15
	0X23,0X4C,
	0X21,0X1,
	0X22,0X1
};

u8 pedosm_walking_mode[] = {
	/*address, value*/
	0X22,0X00,                //stop the SM2
	0X52,0X30,                //SM1 T2
	0X54,0X46,                //SM1 T1
	0X57,0X04,                //SM1 THS1 +-4G
	0X72,0X16,                //SM2 T2
	0X74,0X17,                //SM2 T1
	0X77,0X16,                //SM  THS1 +-4G
	0X60,0X15,                //SM2 S0
	0X61,0X02,                //SM2 S1
	0X62,0X15,                //SM2 S2
	0X63,0X02,                //SM2 S3
	0X22,0X01                 //re start the SM2

};

u8 pedosm_running_mode[] = {
	/*address, value*/
	0X22,0X00,                //stop the SM2
	0X52,0X17,                //SM1 T2
	0X54,0X32,                //SM1 T1
	0X57,0X08,                //SM1 THS1 +-4G
	0X74,0X32,                //SM2 T1
	0X77,0X10,                //SM  THS1 +-4G
	0X60,0X51,                //SM2 S0
	0X61,0X01,                //SM2 S1
	0X62,0X51,                //SM2 S2
	0X63,0X11,                //SM2 S3
	0X22,0X01                 //re start the SM2

};
#endif

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lsm330_acc_odr_table[] = {
		{    1, LSM330_ODR1600 },
		{    3, LSM330_ODR400  },
		{   10, LSM330_ODR100  },
		{   20, LSM330_ODR50   },
		{   40, LSM330_ODR25   },
		{   80, LSM330_ODR12_5 },
		{  160, LSM330_ODR6_25 },
		{  320, LSM330_ODR3_125},
};

static const struct lsm330_acc_platform_data default_lsm330_acc_pdata = {
	.fs_range = LSM330_ACC_G_4G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 1,
	.negate_z = 1,
	.poll_interval = 100,
	.min_interval = LSM330_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LSM330_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM330_ACC_DEFAULT_INT2_GPIO,
};

struct lsm330_acc_data {
	struct i2c_client *client;
	struct lsm330_acc_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	atomic_t sensor_phone_calling;
	struct wake_lock sensor_suspend_lock;
	struct timer_list sensor_phone_calling_timer;
	int on_before_suspend;

	u16 sensitivity;
	u8 stateprogs_enable_setting;
	u8 interrupts_enable_setting;

	u8 resume_state[LSM330_RESUME_ENTRIES];
	u8 resume_stmach_program1[LSM330_STATE_PR_SIZE];
	u8 resume_stmach_program2[LSM330_STATE_PR_SIZE];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	int calib_data[3];

#ifdef LSM330_STEPCOUNTER_EN
	/* 0 --- walking mode, 1 --- running mode */
	int stepcounter_mode;
	struct input_dev *step_idev;
	struct delayed_work step_iwork;
	atomic_t step_enabled;
	unsigned int step_poll_interval;
#endif

#ifdef DEBUG
	u8 reg_addr;
#endif
};


/* sets default init values to be written in registers at probe stage */
static void lsm330_acc_set_init_register_values(struct lsm330_acc_data *acc)
{
	/* 16-bit long-counter register for interrupt state machine programs timing */
	acc->resume_state[LSM330_RES_LC_L] = 0x00;
	acc->resume_state[LSM330_RES_LC_H] = 0x00;

	acc->resume_state[LSM330_RES_CTRL_REG1] = LSM330_INT_ACT_H;
	acc->resume_state[LSM330_RES_CTRL_REG2] = 0x00;
	acc->resume_state[LSM330_RES_CTRL_REG3] = 0x00;
	acc->resume_state[LSM330_RES_CTRL_REG4] = 0x8f;
	acc->resume_state[LSM330_RES_CTRL_REG5] = 0x00;
}

static int lsm330_acc_i2c_read(struct lsm330_acc_data *acc,
				u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm330_acc_i2c_write(struct lsm330_acc_data *acc, u8 * buf,
								int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm330_acc_i2c_update(struct lsm330_acc_data *acc,
				u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 rdbuf[1] = { reg_address };
	u8 wrbuf[2] = { reg_address , 0x00 };

	u8 init_val;
	u8 updated_val;
	err = lsm330_acc_i2c_read(acc, rdbuf, 1);
	if (!(err < 0)) {
		init_val = rdbuf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		wrbuf[1] = updated_val;
		err = lsm330_acc_i2c_write(acc, wrbuf, 1);
	}
	return err;
}

static int lsm330_acc_hw_init(struct lsm330_acc_data *acc)
{
	int err = -1;
	u8 buf[17];

	pr_info("%s: hw init start\n", LSM330_ACC_DEV_NAME);

	buf[0] = LSM330_WHO_AM_I;
	err = lsm330_acc_i2c_read(acc, buf, 1);
	if (err < 0) {
	dev_warn(&acc->client->dev, "Error reading WHO_AM_I: is device "
		"available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;

	if (buf[0] != WHOAMI_LSM330_ACC) {
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x,"
		" Replies: 0x%x\n", WHOAMI_LSM330_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG6);
	buf[1] = 0x10;
	err = lsm330_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_LC_L);
	buf[1] = acc->resume_state[LSM330_RES_LC_L];
	buf[2] = acc->resume_state[LSM330_RES_LC_H];
	err = lsm330_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG2);
	buf[1] = acc->resume_state[LSM330_RES_CTRL_REG2];
	buf[2] = acc->resume_state[LSM330_RES_CTRL_REG3];
	err = lsm330_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG4);
	buf[1] = acc->resume_state[LSM330_RES_CTRL_REG4];
	buf[2] = acc->resume_state[LSM330_RES_CTRL_REG1];
	err = lsm330_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	pr_info("%s: hw init done\n", LSM330_ACC_DEV_NAME);
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

#ifdef LSM330_STEPCOUNTER_EN

static int pedosm_mode_switch(struct lsm330_acc_data *acc)
{
	int size, i, ret;
	u8 *buf;
	u8 msg[2];

	mutex_lock(&acc->lock);
	if (acc->stepcounter_mode == 0) {
		acc->stepcounter_mode = 1;
		buf = pedosm_running_mode;
		dev_dbg(&acc->client->dev, "pedosm switch to running mode\n");
	} else {
		acc->stepcounter_mode = 0;
		buf = pedosm_walking_mode;
		dev_dbg(&acc->client->dev, "pedosm switch to walking mode\n");
	}
	mutex_unlock(&acc->lock);

	size = sizeof(buf) / (sizeof(u8) * 2);
	for(i = 0; i < size; i++) {
		msg[0] = buf[i * 2]; // address
		msg[1] = buf[i * 2 + 1]; // value
		ret = lsm330_acc_i2c_write(acc, msg, 1);
		if (ret < 0) {
			dev_err(&acc->client->dev, "i2c write error add: 0x%x, val:0x%x\n",
				buf[i * 2], buf[i * 2 + 1]);
			return -1;
		}
	}

	return 0;
}

static int lsm330_acc_stepcounter_init(struct lsm330_acc_data *acc)
{
	int i, size, ret = 0;
	u8 buf[2];

	dev_dbg(&acc->client->dev, "step counter init\n");
	//default walking mode
	acc->stepcounter_mode = 0;
	size = sizeof(pedosm_walking_default) / (sizeof(u8) * 2);
	for(i = 0; i < size; i++) {
		buf[0] = pedosm_walking_default[i * 2]; // address
		buf[1] = pedosm_walking_default[i * 2 + 1]; // value
		ret = lsm330_acc_i2c_write(acc, buf, 1);
	}

	return ret;
}

#endif

static void lsm330_acc_device_power_off(struct lsm330_acc_data *acc)
{
	int err;

	dev_dbg(&acc->client->dev, "lsm330 acc device power off\n");

	err = lsm330_acc_i2c_update(acc, LSM330_CTRL_REG4,
					LSM330_ODR_MASK, LSM330_PM_OFF);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);
	if (acc->hw_initialized) {
		acc->hw_initialized = 0;
	}
#if 1
	// disable step counter interrupt
	mt_eint_mask(CUST_EINT_GYRO_NUM);
#endif
}

static int lsm330_acc_device_power_on(struct lsm330_acc_data *acc)
{
	int err = -1;

	dev_dbg(&acc->client->dev, "lsm330 acc device power on\n");

	if (!acc->hw_initialized) {
		err = lsm330_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			lsm330_acc_device_power_off(acc);
			return err;
		}
	}
#if 1
	mt_eint_unmask(CUST_EINT_GYRO_NUM);
#endif
#ifdef LSM330_STEPCOUNTER_EN
	lsm330_acc_stepcounter_init(acc);
#endif
	return 0;
}

struct lsm330_acc_data *g_acc_data;
static void lsm330_acc_isr1(void)
{
	struct lsm330_acc_data *acc = g_acc_data;

	dev_dbg(&acc->client->dev, "lsm330_acc_isr1 enter\n");

	if (acc->irq1_work_queue != NULL) {
		dev_info(&acc->client->dev, "m330_acc_isr1 queued\n");
		queue_work(acc->irq1_work_queue, &acc->irq1_work);
	}

	dev_dbg(&acc->client->dev, "lsm330_acc_isr1 leave\n");
}

static void lsm330_acc_irq1_work_func(struct work_struct *work)
{

	struct lsm330_acc_data *acc;
	acc = container_of(work, struct lsm330_acc_data, irq1_work);

#ifdef LSM330_STEPCOUNTER_EN
	pedosm_mode_switch(acc);
#endif
	pr_info("%s: IRQ1 triggered\n", LSM330_ACC_DEV_NAME);
#if 1
	mt_eint_unmask(CUST_EINT_GYRO_NUM);
#endif
}

static int lsm330_acc_register_masked_update(struct lsm330_acc_data *acc,
		u8 reg_address, u8 mask, u8 new_bit_values, int resume_index)
{
	u8 config[2] = {0};
	u8 init_val, updated_val;
	int err;
	int step = 0;

	config[0] = reg_address;
	err = lsm330_acc_i2c_read(acc, config, 1);
	if (err < 0)
		goto error;
	init_val = config[0];
	init_val = acc->resume_state[resume_index];
	step = 1;
	updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
	config[0] = reg_address;
	config[1] = updated_val;
	err = lsm330_acc_i2c_write(acc, config, 1);
	if (err < 0)
		goto error;
	acc->resume_state[resume_index] = updated_val;

	return err;
	error:
		dev_err(&acc->client->dev,
			"register 0x%x update failed at step %d, error: %d\n",
				config[0], step, err);
	return err;
}

static int lsm330_acc_update_fs_range(struct lsm330_acc_data *acc,
								u8 new_fs_range)
{
	int err=-1;
	u16 sensitivity;

	switch (new_fs_range) {
	case LSM330_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case LSM330_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case LSM330_ACC_G_6G:
		sensitivity = SENSITIVITY_6G;
		break;
	case LSM330_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	case LSM330_ACC_G_16G:
		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid g range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Updates configuration register 1,
		* which contains g range setting */
		err = lsm330_acc_register_masked_update(acc, LSM330_CTRL_REG5,
			LSM330_ACC_FS_MASK, new_fs_range, LSM330_RES_CTRL_REG5);
		if(err < 0) {
			dev_err(&acc->client->dev, "update g range failed\n");
			return err;
		}
		else
			acc->sensitivity = sensitivity;
	}

	if(err < 0)
		dev_err(&acc->client->dev, "update g range not executed "
						"because the device is off\n");
	return err;
}


static int lsm330_acc_update_odr(struct lsm330_acc_data *acc,
							int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 new_odr;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lsm330_acc_odr_table) - 1; i >= 0; i--) {
		if (lsm330_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	new_odr = lsm330_acc_odr_table[i].mask;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	/*
	if (atomic_read(&acc->enabled)) {
		err = lsm330_acc_register_masked_update(acc,
			LSM330_CTRL_REG4, LSM330_ODR_MASK, new_odr,
							LSM330_RES_CTRL_REG4);
	}
	*/
	err = 0;

	if(err < 0)
		dev_err(&acc->client->dev, "update odr failed\n");
	return err;
}


#ifdef DEBUG
static int lsm330_acc_register_write(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = lsm330_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	return err;
}

static int lsm330_acc_register_read(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = lsm330_acc_i2c_read(acc, buf, 1);
	return err;
}

static int lsm330_acc_register_update(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = lsm330_acc_register_read(acc, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lsm330_acc_register_write(acc, buf, reg_address,
				updated_val);
	}
	return err;
}
#endif

static int lsm330_acc_get_raw_data(struct lsm330_acc_data *acc,
		int *xyz)
{
	int err = 0;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data_l[1];
	u8 acc_data_h[1];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };

	/* when we in selftest mode, multi bytes read is not avaliable */

	acc_data_l[0] = LSM330_OUTX_L;
	err |= lsm330_acc_i2c_read(acc, acc_data_l, 1);
	acc_data_h[0] = LSM330_OUTX_H;
	err |= lsm330_acc_i2c_read(acc, acc_data_h, 1);
	hw_d[0] = ((s16) ((acc_data_h[0] << 8) | acc_data_l[0]));


	acc_data_l[0] = LSM330_OUTY_L;
	err |= lsm330_acc_i2c_read(acc, acc_data_l, 1);
	acc_data_h[0] = LSM330_OUTY_H;
	err |= lsm330_acc_i2c_read(acc, acc_data_h, 1);
	hw_d[1] = ((s16) ((acc_data_h[0] << 8) | acc_data_l[0]));


	acc_data_l[0] = LSM330_OUTZ_L;
	err |= lsm330_acc_i2c_read(acc, acc_data_l, 1);
	acc_data_h[0] = LSM330_OUTZ_H;
	err |= lsm330_acc_i2c_read(acc, acc_data_h, 1);
	hw_d[2] = ((s16) ((acc_data_h[0] << 8) | acc_data_l[0]));


	xyz[0] = hw_d[0];
	xyz[1] = hw_d[1];
	xyz[2] = hw_d[2];

	return err;
}

static int lsm330_acc_get_acceleration_data(struct lsm330_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };

	acc_data[0] = (I2C_AUTO_INCREMENT | OUT_AXISDATA_REG);
	err = lsm330_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]));
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]));
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]));

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;


	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));

#ifdef DEBUG
//	pr_info("%s read x=%d, y=%d, z=%d\n",
//			LSM330_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
#endif
	return err;
}

static void lsm330_acc_report_values(struct lsm330_acc_data *acc,
					int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, xyz[0] - acc->calib_data[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1] - acc->calib_data[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2] - acc->calib_data[2]);
	input_sync(acc->input_dev);
}

static int lsm330_acc_enable(struct lsm330_acc_data *acc)
{
	int err;

	dev_dbg(&acc->client->dev, "lsm330 acc enable\n");

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = lsm330_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		schedule_delayed_work(&acc->input_work,
			msecs_to_jiffies(acc->pdata->poll_interval));
	}

	return 0;
}

static int lsm330_acc_disable(struct lsm330_acc_data *acc)
{

	dev_dbg(&acc->client->dev, "lsm330 acc enable\n");

	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		lsm330_acc_device_power_off(acc);
	}

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
					struct device_attribute *attr,
								char *buf)
{
	int val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	dev_dbg(&acc->client->dev, "get poll rate: %d", val);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	dev_dbg(&acc->client->dev, "set poll rate: %s", buf);

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	err = lsm330_acc_update_odr(acc, interval_ms);
	if(err >= 0)
	{
		acc->pdata->poll_interval = interval_ms;
	}
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int range = 2;
	mutex_lock(&acc->lock);
	val = acc->pdata->fs_range ;
	switch(val) {
	case LSM330_ACC_G_2G:
		range = 2;
		break;
	case LSM330_ACC_G_4G:
		range = 4;
		break;
	case LSM330_ACC_G_6G:
		range = 6;
		break;
	case LSM330_ACC_G_8G:
		range = 8;
		break;
	case LSM330_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch(val) {
		case 2:
			range = LSM330_ACC_G_2G;
			break;
		case 4:
			range = LSM330_ACC_G_4G;
			break;
		case 6:
			range = LSM330_ACC_G_6G;
			break;
		case 8:
			range = LSM330_ACC_G_8G;
			break;
		case 16:
			range = LSM330_ACC_G_16G;
			break;
		default:
			return -1;
	}

	mutex_lock(&acc->lock);
	err = lsm330_acc_update_fs_range(acc, range);
	if(err >= 0)
	{
		acc->pdata->fs_range = range;
	}
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm330_acc_enable(acc);
	else
		lsm330_acc_disable(acc);

	return size;
}

static ssize_t attr_get_sensor_phone_calling(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->sensor_phone_calling);
	return sprintf(buf, "%d\n", val);
}

void sensor_phone_calling_timer_fn(unsigned long data)
{
	struct lsm330_acc_data *acc = data;

	if (atomic_read(&acc->sensor_phone_calling)) {
		acc->sensor_phone_calling_timer.expires = jiffies + HZ*1/5;
		add_timer(&acc->sensor_phone_calling_timer);
	}
}
static ssize_t attr_set_sensor_phone_calling(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val) {
		if (!atomic_cmpxchg(&acc->sensor_phone_calling, 0, 1)) {
			/* we need a timer to wake up the phone */
			acc->sensor_phone_calling_timer.function = &sensor_phone_calling_timer_fn;
			acc->sensor_phone_calling_timer.expires  = jiffies + HZ*1/5;
			acc->sensor_phone_calling_timer.data     = acc;
			add_timer(&acc->sensor_phone_calling_timer);
			//wake_lock(&acc->sensor_suspend_lock);
		}
	} else {
		if (atomic_cmpxchg(&acc->sensor_phone_calling, 1, 0)) {
			del_timer(&acc->sensor_phone_calling_timer);
			//wake_unlock(&acc->sensor_suspend_lock);
		}
	}

	return size;
}

/* selftest start */
#define SELFTEST_MEASURE_TIMEOUT 100
#define SELFTEST_ZYXDA (0x1 << 3)
#define SELFTEST_SAMPLES 5

static int selftest_init(struct lsm330_acc_data *acc)
{
	unsigned char buf[5];
	//	pr_info("%s\n", __func__);

	/* BDU=1, ODR=200Hz, FS=+/-2G */
	buf[0] = 0x20;
	buf[1] = 0x6F;
	lsm330_acc_i2c_write(acc, buf, 1);
	pr_debug("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x24;
	buf[1] = 0xC0;
	lsm330_acc_i2c_write(acc, buf, 1);
	pr_debug("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

#if 1
	buf[0] = 0x23;
	buf[1] = 0x00;
	lsm330_acc_i2c_write(acc, buf, 1);
	pr_debug("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x25;
	buf[1] = 0x00;
	lsm330_acc_i2c_write(acc, buf, 1);
	pr_debug("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);
 #endif
 	return 0;
}

static int selftest_enable(struct lsm330_acc_data *acc)
{
	unsigned char buf[2];

	buf[0] = 0x24;
	buf[1] = 0xC2;	/* BDU=1, ST1 = 1, ST0 = 0 */

	pr_debug("%s\n", __func__);
	pr_debug("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);
	return lsm330_acc_i2c_write(acc, buf, 1);
}

static void selftest_disable(struct lsm330_acc_data *acc)
{
	unsigned char buf[2];

	buf[1] = 0x00;
	pr_debug("%s\n", __func__);

	/* Disable sensor */
	buf[0] = 0x20;
	lsm330_acc_i2c_write(acc, buf, 1);
	pr_debug("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	/* Disable selftest */
	buf[0] = 0x24;
	lsm330_acc_i2c_write(acc, buf, 1);
	pr_debug("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	if (atomic_read(&acc->enabled)) {
		lsm330_acc_device_power_on(acc);
	}
}

static int selftest_wait_ZYXDA(struct lsm330_acc_data *acc)
{
	int i, ret;
	unsigned char data_ready;

	pr_debug("%s\n", __func__);

	for (i = SELFTEST_MEASURE_TIMEOUT; i != 0; i--) {
		data_ready = LSM330_STATUS_REG;
		ret = lsm330_acc_i2c_read(acc, &data_ready, 1);
		if (ret < 0) {
			pr_err("%s: lsm330_acc_i2c_read fail, retry %d\n",
					__func__, i);
			msleep(I2C_RETRY_DELAY);
			continue;
		} else if (data_ready & SELFTEST_ZYXDA) {
			pr_debug("%s: data ready\n", __func__);
			break;
		}
	}
	if (i == 0) {
		pr_err("%s: failed\n", __func__);
		return ret;
	}

	return 0;
}

static int selftest_read(struct lsm330_acc_data *acc, int data[3])
{
	int total[3];
	int i, ret;

	pr_debug("%s\n", __func__);

	total[0] = 0;
	total[1] = 0;
	total[2] = 0;

	pr_debug("%s: Read OUTX/OUTY/OUTZ to clear ZYXDA bit\n", __func__);
	lsm330_acc_get_raw_data(acc, data);
	for (i = 0; i < SELFTEST_SAMPLES; i++) {
		ret = selftest_wait_ZYXDA(acc);
		if (ret) {
			pr_err("%s: selftest_check_ZYXDA fail\n", __func__);
			return ret;
		}
		mutex_lock(&acc->lock);
		ret = lsm330_acc_get_raw_data(acc, data);
		mutex_unlock(&acc->lock);
		if (ret < 0) {
			pr_err("%s: lsm330_acc_get_raw_data fail\n", __func__);
			return ret;
		}
		/* convert to mg */
		data[0] = data[0] * 61 / 1000;
		data[1] = data[1] * 61 / 1000;
		data[2] = data[2] * 61 / 1000;
		pr_debug("%s: data: x = %d, y = %d, z = %d\n", __func__,
				data[0], data[1], data[2]);
		total[0] += data[0];
		total[1] += data[1];
		total[2] += data[2];
		pr_debug("%s: total: x = %d, y = %d, z = %d\n", __func__,
				total[0], total[1], total[2]);
	}
	data[0] = total[0] / SELFTEST_SAMPLES;
	data[1] = total[1] / SELFTEST_SAMPLES;
	data[2] = total[2] / SELFTEST_SAMPLES;
	pr_debug("%s: average: x = %d, y = %d, z = %d\n", __func__,
			data[0], data[1], data[2]);

	return 0;
}

/*
 * Part Number Min_X Max_X Min_Y Max_Y Min_Z Max_Z Unit
 * LSM330      60    1700  60    1700  60    1700  LSB (@ FS = +/-2g,2.8v)
 */

static int check_selftest_result(int data_nost[3],
		int data_st[3])
{
	pr_debug("data_st:   %d %d %d\n", data_st[0],data_st[1],data_st[2]);
	pr_debug("data_nost: %d %d %d\n", data_nost[0],data_nost[1],data_nost[2]);
	data_st[0] = abs(data_st[0] - data_nost[0]);
	data_st[1] = abs(data_st[1] - data_nost[1]);
	data_st[2] = abs(data_st[2] - data_nost[2]);
	pr_debug("abs(st - nost):  %d %d %d\n", data_st[0],data_st[1],data_st[2]);

	if(data_st[0] >= 60 && data_st[0] <= 1700){
		pr_debug("expect 60 =< x <= 1700, x = %d selftest pass\n", data_st[0]);
	}

	if(data_st[1] >= 60 && data_st[1] <= 1700){
		pr_debug("expect 60 =< y <= 1700, y = %d selftest pass\n", data_st[1]);
	}

	if(data_st[2] >= 60 && data_st[2]<= 1700){
		pr_debug("expect 60 =< z <= 1400, z = %d selftest pass\n",data_st[2]);
	}

	if (data_st[0] >= 60 && data_st[0] <= 1700 && 
	    data_st[1] >= 60 && data_st[1] <= 1700 && 
	    data_st[2] >= 60 && data_st[2] <= 1700) {
		return 1;
	}

	return -1;
}

static int lsm330_acc_selftest(struct lsm330_acc_data *acc, int *test_result)
{
	int ret;
	int data_nost[3];
	int self_data[3];

	*test_result = 0;

	pr_info("%s: =========================\n", __func__);
	pr_info("%s: lsm330_acc_selftest begin\n", __func__);

	/* Initialize Sensor, turn on sensor, enable P/R/Y */
	ret = selftest_init(acc);
	if (ret < 0) {
		pr_err("%s: selftest_init fail\n", __func__);
		return ret;
	}

	/* Wait for stable output */
	pr_debug("%s: wait 80ms for stable output\n", __func__);
	msleep(80);

	/* Read out normal output */
	ret = selftest_read(acc, data_nost);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}
	pr_debug("%s: normal output: x = %d, y = %d, z = %d\n",
			__func__, data_nost[0], data_nost[1], data_nost[2]);

	/* Enable self test */
	ret = selftest_enable(acc);
	if (ret < 0) {
		pr_err("%s: selftest_enable failed\n", __func__);
		return ret;
	}

	pr_debug("%s: wait 80ms for stable output\n", __func__);
	mdelay(80);

	/* Read out selftest output */
	ret = selftest_read(acc, self_data);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}

	/* Check output */
	ret = check_selftest_result(data_nost, self_data);
	if (ret < 0) {
		pr_err("%s: ***fail***\n", __func__);
		*test_result = 0;
	} else {
		pr_info("%s: ***success***\n", __func__);
		*test_result = 1;
	}

	/* selftest disable */
	selftest_disable(acc);

	pr_info("%s: lsm330_acc_selftest end\n\n\n\n", __func__);
	return ret;
}

static ssize_t attr_self_test_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i;
	int ret;
	int self_test;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
#if 1
	for (i=1; i>0; i--) {
		ret = lsm330_acc_selftest(acc, &self_test);
		if (ret < 0 || self_test==0) {
			dev_err(dev, "failed to perform acc self test\n");
			break;
		}
	}

#else
	self_test = 1;
#endif
	return sprintf(buf, "%d\n", self_test);
}

/*calibrate*/
/*lsm330 calibration*/
#define CALIB_DATA_AMOUNT 100
static int lsm330_acc_calibration(struct lsm330_acc_data *acc, bool do_calib)
{
	int xyz[3] = {0,};
	int sum[3] = {0,};
	int err = 0;
	int i;

	if (do_calib) {
		lsm330_acc_update_odr(acc, 10);
		for (i =0; i < CALIB_DATA_AMOUNT; i++) {
			mutex_lock(&acc->lock);
			err = lsm330_acc_get_acceleration_data(acc, xyz);
			mutex_unlock(&acc->lock);
			msleep(10);
			if (err < 0) {
				dev_err(&acc->client->dev, "get_acceleration_data failed\n");
				return err;
			}

			sum[0] += xyz[0];
			sum[1] += xyz[1];
			sum[2] += xyz[2];
		}

		acc->calib_data[0] = sum[0] / CALIB_DATA_AMOUNT;
		acc->calib_data[1] = sum[1] / CALIB_DATA_AMOUNT;
		// 1000000ug = 9.86 m/s^2
		acc->calib_data[2] = sum[2] / CALIB_DATA_AMOUNT - 1000000;
		lsm330_acc_update_odr(acc, acc->pdata->poll_interval);
	} else {
		acc->calib_data[0] = 0;
		acc->calib_data[1] = 0;
		acc->calib_data[2] = 0;
	}

	pr_info("%s: calibration data (%d,%d,%d)\n",
			__func__, acc->calib_data[0], acc->calib_data[1],
			acc->calib_data[2]);
	return err;
}

static ssize_t attr_set_calibration(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	bool do_calib;
	int err = 0;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		do_calib = true;
	else
		do_calib = false;

	/*if off , before the calibration, we should turn on it*/
	if (!atomic_read(&acc->enabled)) {
		err = lsm330_acc_device_power_on(acc);
		if (err < 0) {
			pr_err("%s: lsm330_acc_device_power_on fail\n",
					__func__);
			return err;
		}
		//atomic_set(&acc->enabled, 1);
	}

	lsm330_acc_calibration(acc, do_calib);
	if (err < 0) {
		dev_err(&acc->client->dev, "lsm330_acc_calibration failed\n");
		return err;
	}

	return size;
}

static ssize_t attr_get_calibration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);

	return sprintf(buf, "%d %d %d\n", acc->calib_data[0],
			acc->calib_data[1], acc->calib_data[2]);
}

static ssize_t attr_set_calibvalue(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int ret;
	int calib_data[3];

	if (buf == NULL)
		return -EINVAL;

	ret = sscanf(buf, "%d %d %d", &calib_data[0],
		&calib_data[1], &calib_data[2]);
	if (ret != 3) {
		pr_err("wrong format of calibration data\n");
		return -EINVAL;
	}

	acc->calib_data[0] = calib_data[0];
	acc->calib_data[1] = calib_data[1];
	acc->calib_data[2] = calib_data[2];

	pr_info("x = %d, y = %d, z= %d\n", acc->calib_data[1],
			acc->calib_data[1], acc->calib_data[2]);

	return size;
}

static ssize_t attr_get_calibvalue(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);

	return sprintf(buf, "%d %d %d\n", acc->calib_data[0],
			acc->calib_data[1], acc->calib_data[2]);
}

#ifdef LSM330_STEPCOUNTER_EN

static ssize_t attr_get_reg_dump(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, cnt = 0;
	u8 tmp;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);

	for (i = 0xf; i < 0x7f; i++)
	{
		tmp = i;
		lsm330_acc_i2c_read(acc, &tmp, 1);
		cnt += sprintf(buf + cnt, "%x: %x\n", i, tmp);
	}
	return cnt;
}

static ssize_t attr_get_stepcounter(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	u8 data[2];
	int ret;

	data[0] = 0x16 | I2C_AUTO_INCREMENT;
	ret = lsm330_acc_i2c_read(acc, data, 2);
	printk("step counter raw data %x %x\n", data[1], data[0]);
	if (ret < 0) {
		printk("step counter read error\n");
		return ret;
	} else {
		return sprintf(buf, "%d\n", (data[1] << 8) | data[0]);
	}
}


static ssize_t attr_get_pedo_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int state, ret;

	mutex_lock(&acc->lock);
	state = acc->stepcounter_mode;
	mutex_unlock(&acc->lock);

	if (state == 0)
		ret = sprintf(buf, "walking\n");
	else
		ret = sprintf(buf, "running\n");
	return ret;
}

static ssize_t attr_set_pedo_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);

	pedosm_mode_switch(acc);

	return size;
}

static ssize_t attr_get_step_polling_rate(struct device *dev,
					struct device_attribute *attr,
								char *buf)
{
	int val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	dev_dbg(&acc->client->dev, "get_step_polling_rate ");
	mutex_lock(&acc->lock);
	val = acc->step_poll_interval;
	mutex_unlock(&acc->lock);
	dev_dbg(&acc->client->dev, "rate: %d\n", val);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_step_polling_rate(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	dev_dbg(&acc->client->dev, "set_step_polling_rate, rate: %s\n", buf);

	if (strict_strtoul(buf, 10, &interval_ms)) {
		dev_err(&acc->client->dev, "set_step_polling_rate error: write data format error, %s\n", buf);
		return -EINVAL;
	}
	if (!interval_ms) {
		dev_err(&acc->client->dev, "set_step_polling_rate error: rate cannot be 0\n");
		return -EINVAL;
	}

	mutex_lock(&acc->lock);

	dev_dbg(&acc->client->dev, "set_step_polling_rate %ld\n", interval_ms);
	acc->step_poll_interval = interval_ms;

	mutex_unlock(&acc->lock);
	return size;
}



static int lsm330_step_enable(struct lsm330_acc_data *acc)
{

	dev_dbg(&acc->client->dev, "lsm330_step_enable\n");

	if (!atomic_cmpxchg(&acc->step_enabled, 0, 1)) {
		dev_dbg(&acc->client->dev, "schedule step work\n");
		schedule_delayed_work(&acc->step_iwork,
			msecs_to_jiffies(acc->step_poll_interval));
	}

	return 0;
}

static int lsm330_step_disable(struct lsm330_acc_data *acc)
{
	dev_dbg(&acc->client->dev, "lsm330_step_disable\n");

	if (atomic_cmpxchg(&acc->step_enabled, 1, 0)) {
		dev_dbg(&acc->client->dev, "cancel step work\n");
		cancel_delayed_work_sync(&acc->step_iwork);
	}

	return 0;
}

static ssize_t attr_get_step_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->step_enabled);

	dev_dbg(&acc->client->dev, "get step enable: %d\n", val);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_step_enable(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val)) {
		dev_err(&acc->client->dev, "set step enable error, %s\n", buf);
		return -EINVAL;
	}

	dev_dbg(&acc->client->dev, "set step enable: %ld\n", val);

	if (val)
		lsm330_step_enable(acc);
	else
		lsm330_step_disable(acc);

	return size;
}

#endif /* end of #if LSM330_STEPCOUNTER_EN */

static int lsm330_acc_interrupt_enable_control(struct lsm330_acc_data *acc,
								u8 settings)
{
	u8 val1;
	u8 val2 = LSM330_INTEN_ON;
	u8 mask1 = (LSM330_INT1_EN_MASK | LSM330_INT2_EN_MASK);
	int err = -1;
	settings = settings & 0x03;

	switch ( settings ) {
	case LSM330_INT1_DIS_INT2_DIS:
		val1 = (LSM330_INT1_EN_OFF | LSM330_INT2_EN_OFF);
		val2 = LSM330_INTEN_OFF;
		break;
	case LSM330_INT1_DIS_INT2_EN:
		val1 = (LSM330_INT1_EN_OFF | LSM330_INT2_EN_ON);
		break;
	case LSM330_INT1_EN_INT2_DIS:
		val1 = (LSM330_INT1_EN_ON | LSM330_INT2_EN_OFF);
		break;
	case LSM330_INT1_EN_INT2_EN:
		val1 = ( LSM330_INT1_EN_ON | LSM330_INT2_EN_ON);
		break;
	default :
		pr_err("invalid interrupt setting : 0x%02x\n",settings);
		return err;
	}
	err = lsm330_acc_register_masked_update(acc,
		LSM330_CTRL_REG3, mask1, val1, LSM330_RES_CTRL_REG3);
	if (err < 0 )
		return err;

	err = lsm330_acc_register_masked_update(acc,
		LSM330_CTRL_REG1, LSM330_INTEN_MASK, val2,
							LSM330_RES_CTRL_REG1);
	if (err < 0 )
			return err;
	acc->interrupts_enable_setting = settings;
#ifdef DEBUG
	pr_err("interrupt setting : 0x%02x\n",acc->interrupts_enable_setting);
#endif
	return err;
}

static ssize_t attr_get_interr_enable(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	u8 val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->interrupts_enable_setting;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_set_interr_enable(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	int err = -1;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	long val=0;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;


	if ( val < 0x00 || val > LSM330_INT1_EN_INT2_EN){
#ifdef DEBUG
		pr_err("invalid interrupt setting, val: %d\n",val);
#endif
		return -EINVAL;
	}

	mutex_lock(&acc->lock);
	err = lsm330_acc_interrupt_enable_control(acc, val);
	mutex_unlock(&acc->lock);
	if (err < 0)
		return err;
	return size;
}

#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = lsm330_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = lsm330_acc_i2c_read(acc, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0664, attr_get_polling_rate,
							attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
	__ATTR(sensor_phone_calling, 0664, attr_get_sensor_phone_calling, attr_set_sensor_phone_calling),
	__ATTR(enable_interrupt_output, 0664, attr_get_interr_enable,
							attr_set_interr_enable),
	__ATTR(self_test, S_IRUGO, attr_self_test_show, NULL),
	__ATTR(calibration, 0664, attr_get_calibration, attr_set_calibration),
	__ATTR(calib_value, 0664, attr_get_calibvalue, attr_set_calibvalue),

#ifdef LSM330_STEPCOUNTER_EN
	__ATTR(stepcounter, 0664, attr_get_stepcounter, NULL),
	__ATTR(pedo_mode, 0664, attr_get_pedo_mode, attr_set_pedo_mode),
	__ATTR(reg_dump, 0664, attr_get_reg_dump, NULL),
	__ATTR(step_enable_device, 0664, attr_get_step_enable, attr_set_step_enable),
	__ATTR(step_pollrate_ms, 0664, attr_get_step_polling_rate,
							attr_set_step_polling_rate),
#endif

#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void lsm330_acc_input_work_func(struct work_struct *work)
{
	struct lsm330_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work, struct lsm330_acc_data, input_work);

	mutex_lock(&acc->lock);
	err = lsm330_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		lsm330_acc_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
			acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

int lsm330_acc_input_open(struct input_dev *input)
{
	struct lsm330_acc_data *acc = input_get_drvdata(input);

	return lsm330_acc_enable(acc);
}

void lsm330_acc_input_close(struct input_dev *dev)
{
	struct lsm330_acc_data *acc = input_get_drvdata(dev);

	lsm330_acc_disable(acc);
}

static int lsm330_acc_validate_pdata(struct lsm330_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
		acc->pdata->axis_map_y > 2 ||
		 acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", acc->pdata->axis_map_x,
				acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", acc->pdata->negate_x,
				acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm330_acc_input_init(struct lsm330_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, lsm330_acc_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocation failed\n");
		goto err0;
	}
#if 0
	acc->input_dev->open = lsm330_acc_input_open;
	acc->input_dev->close = lsm330_acc_input_close;
#endif
	acc->input_dev->name = LSM330_ACC_DEV_NAME;

	acc->input_dev->id.bustype = BUS_I2C;
	acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptA sources data if the case */
//input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
//input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);


	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input device %s\n",
				acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

#ifdef LSM330_STEPCOUNTER_EN

static int lsm330_get_step_data(struct lsm330_acc_data *acc)
{
	u8 data[2];
	int ret;

	data[0] = 0x16 | I2C_AUTO_INCREMENT;
	ret = lsm330_acc_i2c_read(acc, data, 2);

	dev_dbg(&acc->client->dev, "step counter raw data %x %x\n", data[1], data[0]);

	if (ret < 0) {
		dev_err(&acc->client->dev,
				"lsm330_get_step_data i2c read error\n");
	} else {
		ret = 32767 - ((data[1] << 8) | data[0]);
		dev_dbg(&acc->client->dev, "step counter: %d\n", ret);
	}

	return ret;
}

static void lsm330_step_input_work_func(struct work_struct *work)
{
	struct lsm330_acc_data *acc;

	int step = 0;

	acc = container_of((struct delayed_work *)work,
		struct lsm330_acc_data, step_iwork);

	dev_dbg(&acc->client->dev, "lsm330_step_input_work_func enter\n");

	mutex_lock(&acc->lock);
	step = lsm330_get_step_data(acc);
	if (step < 0)
		dev_err(&acc->client->dev, "lsm330_get_step_data failed\n");
	else{
		dev_dbg(&acc->client->dev, "report step: %d\n", step);
		input_report_abs(acc->step_idev, ABS_MISC, step);
		input_sync(acc->step_idev);
	}

	schedule_delayed_work(&acc->step_iwork, msecs_to_jiffies(
			acc->step_poll_interval));
	mutex_unlock(&acc->lock);

	dev_dbg(&acc->client->dev, "lsm330_step_input_work_func leave\n");
}
#if 0
int lsm330_step_input_open(struct input_dev *input)
{
	int ret = 0;
	struct lsm330_acc_data *acc = input_get_drvdata(input);

	dev_dbg(&acc->client->dev, "lsm330_step_input_open enter\n");

	//ret = lsm330_step_enable(acc);

	dev_dbg(&acc->client->dev, "lsm330_step_input_open leave, ret: %d\n", ret);
	return ret;
}

void lsm330_step_input_close(struct input_dev *dev)
{
	struct lsm330_acc_data *acc = input_get_drvdata(dev);
	//lsm330_step_disable(acc);
}
#endif
static int lsm330_step_input_init(struct lsm330_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->step_iwork, lsm330_step_input_work_func);

	acc->step_poll_interval = 100;
	atomic_set(&acc->step_enabled, 0);

	acc->step_idev = input_allocate_device();
	if (!acc->step_idev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "step input device allocation failed\n");
		goto err0;
	}
#if 0
	acc->step_idev->open = lsm330_step_input_open;
	acc->step_idev->close = lsm330_step_input_close;
#endif
	acc->step_idev->name = LSM330_STEP_DEV_NAME;

	acc->step_idev->id.bustype = BUS_I2C;
	acc->step_idev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->step_idev, acc);

	set_bit(EV_ABS, acc->step_idev->evbit);
	/*	next is used for step sources data if the case */
	set_bit(ABS_MISC, acc->step_idev->absbit);

	input_set_abs_params(acc->step_idev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

	err = input_register_device(acc->step_idev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input device %s\n",
				acc->step_idev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->step_idev);
err0:
	return err;
}
#endif

static void lsm330_acc_input_cleanup(struct lsm330_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static void lsm330_acc_eint_init(struct lsm330_acc_data *acc)
{
#if 1
	mt_set_gpio_dir(GPIO_GYRO_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_GYRO_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_GYRO_EINT_PIN, GPIO_PULL_DOWN);
	//mt_set_gpio_mode(GPIO_GYRO_EINT_PIN, GPIO_MODE_00);
	mt_eint_set_sens(CUST_EINT_GYRO_NUM, MT_EDGE_SENSITIVE);
	mt_eint_registration(CUST_EINT_GYRO_NUM, EINTF_TRIGGER_RISING,
			lsm330_acc_isr1, 0);
	mt_eint_unmask(CUST_EINT_GYRO_NUM);
#endif

	INIT_WORK(&acc->irq1_work, lsm330_acc_irq1_work_func);
	acc->irq1_work_queue =
		create_singlethread_workqueue("lsm330_acc_wq1");
}

static int lsm330_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct lsm330_acc_data *acc;

	int err = -1;

	pr_info("%s: probe start.\n", LSM330_ACC_DEV_NAME);

	/*if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}*/

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}


	acc = kzalloc(sizeof(struct lsm330_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
	i2c_set_clientdata(client, acc);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}

	if(client->dev.platform_data == NULL) {
		pr_info("using default platform_data for accelerometer\n");
		memcpy(acc->pdata, &default_lsm330_acc_pdata,
							sizeof(*acc->pdata));
	} else {
		memcpy(acc->pdata, client->dev.platform_data,
							sizeof(*acc->pdata));
	}

	err = lsm330_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}


	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}
/*
	if(acc->pdata->gpio_int1 >= 0){
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d "
							"mapped on gpio:%d\n",
			LSM330_ACC_DEV_NAME, __func__, acc->irq1,
							acc->pdata->gpio_int1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d "
							"mapped on gpio:%d\n",
			LSM330_ACC_DEV_NAME, __func__, acc->irq2,
							acc->pdata->gpio_int2);
	}
*/
	/* resume state init config */
	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));
	lsm330_acc_set_init_register_values(acc);

	err = lsm330_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = lsm330_acc_interrupt_enable_control(acc,
						acc->interrupts_enable_setting);
	if (err < 0) {
		dev_err(&client->dev, "interrupt settings failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_update_fs_range(acc, acc->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}

#ifdef LSM330_STEPCOUNTER_EN
	err = lsm330_step_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "step input init failed\n");
		goto err_power_off;
	}
#endif

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
		   "device LSM330_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lsm330_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);
	atomic_set(&acc->sensor_phone_calling, 0);

	g_acc_data = acc;
	lsm330_acc_eint_init(acc);

	init_timer(&acc->sensor_phone_calling_timer);
	wake_lock_init(&acc->sensor_suspend_lock, WAKE_LOCK_SUSPEND, "LSM330 ACC wakelock");
/*
	if(acc->pdata->gpio_int1 >= 0){
		INIT_WORK(&acc->irq1_work, lsm330_acc_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("lsm330_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(acc->irq1, lsm330_acc_isr1,
				IRQF_TRIGGER_RISING, "lsm330_acc_irq1", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		INIT_WORK(&acc->irq2_work, lsm330_acc_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("lsm330_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, lsm330_acc_isr2,
				IRQF_TRIGGER_RISING, "lsm330_acc_irq2", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}
	*/

/* do not init stepcounter for saving 0.3 mA when the machine power up */
#if 0
#ifdef LSM330_STEPCOUNTER_EN
	lsm330_acc_stepcounter_init(acc);
#endif
#endif
	mutex_unlock(&acc->lock);

	dev_info(&client->dev, "%s: probed\n", LSM330_ACC_DEV_NAME);

	return 0;

err_input_cleanup:
	lsm330_acc_input_cleanup(acc);
err_power_off:
	lsm330_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
//err_freedata:
	kfree(acc);
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LSM330_ACC_DEV_NAME);
	return err;
}

static int lsm330_acc_remove(struct i2c_client *client)
{
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	if (acc->irq1_work_queue)
		destroy_workqueue(acc->irq1_work_queue);
	if (acc->irq2_work_queue)
		destroy_workqueue(acc->irq2_work_queue);

	if (atomic_cmpxchg(&acc->enabled, 1, 0))
			cancel_delayed_work_sync(&acc->input_work);

	lsm330_acc_device_power_off(acc);
	lsm330_acc_input_cleanup(acc);
	remove_sysfs_interfaces(&client->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();

	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM
static int lsm330_acc_resume(struct i2c_client *client)
{
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	if (atomic_read(&acc->sensor_phone_calling))
		return 0;

	if (acc->on_before_suspend)
		return lsm330_acc_enable(acc);
	return 0;
}

static int lsm330_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	if (atomic_read(&acc->sensor_phone_calling))
		return 0;

	acc->on_before_suspend = atomic_read(&acc->enabled);
	return lsm330_acc_disable(acc);
}
#else
#define lsm330_acc_suspend	NULL
#define lsm330_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm330_acc_id[]
		= { { LSM330_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, lsm330_acc_id);

static struct i2c_driver lsm330_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM330_ACC_DEV_NAME,
		  },
	.probe = lsm330_acc_probe,
	.remove = lsm330_acc_remove,
	.suspend = lsm330_acc_suspend,
	.resume = lsm330_acc_resume,
	.id_table = lsm330_acc_id,
};

static struct i2c_board_info i2c_lsm330_acc[] = {
	{
		I2C_BOARD_INFO("lsm330_acc", 0x1d)
	}
};

static int __init lsm330_acc_init(void)
{
	pr_info("%s accelerometer driver: init\n", LSM330_ACC_DEV_NAME);
	i2c_register_board_info(1, i2c_lsm330_acc, 1);
	return i2c_add_driver(&lsm330_acc_driver);
}

static void __exit lsm330_acc_exit(void)
{
#ifdef DEBUG
	pr_info("%s accelerometer driver exit\n", LSM330_ACC_DEV_NAME);
#endif /* DEBUG */
	i2c_del_driver(&lsm330_acc_driver);
	return;
}

module_init(lsm330_acc_init);
module_exit(lsm330_acc_exit);

MODULE_DESCRIPTION("lsm330 accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");

