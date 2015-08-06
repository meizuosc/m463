#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include <mach/irqs.h>
#include <linux/input.h>
#include "cust_eint.h"
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/input/pa12200002.h>
#include <linux/sensors_io.h>
#include <linux/suspend.h>
#include <linux/meizu-sys.h>

//#define	SUNLIGHT_ALS_USED

static struct i2c_board_info i2c_txc[] = {
	{
		I2C_BOARD_INFO("PA122", 0x1e)
	}
};

struct txc_data *txc_info = NULL;

// I2C read one byte data from register 
static int i2c_read_reg(struct i2c_client *client,u8 reg,u8 *data)
{
	u8 databuf[2];
	int res = 0;
	databuf[0]= reg;

	mutex_lock(&txc_info->i2c_lock);
	res = i2c_master_send(client,databuf,0x1);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		mutex_unlock(&txc_info->i2c_lock);
		return res;
	}
	res = i2c_master_recv(client,data,0x1);
	if(res <= 0)
	{
		APS_ERR("i2c_master_recv function err\n");
		mutex_unlock(&txc_info->i2c_lock);
		return res;
	}

	mutex_unlock(&txc_info->i2c_lock);
	return 0;
}
// I2C Write one byte data to register
static int i2c_write_reg(struct i2c_client *client,u8 reg,u8 value)
{
	u8 databuf[2];
	int res = 0;

	databuf[0] = reg;
	databuf[1] = value;

	mutex_lock(&txc_info->i2c_lock);
	res = i2c_master_send(client,databuf,0x2);
	if (res < 0){
		APS_ERR("i2c_master_send function err\n");
		mutex_unlock(&txc_info->i2c_lock);
		return res;
	}
	mutex_unlock(&txc_info->i2c_lock);
	return 0;
}

static int pa12201001_init(struct txc_data *data)
{
	int ret = 0;
	u8 sendvalue = 0;
	struct i2c_client *client = data->client;

	data->pa122_sys_run_cal = 0;
	data->ps_calibvalue = PA12_PS_OFFSET_DEFAULT + PA12_PS_OFFSET_EXTRA;
	data->fast_calib_flag = 0;
	data->ps_data = PS_UNKONW;

	// reset ps to initial state
	ret = i2c_write_reg(client,REG_CFG2, PA12_RESET_CLEAR);

	/* Sun light */
#ifdef	SUNLIGHT_ALS_USED
	ret = i2c_write_reg(client,REG_CFG0, PA12_ALS_GAIN4000);
#else
	// make sure ps and als is disabled
	ret = i2c_write_reg(client,REG_CFG0, 0);
#endif

	ret = i2c_write_reg(client,REG_PS_SET, 0x03); //PSET, Normal Mode

	sendvalue= PA12_PS_INT_HYSTERESIS | PA12_PS_PERIOD12;
	ret=i2c_write_reg(client,REG_CFG3,sendvalue);

	// Set PS threshold
	sendvalue=PA12_PS_FAR_TH_HIGH;
	ret=i2c_write_reg(client,REG_PS_TH,sendvalue); //set TH threshold

	sendvalue=PA12_PS_NEAR_TH_LOW;
	ret=i2c_write_reg(client,REG_PS_TL,sendvalue); //set TL threshold
	ret = i2c_write_reg(client, REG_PS_OFFSET, PA12_PS_OFFSET_DEFAULT);
	if(ret < 0)
	{
		APS_ERR("i2c_send function err\n");
	}

	return ret;
}

#ifdef	SUNLIGHT_ALS_USED
static int pa12201001_set_ps_mode(struct i2c_client *client)
{
	u8 sendvalue=0, regdata = 0;
	int res = 0;

	if (txc_info->ps_enable) {
		sendvalue= PA12_LED_CURR150 | PA12_PS_PRST4 | PA12_ALS_PRST4;
		res=i2c_write_reg(client,REG_CFG1,sendvalue);

		// Interrupt Setting	 
		res=i2c_write_reg(client,REG_CFG2,
				(PA12_INT_ALS_PS_BOTH | PA12_PS_MODE_NORMAL |
				 PA12_PS_INTF_INACTIVE | PA12_ALS_INTF_INACTIVE
				 )); //set int mode

		/* Sun Light */
		res = i2c_write_reg(client, REG_ALS_TH_LSB,
				PA12_ALS_TH_LSB_SUN_LIGHT_ON);
		res = i2c_write_reg(client, REG_ALS_TH_MSB,
				PA12_ALS_TH_MSB_SUN_LIGHT_ON);
		res = i2c_write_reg(client, REG_ALS_TL_LSB,
				PA12_ALS_TL_LSB_SUN_LIGHT_ON);
		res = i2c_write_reg(client, REG_ALS_TL_MSB,
				PA12_ALS_TL_MSB_SUN_LIGHT_ON);
	} else {
		sendvalue= PA12_LED_CURR150 | PA12_PS_PRST4;
		res=i2c_write_reg(client,REG_CFG1,sendvalue);

		/* when mcu enbaled, we should set the irq as edge type*/
		res = i2c_read_reg(client, REG_CFG2, &regdata);
		sendvalue = (regdata & 0x03) | PA12_INT_PS |
			PA12_PS_MODE_NORMAL;
		res=i2c_write_reg(client,REG_CFG2, sendvalue); //set int mode
	}
	return 0 ;
}

#else

static int pa12201001_set_ps_mode(struct i2c_client *client)
{
	u8 sendvalue=0, regdata = 0;
	int res = 0;

	sendvalue= PA12_LED_CURR150 | PA12_PS_PRST4;
	res=i2c_write_reg(client,REG_CFG1,sendvalue);


	// Interrupt Setting	   
	res = i2c_read_reg(client, REG_CFG2, &regdata);
	sendvalue = (regdata & 0x03) | PA12_INT_PS | PA12_PS_MODE_NORMAL;
	res=i2c_write_reg(client,REG_CFG2, sendvalue); //set int mode

	return 0 ;
}

#endif

//PS enable function
int pa12201001_enable_ps(struct i2c_client *client, int enable)
{
	int res;
	u8 regdata=0;
	u8 sendvalue=0;

	if(enable == 1) //PS ON
	{
		printk("pa12201001 enable ps sensor\n");
		res=i2c_read_reg(client,REG_CFG0,&regdata); //Read Status
		if(res<0){
			APS_ERR("i2c_read function err\n");
			return res;
		}else{

			//sendvalue=regdata & 0xFD; //clear bit
			//sendvalue=sendvalue | 0x02; //0x02 PS Flag
			sendvalue=regdata & 0xFC; //clear bit-0 & bit-1
#ifdef	SUNLIGHT_ALS_USED
			sendvalue=sendvalue | 0x03; //0x03 PS On & ALS On
#else
			sendvalue=sendvalue | 0x02; //0x02 PS On
#endif
			res=i2c_write_reg(client,REG_CFG0,sendvalue); //Write PS enable 

			if(res<0){
				APS_ERR("i2c_write function err\n");
				return res;
			}
			res=i2c_read_reg(client,REG_CFG0,&regdata); //Read Status
			APS_LOG("CFG0 Status: %d\n",regdata);
		}
	}else{       //PS OFF

		printk("pa12201001 disable ps sensor\n");
		res=i2c_read_reg(client,REG_CFG0,&regdata); //Read Status
		if(res<0){
			APS_ERR("i2c_read function err\n");
			return res;
		}else{
			APS_LOG("CFG0 Status: %d\n",regdata);

			//sendvalue=regdata & 0xFD; //clear bit
			sendvalue=regdata & 0xFC; //clear bit-0 & bit-1
			res=i2c_write_reg(client,REG_CFG0,sendvalue); //Write PS disable 

			if(res<0){
				APS_ERR("i2c_write function err\n");
				return res;
			}
		}
	}

	return 0;
}

//Read PS Count : 8 bit
int pa12201001_read_ps(struct i2c_client *client, u8 *data)
{
	int res;
	u8 psdata = 0;

	// APS_FUN(f);
	res = i2c_read_reg(client,REG_PS_DATA,data); //Read PS Data
	//psdata = i2c_smbus_read_byte_data(client, REG_PS_DATA); 
	if(res < 0){
		APS_ERR("i2c_send function err\n");
	}

	return res;
}

//Read ALS Count : 16 bit
int pa12201001_read_als(struct i2c_client *client, u16 *data)
{
	int res;
	u8 LSB = 0;
	u8 MSB = 0;

	res = i2c_read_reg(client, REG_ALS_DATA_LSB, &LSB); //Read PS Data
	res = i2c_read_reg(client, REG_ALS_DATA_MSB, &MSB); //Read PS Data
	*data = (MSB << 8) | LSB;
	//psdata = i2c_smbus_read_byte_data(client, REG_PS_DATA); 
	if(res < 0){
		APS_ERR("i2c_send function err\n");
	}

	return res;
}

void pa12_swap(u8 *x, u8 *y)
{
	u8 temp = *x;
	*x = *y;
	*y = temp;
}

static int pa122_run_fast_calibration(struct i2c_client *client)
{

	struct txc_data *data = i2c_get_clientdata(client);
	int i = 0;
	int j = 0;
	u16 sum_of_pdata = 0;
	u8  xtalk_temp = 0;
	u8 temp_pdata[4], cfg0data = 0,cfg2data = 0,cfg3data = 0;
	unsigned int ArySize = 4;

	APS_LOG("START proximity sensor calibration\n");

	i2c_write_reg(client, REG_PS_TH, 0xFF);
	i2c_write_reg(client, REG_PS_TL, 0);

	i2c_read_reg(client, REG_CFG0, &cfg0data);
	i2c_read_reg(client, REG_CFG2, &cfg2data);
	i2c_read_reg(client, REG_CFG3, &cfg3data);

	/*Offset mode & disable intr from ps*/
	i2c_write_reg(client, REG_CFG2, cfg2data & 0x33);

	/*PS sleep time 6.5ms */
	i2c_write_reg(client, REG_CFG3, cfg3data & 0xC7);

	/*Set crosstalk = 0*/
	i2c_write_reg(client, REG_PS_OFFSET, 0x00);

	/*PS On*/
	i2c_write_reg(client, REG_CFG0, cfg0data | 0x02);
	mdelay(50);

	for(i = 0; i < 4; i++)
	{
		mdelay(7);
		i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
		APS_LOG("temp_data = %d\n", temp_pdata[i]);
	}

	/* pdata sorting */
	for (i = 0; i < ArySize - 1; i++)
		for (j = i+1; j < ArySize; j++)
			if (temp_pdata[i] > temp_pdata[j])
				pa12_swap(temp_pdata + i, temp_pdata + j);

	/* calculate the cross-talk using central 2 data */
	for (i = 1; i < 3; i++)
	{
		APS_LOG("%s: temp_pdata = %d\n", __func__, temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	xtalk_temp = sum_of_pdata/2;
	APS_LOG("%s: sum_of_pdata = %d   calibvalue = %d\n",
			__func__, sum_of_pdata, xtalk_temp);

	/* Restore Data */
	i2c_write_reg(client, REG_CFG0, cfg0data);
	i2c_write_reg(client, REG_CFG2, cfg2data | 0xC0); //make sure return normal mode
	i2c_write_reg(client, REG_CFG3, cfg3data);

	i2c_write_reg(client, REG_PS_TH, PA12_PS_FAR_TH_HIGH);
	i2c_write_reg(client, REG_PS_TL, PA12_PS_NEAR_TH_LOW);

	if (((xtalk_temp + PA12_PS_OFFSET_EXTRA) > data->factory_calibvalue) &&
			(xtalk_temp < PA12_PS_OFFSET_MAX))
	{
		printk("Fast calibrated data=%d\n",xtalk_temp);
		data->fast_calib_flag = 1;
		data->fast_calibvalue = xtalk_temp + PA12_PS_OFFSET_EXTRA;
		/* Write offset value to 0x10 */
		i2c_write_reg(client, REG_PS_OFFSET,
				xtalk_temp + PA12_PS_OFFSET_EXTRA);
		return xtalk_temp + PA12_PS_OFFSET_EXTRA;
	}
	else
	{
		printk("Fast calibration fail, calibvalue=%d, use factory_calibvalue %d\n",xtalk_temp, data->factory_calibvalue);
		data->fast_calib_flag = 0;
		if(data->pa122_sys_run_cal) {
			i2c_write_reg(client, REG_PS_OFFSET,
					data->factory_calibvalue);
		} else {
			i2c_write_reg(client, REG_PS_OFFSET,
					PA12_PS_OFFSET_MAX);
		}
		xtalk_temp = data->factory_calibvalue;

		return xtalk_temp;
	}
}

static int pa122_run_calibration(struct txc_data *data)
{
	struct i2c_client *client = data->client;
	int i, j;
	int ret;
	u16 sum_of_pdata = 0;
	u8 temp_pdata[20],buftemp[1],cfg0data=0,cfg2data=0;
	unsigned int ArySize = 20;
	unsigned int cal_check_flag = 0;
	u8 value=0;
	int calibvalue;

	printk("%s: START proximity sensor calibration\n", __func__);

RECALIBRATION:
	sum_of_pdata = 0;

	ret = i2c_read_reg(client, REG_CFG0, &cfg0data);
	ret = i2c_read_reg(client, REG_CFG2, &cfg2data);

	/*Set to offset mode & disable interrupt from ps*/
	ret = i2c_write_reg(client, REG_CFG2, cfg2data & 0x33);

	/*Set crosstalk = 0*/
	ret = i2c_write_reg(client, REG_PS_OFFSET, 0x00);
	if (ret < 0) {
		pr_err("%s: txc_write error\n", __func__);
		/* Restore CFG2 (Normal mode) and Measure base x-talk */
		ret = i2c_write_reg(client, REG_CFG0, cfg0data);
		ret = i2c_write_reg(client, REG_CFG2, cfg2data | 0xC0);
		return ret;
	}

	/*PS On*/
	ret = i2c_write_reg(client, REG_CFG0, cfg0data | 0x02);
	mdelay(50);

	for(i = 0; i < 20; i++)
	{
		mdelay(15);
		ret = i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
		APS_LOG("temp_data = %d\n", temp_pdata[i]);
	}

	/* pdata sorting */
	for (i = 0; i < ArySize - 1; i++)
		for (j = i+1; j < ArySize; j++)
			if (temp_pdata[i] > temp_pdata[j])
				pa12_swap(temp_pdata + i, temp_pdata + j);

	/* calculate the cross-talk using central 10 data */
	for (i = 5; i < 15; i++)
	{
		APS_LOG("%s: temp_pdata = %d\n", __func__, temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	calibvalue = sum_of_pdata/10;

	/* Restore CFG2 (Normal mode) and Measure base x-talk */
	ret = i2c_write_reg(client, REG_CFG0, cfg0data);
	ret = i2c_write_reg(client, REG_CFG2, cfg2data | 0xC0);

	if (calibvalue > PA12_PS_OFFSET_MAX)
	{
		APS_LOG("%s: invalid calibrated data, calibvalue %d\n",
				__func__, data->ps_calibvalue);
		data->pa122_sys_run_cal = 0;

		if(cal_check_flag == 0)
		{
			APS_LOG("%s: RECALIBRATION start\n", __func__);
			cal_check_flag = 1;
			goto RECALIBRATION;
		}
		else
		{
			APS_LOG("%s: CALIBRATION FAIL -> "
					"cross_talk is set to DEFAULT\n", __func__);
			ret = i2c_write_reg(client, REG_PS_OFFSET,
					data->factory_calibvalue);
			return -EINVAL;
		}
	}

	data->ps_calibvalue = calibvalue + PA12_PS_OFFSET_EXTRA;

CROSSTALKBASE_RECALIBRATION:

	/*PS On*/
	ret = i2c_write_reg(client, REG_CFG0, cfg0data | 0x02);

	/*Write offset value to register 0x10*/
	ret = i2c_write_reg(client, REG_PS_OFFSET, data->ps_calibvalue);

	for(i = 0; i < 10; i++)
	{
		mdelay(15);
		ret = i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
		APS_LOG("temp_data = %d\n", temp_pdata[i]);
	}

	/* pdata sorting */
	for (i = 0; i < 9; i++)
	{
		for (j = i+1; j < 10; j++)
		{
			if (temp_pdata[i] > temp_pdata[j])
				pa12_swap(temp_pdata + i, temp_pdata + j);
		}
	}

	/* calculate the cross-talk_base using central 5 data */
	sum_of_pdata = 0;

	for (i = 3; i < 8; i++)
	{
		APS_LOG("%s: temp_pdata = %d\n", __func__, temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	data->calibvalue_base = sum_of_pdata/5;

	if(data->calibvalue_base > 0)
	{
		if (data->calibvalue_base >= 8)
			data->ps_calibvalue += data->calibvalue_base/8;
		else
			data->ps_calibvalue += 1;
		goto CROSSTALKBASE_RECALIBRATION;
	} else if (data->calibvalue_base == 0) {
		data->pa122_sys_run_cal = 1;
	}
	/* Restore CFG0  */
	ret = i2c_write_reg(client, REG_CFG0, cfg0data);

	printk("%s: FINISH proximity sensor calibration, %d\n",
			__func__, data->ps_calibvalue);
}

static void txc_set_enable(struct txc_data *txc, int enable)
{
	struct i2c_client *client = txc->client;
	int ret;

	mutex_lock(&txc->enable_lock);

	if (enable) {
		mt_eint_mask(CUST_EINT_ALS_NUM);
		pa12201001_set_ps_mode(client);
#if defined(PA12_FAST_CAL)
		if(!txc->fast_calib_flag) {
			pa122_run_fast_calibration(client);
		}
#endif
	} else {
		input_report_abs(txc->input_dev, ABS_DISTANCE, PS_UNKONW);
		input_sync(txc->input_dev);
	}

	ret = pa12201001_enable_ps(client, enable);

	if (enable) {
		mdelay(50);
		mt_eint_unmask(CUST_EINT_ALS_NUM);
	}
	if (ret < 0) {
		mutex_unlock(&txc->enable_lock);
		APS_ERR("pa12201001_enable_ps function err\n");
		return ret;
	}
	mutex_unlock(&txc->enable_lock);
}

static ssize_t txc_ps_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int enabled;

	enabled = txc_info->ps_enable;

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t txc_ps_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int enable = simple_strtol(buf, NULL, 10);

	pr_err("[pa122] enable ps %d\n", enable);

	// if fast calibration failed, we should run again
	if(txc_info->ps_enable == enable && txc_info->fast_calib_flag)
		return count;

	txc_info->ps_enable = enable;
	txc_info->ps_data = PS_UNKONW;

	txc_set_enable(txc_info, enable);

	return count;
}

static ssize_t txc_ps_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	u8 ps_data;

	ret = i2c_read_reg(client, REG_PS_DATA, &ps_data); //Read PS Data

	printk("ps data is %d \n", ps_data);

	return sprintf(buf, "%d\n", ps_data);
}

static ssize_t txc_als_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	u8 msb, lsb;
	u16 als_data;

	ret = i2c_read_reg(client, REG_ALS_DATA_LSB, &lsb);
	ret = i2c_read_reg(client, REG_ALS_DATA_MSB, &msb);

	als_data = (msb << 8) | lsb;
	printk("als data is %d \n", als_data);

	return sprintf(buf, "%d\n", als_data);
}


static ssize_t pa12200001_show_reg(struct device *dev,
		struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 regdata;
	int res=0;
	int count=0;
	int i=0	;

	for(i;i <17 ;i++)
	{
		res=i2c_read_reg(client,0x00+i,&regdata);
		if(res<0)
		{
			break;
		}
		else
			count+=sprintf(buf+count,"[%x] = (%x)\n",0x00+i,regdata);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t pa12200001_store_send(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int addr, cmd;

	if(2 != sscanf(buf, "%x %x", &addr, &cmd)) {
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	i2c_write_reg(client, addr, cmd);

	return count;
}

static ssize_t txc_calibration_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	int calibration = simple_strtol(buf, NULL, 10);

	if (calibration) {
		pr_err("[pa122] run calibration\n");
		pa122_run_calibration(txc_info);
	}

	return count;
}

static ssize_t txc_calibvalue_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;

	if (txc_info->pa122_sys_run_cal)
		ret = txc_info->ps_calibvalue;
	else
		ret = -1;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t txc_calibvalue_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	int ret;

	txc_info->factory_calibvalue = simple_strtol(buf, NULL, 10);
	txc_info->pa122_sys_run_cal = 1;
	pr_err("[pa122] write factory_calibvalue %d\n",
			txc_info->factory_calibvalue);

	return count;
}

static ssize_t txc_batch_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	/*
	int ps_batch = simple_strtol(buf, NULL, 10);

	if (ps_batch == 1) {
	input_report_abs(txc_info->input_dev, ABS_DISTANCE, PS_UNKONW);
	input_sync(txc_info->input_dev);
	}
	*/
	return count;
}

extern int mt_eint_get_polarity(unsigned int eint_num);

static ssize_t txc_interrupt_show(struct device *dev,
		struct device_attribute *attr, const char *buf)
{
	int count = 0;
	int ps_gpio = mt_get_gpio_in(GPIO_ALS_EINT_PIN);
	int eint_state = mt_eint_get_polarity(CUST_EINT_ALS_NUM);

	count += sprintf(buf, "%d, %d\n", ps_gpio, eint_state);
	count += sprintf(buf + count, "%d, %d, %d, %d\n",
			txc_info->fast_calibvalue,
			txc_info->factory_calibvalue,
			txc_info->fast_calib_flag,
			txc_info->pa122_sys_run_cal);

	return count;
}

static ssize_t txc_interrupt_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	//int state = simple_strtol(buf, NULL, 10);
	u8 data = 0;

	// make sure the interrupt is inactive
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);

	i2c_read_reg(txc_info->client, REG_CFG2, &data);

	data &= 0xfc;
	i2c_write_reg(txc_info->client, REG_CFG2, data);

	mt_eint_unmask(CUST_EINT_ALS_NUM);

	return count;
}

static ssize_t txc_wakeup_enable_show(struct device *dev,
		struct device_attribute *attr, const char *buf)
{
	return sprintf(buf, "%d\n", txc_info->wakeup_enable);
}

static ssize_t txc_wakeup_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	txc_info->wakeup_enable = !!simple_strtol(buf, NULL, 10);

	return count;
}

/* sysfs attributes operation function*/
static DEVICE_ATTR(ps_enable, 0664, txc_ps_enable_show, txc_ps_enable_store);
static DEVICE_ATTR(ps_data, 0664, txc_ps_data_show, NULL);
static DEVICE_ATTR(als_data, 0664, txc_als_data_show, NULL);
static DEVICE_ATTR(reg, 0664, pa12200001_show_reg, pa12200001_store_send);
static DEVICE_ATTR(calibration, 0664, NULL, txc_calibration_store);
static DEVICE_ATTR(calibvalue, 0664, txc_calibvalue_show, txc_calibvalue_store);
static DEVICE_ATTR(ps_batch, 0664, NULL, txc_batch_store);
static DEVICE_ATTR(interrupt, 0664, txc_interrupt_show, txc_interrupt_store);
static DEVICE_ATTR(wakeup_enable, 0664, txc_wakeup_enable_show,
		txc_wakeup_enable_store);

static struct attribute *txc_attributes[] = {
	&dev_attr_ps_enable.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_als_data.attr,
	&dev_attr_reg.attr,
	&dev_attr_calibration.attr,
	&dev_attr_calibvalue.attr,
	&dev_attr_ps_batch.attr,
	&dev_attr_interrupt.attr,
	&dev_attr_wakeup_enable.attr,
	NULL,
};

static struct attribute_group txc_attribute_group = {
	.attrs = txc_attributes,
};

static void txc_ps_handler(struct work_struct *work)
{
	struct txc_data *txc = container_of(work, struct txc_data, ps_dwork.work);
	struct i2c_client *client = txc_info->client;
	u8 psdata=0;
	u16 alsdata = 0;
	int ps_data = PS_UNKONW;
	u8 sendvalue;
	int res;
	u8 data;
	int ret;

	ret = pa12201001_read_ps(client,&psdata);
#ifdef	SUNLIGHT_ALS_USED
	ret |= pa12201001_read_als(client, &alsdata);
#endif

	if (ret < 0) {
		pr_err("%s: txc_write error\n", __func__);
		goto exit;
	}

#ifdef	SUNLIGHT_ALS_USED
	if(alsdata > PA12_SUN_LIGHT_ON)
	{
		i2c_write_reg(client, REG_ALS_TH_LSB, PA12_ALS_TH_LSB_SUN_LIGHT_OFF);
		i2c_write_reg(client, REG_ALS_TH_MSB, PA12_ALS_TH_MSB_SUN_LIGHT_OFF);
		i2c_write_reg(client, REG_ALS_TL_LSB, PA12_ALS_TL_LSB_SUN_LIGHT_OFF);
		i2c_write_reg(client, REG_ALS_TL_MSB, PA12_ALS_TL_MSB_SUN_LIGHT_OFF);
		i2c_write_reg(client, REG_PS_OFFSET, 0);
	}
	else if(alsdata < PA12_SUN_LIGHT_OFF)
	{
		i2c_write_reg(client, REG_ALS_TH_LSB, PA12_ALS_TH_LSB_SUN_LIGHT_ON);
		i2c_write_reg(client, REG_ALS_TH_MSB, PA12_ALS_TH_MSB_SUN_LIGHT_ON);
		i2c_write_reg(client, REG_ALS_TL_LSB, PA12_ALS_TL_LSB_SUN_LIGHT_ON);
		i2c_write_reg(client, REG_ALS_TL_MSB, PA12_ALS_TL_MSB_SUN_LIGHT_ON);
		if (txc->fast_calib_flag)
			i2c_write_reg(client, REG_PS_OFFSET, txc->fast_calibvalue);
		else
			i2c_write_reg(client, REG_PS_OFFSET, txc->ps_calibvalue);
	}
#endif
	if (txc->ps_data == PS_UNKONW || txc->ps_data == PS_FAR) {
		if(psdata > PA12_PS_FAR_TH_HIGH){
			mt_eint_set_polarity(CUST_EINT_ALS_NUM, MT_EINT_POL_POS);
			ps_data = PS_NEAR;

			sendvalue= PA12_LED_CURR150 | PA12_PS_PRST2 | PA12_ALS_PRST4;
			res|=i2c_write_reg(client,REG_CFG1,sendvalue);
			if (res < 0) {
				pr_err("%s: txc_write error\n", __func__);
				goto exit;
			}

			sendvalue= PA12_PS_INT_HYSTERESIS | PA12_PS_PERIOD6;
			res=i2c_write_reg(client,REG_CFG3,sendvalue);
			if (res < 0) {
				pr_err("%s: txc_write error\n", __func__);
				goto exit;
			}
		} else if (psdata < PA12_PS_NEAR_TH_LOW) {
			ps_data= PS_FAR;
			mt_eint_set_polarity(CUST_EINT_ALS_NUM, MT_EINT_POL_NEG);
		} 
	} else if (txc->ps_data == PS_NEAR) {
		if(psdata < PA12_PS_NEAR_TH_LOW){
			mt_eint_set_polarity(CUST_EINT_ALS_NUM, MT_EINT_POL_NEG);
			ps_data = PS_FAR;

#ifdef	SUNLIGHT_ALS_USED
			sendvalue= PA12_LED_CURR150 | PA12_PS_PRST4 | PA12_ALS_PRST4;
#else
			sendvalue= PA12_LED_CURR150 | PA12_PS_PRST4;
#endif
			res=i2c_write_reg(client,REG_CFG1,sendvalue);
			if (res < 0) {
				pr_err("%s: txc_write error\n", __func__);
				goto exit;
			}
			sendvalue= PA12_PS_INT_HYSTERESIS | PA12_PS_PERIOD12;
			res=i2c_write_reg(client,REG_CFG3,sendvalue);
			if (res < 0) {
				pr_err("%s: txc_write error\n", __func__);
				goto exit;
			}
		}
	}

	pr_debug("[pa122] dump last ps_data %d, cur ps_data %d\n",
			txc->ps_data, ps_data);
	if (txc->ps_data != ps_data) {
		wake_lock_timeout(&txc->ps_wake_lock, 2*HZ);
		txc->ps_data = ps_data;
		input_report_abs(txc->input_dev, ABS_DISTANCE, ps_data);
		input_sync(txc->input_dev);
	}

exit:
#ifdef	SUNLIGHT_ALS_USED
	i2c_read_reg(txc->client, REG_CFG2, &data);
	data &= 0xfe;
	i2c_write_reg(txc->client, REG_CFG2, data);
#endif

	if(mt_get_gpio_in(GPIO_ALS_EINT_PIN) ==
			mt_eint_get_polarity(CUST_EINT_ALS_NUM)) {
		pr_err("[pa122] wrong eint signal detected, pull up\n");
		mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
	}

	mt_eint_unmask(CUST_EINT_ALS_NUM);
}

static void txc_irq_handler(void)
{
	mt_eint_mask(CUST_EINT_ALS_NUM);
	schedule_delayed_work(&txc_info->ps_dwork, 0);
}

static int txc_irq_init(struct txc_data *txc)
{
	int ret;
	int irq;

	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_eint_set_sens(CUST_EINT_ALS_NUM, MT_EDGE_SENSITIVE);
	mt_eint_registration(CUST_EINT_ALS_NUM,
			EINTF_TRIGGER_FALLING, txc_irq_handler, 0);
	mt_eint_unmask(CUST_EINT_ALS_NUM);

	return ret;
}

static int txc_create_input(struct txc_data *txc)
{
	int ret;
	struct input_dev *dev;

	dev = input_allocate_device();
	if (!dev) {
		pr_err("%s()->%d:can not alloc memory to txc input device!\n",
				__func__, __LINE__);
		return -ENOMEM;
	}

	set_bit(EV_ABS, dev->evbit);
	input_set_capability(dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(dev, ABS_DISTANCE, 0, 1, 0, 0);  /*the max value 1bit*/
	dev->name = "pa122";
	dev->dev.parent = &txc->client->dev;

	ret = input_register_device(dev);
	if (ret < 0) {
		pr_err("%s()->%d:can not register txc input device!\n",
				__func__, __LINE__);
		input_free_device(dev);
		return ret;
	}

	txc->input_dev = dev;
	input_set_drvdata(txc->input_dev, txc);

	return 0;
}

static int ps_notifier_handler(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct txc_data *data = container_of(this, struct txc_data,
			ps_pm_notifier);
	static wakeup_enable_save;

	if (event == PM_SUSPEND_PREPARE)
		wakeup_enable_save = data->wakeup_enable;

	pr_debug("[pa122] pm event %lu, wakeup_enable %d\n", event,
			wakeup_enable_save);
	if(!wakeup_enable_save) {// skip mask in dialing
		if (event == PM_SUSPEND_PREPARE) {
			mt_eint_mask(CUST_EINT_ALS_NUM);
		} else if (event == PM_POST_SUSPEND) {
			mt_eint_unmask(CUST_EINT_ALS_NUM);
		}
	}

	return event;
}

static int txc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct txc_data *txc;

	/*request private data*/
	txc = kzalloc(sizeof(struct txc_data), GFP_KERNEL);
	if (!txc) {
		pr_err("%s()->%d:can not alloc memory to private data !\n",
				__func__, __LINE__);
		return -ENOMEM;
	}

	/*set client private data*/
	txc->client = client;
	i2c_set_clientdata(client, txc);
	txc_info = txc;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s()->%d:i2c adapter don't support i2c operation!\n",
				__func__, __LINE__);
		return -ENODEV;
	}
	mutex_init(&txc->enable_lock);
	mutex_init(&txc->i2c_lock);
	wake_lock_init(&txc->ps_wake_lock, WAKE_LOCK_SUSPEND,
			"ps_wake_lock");

	/*create input device for reporting data*/
	ret = txc_create_input(txc);
	if (ret < 0) {
		pr_err("%s()->%d:can not create input device!\n",
				__func__, __LINE__);
		return ret;
	}

	ret = pa12201001_init(txc);
	if (ret < 0) {
		pr_err("%s()->%d:pa12201001_init failed!\n", __func__, __LINE__);
		return ret;
	}

	ret = sysfs_create_group(&client->dev.kobj, &txc_attribute_group);
	if (ret < 0) {
		pr_err("%s()->%d:can not create sysfs group attributes!\n",
				__func__, __LINE__);
		return ret;
	}

	meizu_sysfslink_register_name(&client->dev, "ps");

	/*register ps pm notifier */
	txc->ps_pm_notifier.notifier_call = ps_notifier_handler;
	register_pm_notifier(&txc->ps_pm_notifier);

	INIT_DELAYED_WORK(&txc->ps_dwork, txc_ps_handler);
	txc_irq_init(txc);

	printk("%s: probe ok!!, client addr is 0x%02x\n", __func__, client->addr);
	return 0;
}

static const struct i2c_device_id txc_id[] = {
	{ "PA122", 0 },
	{},
};

static int txc_remove(struct i2c_client *client)
{
	struct txc_data *txc = i2c_get_clientdata(client);

	meizu_sysfslink_unregister(&client->dev);

	sysfs_remove_group(&client->dev.kobj, &txc_attribute_group);
	input_unregister_device(txc->input_dev);
	input_free_device(txc->input_dev);
	unregister_pm_notifier(&txc->ps_pm_notifier);
	kfree(txc);

	return 0;
}

static struct i2c_driver txc_driver = {
	.driver = {
		.name	= TXC_DEV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= txc_probe,
	.remove = txc_remove,
	.id_table = txc_id,
};

static int __init txc_init(void)
{
	i2c_register_board_info(1, i2c_txc, 1);
	return i2c_add_driver(&txc_driver);
}

static void __exit txc_exit(void)
{
	i2c_del_driver(&txc_driver);
}

module_init(txc_init);
module_exit(txc_exit);

