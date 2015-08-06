/* 
 * Copyright (C) 2012 Senodia.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/i2c/st480.h>
//#include "st480.h"
#include <linux/dma-mapping.h>

#define MZ_MAG_DEV_NAME "mz_mag"
#define MZ_MAG_I2C_NAME "mz_mag_i2c"
#define MZ_MAG_MIN_POLLRATE 50
#define ABSMIN_MAG	-32767
#define ABSMAX_MAG	32767

#define	I2C_RETRY_DELAY     5      /* Waiting for signals [ms] */
#define	I2C_RETRIES         5      /* Number of retries */
#define	I2C_AUTO_INCREMENT  0x80   /* Autoincrement i2c address */

#if 0
#define pr_info(format, arg...)         printk(KERN_EMERG format , ## arg)
#define dev_err(dev, format, arg...)    printk(KERN_EMERG format , ## arg)
#define dev_info(dev, format, arg...)   printk(KERN_EMERG format , ## arg)
#define dev_dbg(dev, format, arg...)    printk(KERN_EMERG format , ## arg)
#define dev_warn(dev, format, arg...)   printk(KERN_EMERG format , ## arg)
#define dev_notice(dev, format, arg...) printk(KERN_EMERG format , ## arg)
#endif

/*
	axis_map_x, axis_map_y, axis_map_z can 
	be used to swap x, y and z axes. 
	If axis_map_x is 0 then it will be mapped to x-axis in android, 
	if it is 1 then it will be mapped to y axis in Android and so on.

	To change the axes direction we can set negate_x, 
	negate_y and negate_z value to 1. 
	If negate_x, negate_y and negate_z values are set to 1 
	then driver will negate accelerometer axes readings 
	before passing to user space (Android). 	
*/
struct mz_mag_rotation {
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
};

struct mz_mag_data;

struct mz_mag_ops
{
	int (*enable_device)(struct mz_mag_data *mz_mag);
	int (*disable_device)(struct mz_mag_data *mz_mag);
	int (*set_range)(struct mz_mag_data *mz_mag, int range);
	int (*get_range)(struct mz_mag_data *mz_mag);
	int (*set_pollrate)(struct mz_mag_data *mz_mag, int pollrate);
	int (*get_pollrate)(struct mz_mag_data *mz_mag);
	int (*get_mag_data)(struct mz_mag_data *mz_mag, int16_t *mag_buf);
	int (*self_test)(struct mz_mag_data *mz_mag);
};

struct mz_mag_data {

	struct i2c_client *client;
	struct input_dev *input_dev;
	struct delayed_work input_work;
	struct mutex lock;

	struct mz_mag_rotation *rota;
	struct mz_mag_ops *ops;

	uint16_t device_id;
	uint8_t  sens[3];
	atomic_t enabled;
	int      range;
	int      pollrate;
};



/* ====================================================
	st480 Driver 
   ==================================================== */

static int st480_i2c_transfer_data(struct i2c_client *client,
							int len, char *buf, int length)
{

	int ret;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr  =  client->addr,
			.flags  =  0,
			.len  =  len,
			.buf  =  buf,
		},
		{
			.addr  =  client->addr,
			.flags  = I2C_M_RD,
			.len  =  length,
			.buf  =  buf,
		},
	};

	do {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((ret != 2) && (++tries < I2C_RETRIES));

	if (ret != 2) {
		dev_err(&client->dev, "read transfer error\n");
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;

}

static u8 *gpDMABuf_va;
static dma_addr_t gpDMABuf_pa;

static char * st480_read_measurement(struct i2c_client *client)
{
	int ret = 0;
	char cmd[1] = {READ_MEASUREMENT_CMD};

	struct i2c_msg msgs[] = {
		{
			.addr  =  client->addr,
			.flags  =  0,
			.len  =  1,
			.buf  =  cmd,
			//.scl_rate = 400 * 1000,
		},
		{
			.addr  =  client->addr,
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags  = I2C_M_RD,
			.len  =  9,
			.buf  =  (u8 *)gpDMABuf_pa,
			//.scl_rate = 400 * 1000,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if(ret < 0){
		printk(KERN_EMERG "st480_i2c_transfer_data i2c_transfer error, ret: %d\n", ret);
		return NULL;
	}

	return gpDMABuf_va;

}


static int st480_self_test(struct mz_mag_data *mz_mag)
{
	int    ret;
	char * buffer;
	char * xyz_data;
	s16    hw_data_z0;
	s16    hw_data_z1;
	s16    z_bist;
	unsigned char buf[5];

	buf[0] = SINGLE_MEASUREMENT_MODE_CMD;
	st480_i2c_transfer_data(mz_mag->client, 1, buf, 1);

	msleep(500);

	/* read the normal z data */
	buffer = st480_read_measurement(mz_mag->client);
	if (NULL == buffer) {
		dev_err(&mz_mag->client->dev, "Failed to read measurement\n");
		ret = -1;
		goto err1;
	}

	xyz_data   = buffer + 3;
	hw_data_z0 = (xyz_data[4]<<8)|xyz_data[5];

	/* enter self test mode */
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = 0x01;
	buf[2] = 0x7C;
	buf[3] = ONE_INIT_REG;
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);

	msleep(250);

	buf[0] = SINGLE_MEASUREMENT_MODE_CMD;
	st480_i2c_transfer_data(mz_mag->client, 1, buf, 1);

	msleep(500);

	/* read the self test z data */
	buffer = st480_read_measurement(mz_mag->client);
	if (NULL == buffer) {
		dev_err(&mz_mag->client->dev, "Failed to read measurement\n");
		ret = -1;
		goto err2;
	}

	xyz_data   = buffer + 3;
	hw_data_z1 = (xyz_data[4]<<8)|xyz_data[5];

	/* calculate z bist */
	z_bist = hw_data_z1 - hw_data_z0;
	dev_dbg(&mz_mag->client->dev, "z1: %d, z0: %d, z_bist: %d\n",
		hw_data_z1, hw_data_z0, z_bist);
	if (z_bist >= -180 && z_bist <= -90) {
		dev_dbg(&mz_mag->client->dev, "st480 self test success!\n");
		ret = 1;
	} else {
		dev_err(&mz_mag->client->dev, "self test failed\n");
		ret = -1;
	}

err2:
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = 0x00;
	buf[2] = 0x7C;
	buf[3] = ONE_INIT_REG;
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);
err1:
	return ret;

}

#define READ_MEASUREMENT_XYZ_CMD        (0x4E)
#define READ_MEASUREMENT_T_CMD          (0x41)
static int st480_get_mag_data(struct mz_mag_data *mz_mag, int16_t * mag_data)
{
	int    ret;
	char   cmd[1];
	char * buffer;
	char * temp_data;
	char * xyz_data;
	s16    hw_data[3];
	int16_t mag_x, mag_y, mag_z;

	buffer = st480_read_measurement(mz_mag->client);
	if (NULL == buffer) {
		dev_err(&mz_mag->client->dev, "Failed to read measurement\n");
		ret = -1;
		goto out;
	}

	temp_data = buffer + 1;
	xyz_data  = buffer + 3;

	if(!((buffer[0]>>4) & 0X01)) {

		hw_data[0] = (xyz_data[0]<<8)|xyz_data[1];
		hw_data[1] = (xyz_data[2]<<8)|xyz_data[3];
		hw_data[2] = (xyz_data[4]<<8)|xyz_data[5];

		mag_x = ((mz_mag->rota->negate_x)?(-hw_data[mz_mag->rota->axis_map_x])
				: (hw_data[mz_mag->rota->axis_map_x]));
		mag_y = ((mz_mag->rota->negate_y)?(-hw_data[mz_mag->rota->axis_map_y])
				: (hw_data[mz_mag->rota->axis_map_y]));
		mag_z = ((mz_mag->rota->negate_z)?(-hw_data[mz_mag->rota->axis_map_z])
				: (hw_data[mz_mag->rota->axis_map_z]));

		if( ((temp_data[0]<<8)|(temp_data[1])) > 46244) {

			mag_x = mag_x * (1 + (70/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));
			mag_y = mag_y * (1 + (70/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));
			mag_z = mag_z * (1 + (70/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));

		} else if( ((temp_data[0]<<8)|(temp_data[1])) < 46244) {

			mag_x = mag_x * (1 + (60/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));
			mag_y = mag_y * (1 + (60/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));
			mag_z = mag_z * (1 + (60/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));
		}
		dev_dbg(&mz_mag->client->dev,
				"st480 raw data: x = %d, y = %d, z = %d \n",
				mag_x,mag_y,mag_z);

		mag_data[0] = mag_x;
		mag_data[1] = mag_y;
		mag_data[2] = mag_z;
		ret = 0;

	} else {

		dev_dbg(&mz_mag->client->dev, 
			"get data state: 0x%X \n", buffer[0]);
		dev_dbg(&mz_mag->client->dev, 
			"temp data: 0x%02X 0x%02X\n", temp_data[0], temp_data[1]);
		dev_dbg(&mz_mag->client->dev, 
			"xyz  data: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n",
			xyz_data[0], xyz_data[1], xyz_data[2], 
			xyz_data[3], xyz_data[4], xyz_data[5]);

		ret = -1;
	}

out:
	cmd[0] = SINGLE_MEASUREMENT_MODE_CMD;
	st480_i2c_transfer_data(mz_mag->client, 1, cmd, 1);
	return ret;
}

struct mz_mag_ops st480_ops = {
	.get_mag_data = st480_get_mag_data,
	.self_test    = st480_self_test,
};

struct mz_mag_rotation st480_rota = {
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x   = 0,
	.negate_y   = 1,
	.negate_z   = 0,
};
static int st480_hw_init(struct mz_mag_data *mz_mag)
{
	unsigned char buf[5];

#define ST480_DMA_MAX_TRANSACTION_LENGTH  255
	gpDMABuf_va = (u8 *)dma_alloc_coherent(NULL, ST480_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
	if(!gpDMABuf_va){
		printk(KERN_EMERG "Allocate DMA I2C Buffer failed!\n");
		return -1;
	}

	memset(buf, 0, 5);

	buf[0] = READ_REGISTER_CMD;
	buf[1] = 0x00;
	st480_i2c_transfer_data(mz_mag->client, 2, buf, 3);

	if(buf[2] != ST480_DEVICE_ID) {
		dev_dbg(&mz_mag->client->dev, "mag device is not st480\n");
		return -1;
	}

	dev_dbg(&mz_mag->client->dev, "mag device is st480\n");
	dev_dbg(&mz_mag->client->dev, "device id: 0x00%02X\n", buf[2]);

	mz_mag->device_id = ST480_DEVICE_ID;
	mz_mag->rota      = &st480_rota;
	mz_mag->ops       = &st480_ops;

	//init register step 1
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = ONE_INIT_DATA_HIGH;
	buf[2] = ONE_INIT_DATA_LOW;
	buf[3] = ONE_INIT_REG;  
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);

	//init register step 2
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = TWO_INIT_DATA_HIGH;
	buf[2] = TWO_INIT_DATA_LOW;
	buf[3] = TWO_INIT_REG;
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);

	//disable temperature compensation register
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = TEMP_DATA_HIGH;
	buf[2] = TEMP_DATA_LOW;
	buf[3] = TEMP_REG; 
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);

	//set calibration register
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = CALIBRATION_DATA_HIGH;
	buf[2] = CALIBRATION_DATA_LOW;
	buf[3] = CALIBRATION_REG;
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);

	//set mode config
	buf[0] = SINGLE_MEASUREMENT_MODE_CMD;
	st480_i2c_transfer_data(mz_mag->client, 1, buf, 1);

	return 0;
}


/* ====================================================
	akm09911 Driver 
   ==================================================== */



#define AKM09911_DEVICE_ID    0x0548
#define AKM09911_REG_WIA1     0x00
#define AKM09911_REG_WIA2     0x01
#define AKM09911_REG_INFO1    0x02
#define AKM09911_REG_INFO2    0x03
#define AKM09911_REG_ST1      0x10
#define AKM09911_REG_DATA     AKM09911_REG_HXL
#define AKM09911_REG_HXL      0x11
#define AKM09911_REG_HXH      0x12
#define AKM09911_REG_HYL      0x13
#define AKM09911_REG_HYH      0x14
#define AKM09911_REG_HZL      0x15
#define AKM09911_REG_HZH      0x16
#define AKM09911_REG_TMPS     0x17
#define AKM09911_REG_ST2      0x18
#define AKM09911_REG_CNTL1    0x30
#define AKM09911_REG_CNTL2    0x31
#define AKM09911_REG_CNTL3    0x32

#define AKM09911_FUSE_ASAX     0x60
#define AKM09911_FUSE_ASAY     0x61
#define AKM09911_FUSE_ASAZ     0x62

#define AKM09911_MODE_SNG_MEASURE   0x01
#define AKM09911_MODE_SELF_TEST     0x10
#define AKM09911_MODE_FUSE_ACCESS   0x1F
#define AKM09911_MODE_POWERDOWN     0x00
#define AKM09911_RESET_DATA         0x01


static int akm09911_i2c_read(struct i2c_client *client,
				u8 * buf, int len)
{
	int ret;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = (client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((ret != 2) && (++tries < I2C_RETRIES));

	if (ret != 2) {
		dev_err(&client->dev, "read transfer error\n");
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}


static int akm09911_i2c_write(struct i2c_client *client, u8 * buf,
								int len)
{
	int ret;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};

	do {
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((ret != 1) && (++tries < I2C_RETRIES));

	if (ret != 1) {
		dev_err(&client->dev, "write transfer error\n");
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}


static int akm09911_get_mag_data(struct mz_mag_data *mz_mag, int16_t * mag_data)
{
	int ret;
	u8 buf[9];
	int hw_data[3];
	u8 *sens;
	int16_t mag_x, mag_y, mag_z;

	sens = mz_mag->sens;

	buf[0] = AKM09911_REG_ST1;
	ret = akm09911_i2c_read(mz_mag->client, buf, 1);
	if (ret < 0) {
		goto out;
	}

	if (!(buf[0] & 0x01)) {
		dev_dbg(&mz_mag->client->dev, "data not ready\n");
		ret = -1;
		goto out;
	}

	buf[0] = AKM09911_REG_DATA;
	ret = akm09911_i2c_read(mz_mag->client, buf, 8);
	if (ret < 0) {
		ret = -1;
		goto out;
	}

	if (buf[7] & 0x08) {
		dev_dbg(&mz_mag->client->dev, "magnetic sensor overflow\n");
		ret = -1;
		goto out;
	}

	hw_data[0] = (int)((int16_t)(buf[1]<<8)+((int16_t)buf[0]));
	hw_data[1] = (int)((int16_t)(buf[3]<<8)+((int16_t)buf[2]));
	hw_data[2] = (int)((int16_t)(buf[5]<<8)+((int16_t)buf[4]));

	hw_data[0] = ((hw_data[0] * (sens[0] + 128)) >> 7);
	hw_data[1] = ((hw_data[1] * (sens[1] + 128)) >> 7);
	hw_data[2] = ((hw_data[2] * (sens[2] + 128)) >> 7);

#if 0
	mag_x = -hw_data[0];
	mag_y =  hw_data[1];
	mag_z = -hw_data[2];
#else
	mag_x = ((mz_mag->rota->negate_x) 
			? (-hw_data[mz_mag->rota->axis_map_x])
			: (hw_data[mz_mag->rota->axis_map_x]));
	mag_y = ((mz_mag->rota->negate_y) 
			? (-hw_data[mz_mag->rota->axis_map_y])
			: (hw_data[mz_mag->rota->axis_map_y]));
	mag_z = ((mz_mag->rota->negate_z) 
			? (-hw_data[mz_mag->rota->axis_map_z])
			: (hw_data[mz_mag->rota->axis_map_z]));
#endif

#if 0
	dev_dbg(&mz_mag->client->dev, "mag data: %d %d %d\n",
		mag_x, mag_y, mag_z);
#endif

	mag_data[0] = mag_x;
	mag_data[1] = mag_y;
	mag_data[2] = mag_z;
	ret = 0;

out:
	buf[0] = 0x31;
	buf[1] = 0x01;
	akm09911_i2c_write(mz_mag->client, buf, 1);

	return ret;
}

static int akm09911_self_test(struct mz_mag_data *mz_mag)
{
	int i;
	int ret;
	int retry;
	int hw_data[3];
	unsigned char buf[8];
	int ak09911_st_lower[3] = {-50, -50, -400};
	int ak09911_st_upper[3] = {50, 50, -100};

	/* power down mode */
	buf[0] = AKM09911_REG_CNTL2;
	buf[1] = AKM09911_MODE_POWERDOWN;
	akm09911_i2c_write(mz_mag->client, buf, 1);

	/* self test mode */
	buf[0] = AKM09911_REG_CNTL2;
	buf[1] = AKM09911_MODE_SELF_TEST;
	akm09911_i2c_write(mz_mag->client, buf, 1);

	/* wait for data ready */
	retry = 10;
	while (retry--) {
		msleep(100);
		buf[0] = AKM09911_REG_ST1;
		ret = akm09911_i2c_read(mz_mag->client, buf, 1);
		if (ret < 0) {
			goto err1;
		}

		if (!(buf[0] & 0x01)) {
			dev_dbg(&mz_mag->client->dev, "data not ready\n");
			continue;
		} else {
			dev_dbg(&mz_mag->client->dev, "data is ready\n");
			break;
		}
	}

	if (!(buf[0] & 0x01)) {
		dev_err(&mz_mag->client->dev, "failed to wait data ready\n");
		ret = -1;
		goto err2;
	}

	/* read sensor data */
	buf[0] = AKM09911_REG_DATA;
	ret = akm09911_i2c_read(mz_mag->client, buf, 8);
	if (ret < 0) {
		ret = -1;
		goto err2;
	}

	if (buf[7] & 0x08) {
		dev_dbg(&mz_mag->client->dev, "magnetic sensor overflow\n");
		ret = -1;
		goto err2;
	}

	hw_data[0] = (int)((int16_t)(buf[1]<<8)+((int16_t)buf[0]));
	hw_data[1] = (int)((int16_t)(buf[3]<<8)+((int16_t)buf[2]));
	hw_data[2] = (int)((int16_t)(buf[5]<<8)+((int16_t)buf[4]));

	hw_data[0] = ((hw_data[0] * (mz_mag->sens[0] + 128)) >> 7);
	hw_data[1] = ((hw_data[1] * (mz_mag->sens[1] + 128)) >> 7);
	hw_data[2] = ((hw_data[2] * (mz_mag->sens[2] + 128)) >> 7);

	dev_dbg(&mz_mag->client->dev, "data:  %d %d %d\n",
		hw_data[0], hw_data[1], hw_data[2]);
	dev_dbg(&mz_mag->client->dev, "upper: %d %d %d\n",
		ak09911_st_upper[0], ak09911_st_upper[1], ak09911_st_upper[2]);
	dev_dbg(&mz_mag->client->dev, "lower: %d %d %d\n",
		ak09911_st_lower[0], ak09911_st_lower[1], ak09911_st_lower[2]);

	ret = 1;
	for (i=0; i<3; i++) {
		if (hw_data[i]>ak09911_st_upper[i]
		 || hw_data[i]<ak09911_st_lower[i]) {
			dev_err(&mz_mag->client->dev, "self test failed\n");
			ret = -1;
			break;
		}
	}

	if (ret == 1) {
		dev_dbg(&mz_mag->client->dev, "self test success\n");
	}

err2:
err1:
	/* revert to power down mode */
	buf[0] = AKM09911_REG_CNTL2;
	buf[1] = AKM09911_MODE_POWERDOWN;
	akm09911_i2c_write(mz_mag->client, buf, 1);
	return ret;
}

struct mz_mag_ops akm09911_ops = {
	.get_mag_data = akm09911_get_mag_data,
	.self_test    = akm09911_self_test,
};

struct mz_mag_rotation akm09911_rota = {
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x   = 1,
	.negate_y   = 0,
	.negate_z   = 1,
};

static int akm09911_hw_init(struct mz_mag_data *mz_mag)
{
	unsigned char buf[4];

	memset(buf, 0, 4);

	/* get device id of magnetometer */
	buf[0] = AKM09911_REG_WIA1;
	akm09911_i2c_read(mz_mag->client, buf, 2);

	if (AKM09911_DEVICE_ID != (buf[1]<<8|buf[0])) {
		dev_dbg(&mz_mag->client->dev, "mag device is not akm09911\n");
		return -1;
	}

	dev_dbg(&mz_mag->client->dev, "mag device is akm09911\n");
	dev_dbg(&mz_mag->client->dev, "device id: 0x%02X%02X\n", buf[1], buf[0]);

	mz_mag->device_id = AKM09911_DEVICE_ID;
	mz_mag->rota      = &akm09911_rota;
	mz_mag->ops       = &akm09911_ops;

	/* set AKM to Fuse ROM access mode */
	buf[0] = AKM09911_REG_CNTL2;
	buf[1] = AKM09911_MODE_FUSE_ACCESS;
	akm09911_i2c_write(mz_mag->client, buf, 1);

	/* read sens */
	buf[0] = AKM09911_FUSE_ASAX;
	akm09911_i2c_read(mz_mag->client, buf, 3);

	dev_dbg(&mz_mag->client->dev, "mag sens: 0x%X 0x%X 0x%X\n",
		buf[0], buf[1], buf[2]);

	mz_mag->sens[0] = buf[0];
	mz_mag->sens[1] = buf[1];
	mz_mag->sens[2] = buf[2];

	/* revert to power down mode */
	buf[0] = AKM09911_REG_CNTL2;
	buf[1] = AKM09911_MODE_POWERDOWN;
	akm09911_i2c_write(mz_mag->client, buf, 1);

	return 0;
}



/* ====================================================
	Meizu Magnetometer Driver 
   ==================================================== */


/* sysfs interfaces */

static inline int mz_mag_enable(struct mz_mag_data *mz_mag)
{
	int ret = 0;

	if (!atomic_cmpxchg(&mz_mag->enabled, 0, 1)) {

		if (mz_mag->ops->enable_device)
			ret = mz_mag->ops->enable_device(mz_mag);

		if (ret < 0) {
			atomic_set(&mz_mag->enabled, 0);
			return ret;
		}

		dev_dbg(&mz_mag->client->dev, "mag input work start\n");
		schedule_delayed_work(&mz_mag->input_work,
			msecs_to_jiffies(mz_mag->pollrate));
	}

	return 0;
}

static inline int mz_mag_disable(struct mz_mag_data *mz_mag)
{
	if (atomic_cmpxchg(&mz_mag->enabled, 1, 0)) {

		dev_dbg(&mz_mag->client->dev, "mag input work stop\n");
		cancel_delayed_work_sync(&mz_mag->input_work);

		if (mz_mag->ops->disable_device)
			mz_mag->ops->disable_device(mz_mag);
	}

	return 0;
}


static ssize_t attr_pollrate_ms_show(struct device *dev,
					struct device_attribute *attr,
								char *buf)
{
	int val;
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);

	mutex_lock(&mz_mag->lock);
	val = mz_mag->pollrate;
	mutex_unlock(&mz_mag->lock);

	dev_dbg(&mz_mag->client->dev, "get mag pollrate: %d\n", val);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_pollrate_ms_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	unsigned long rate;

	dev_dbg(&mz_mag->client->dev, "set mag pollrate: %s\n", buf);

	if (strict_strtoul(buf, 10, &rate)) 
		return -EINVAL;
	if (!rate)
		return -EINVAL;

	rate = rate > MZ_MAG_MIN_POLLRATE ? rate : MZ_MAG_MIN_POLLRATE;

	mutex_lock(&mz_mag->lock);

	if (mz_mag->ops->set_pollrate)
		ret = mz_mag->ops->set_pollrate(mz_mag, rate);

	if(ret >= 0) {
		mz_mag->pollrate = rate;
	}

	mutex_unlock(&mz_mag->lock);

	return size;
}


static ssize_t attr_full_scale_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	int range = 2;

	mutex_lock(&mz_mag->lock);

	range = mz_mag->range;

	mutex_unlock(&mz_mag->lock);

	dev_dbg(&mz_mag->client->dev, "get mag range: %d\n", range);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_full_scale_store(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	unsigned long range;

	dev_dbg(&mz_mag->client->dev, "set mag range: %s\n", buf);

	if (strict_strtoul(buf, 10, &range))
		return -EINVAL;

	mutex_lock(&mz_mag->lock);

	if (mz_mag->ops->set_range)
		mz_mag->ops->set_range(mz_mag, range);

	mz_mag->range = range;
	mutex_unlock(&mz_mag->lock);

	return size;
}

static ssize_t attr_enable_device_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	int val = atomic_read(&mz_mag->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_device_store(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		mz_mag_enable(mz_mag);
	else
		mz_mag_disable(mz_mag);

	return size;
}

static ssize_t attr_self_test_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 1;
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);

	mutex_lock(&mz_mag->lock);

	if (mz_mag->ops->self_test)
		ret = mz_mag->ops->self_test(mz_mag);

	mutex_unlock(&mz_mag->lock);

	return sprintf(buf, "%d\n", ret);
}

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0664, attr_pollrate_ms_show, attr_pollrate_ms_store),
	__ATTR(full_scale, 0664, attr_full_scale_show, attr_full_scale_store),
	__ATTR(enable_device, 0664, attr_enable_device_show, 
		attr_enable_device_store),
	__ATTR(self_test, S_IRUGO, attr_self_test_show, NULL),
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
#if 0
static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}
#endif

/* input device interfaces */

static void mz_mag_input_func(struct work_struct *work)
{
	int ret;
	struct mz_mag_data *mz_mag;
	int16_t mag_data[3];

	mz_mag = container_of((struct delayed_work *)work,
					struct mz_mag_data, input_work);

	if (mz_mag->ops->get_mag_data)
		ret = mz_mag->ops->get_mag_data(mz_mag, mag_data);
	else
		ret = -1;

	if (!ret) {
		input_report_abs(mz_mag->input_dev, ABS_X, mag_data[0]);
		input_report_abs(mz_mag->input_dev, ABS_Y, mag_data[1]);
		input_report_abs(mz_mag->input_dev, ABS_Z, mag_data[2]);
		input_event(mz_mag->input_dev, EV_SYN, SYN_REPORT, mz_mag->device_id);
	}

	schedule_delayed_work(&mz_mag->input_work,
					msecs_to_jiffies(mz_mag->pollrate));
}

static int mz_mag_input_init(struct mz_mag_data *mz_mag)
{
	int ret;

	INIT_DELAYED_WORK(&mz_mag->input_work, mz_mag_input_func);

	mz_mag->input_dev = input_allocate_device();
	if (!mz_mag->input_dev) {
		ret = -ENOMEM;
		dev_err(&mz_mag->client->dev, "Failed to allocate input device\n");
		goto err0;
	}

	/* Setup input device */
	set_bit(EV_ABS, mz_mag->input_dev->evbit);

	input_set_capability(mz_mag->input_dev, EV_ABS, ABS_X);
	input_set_capability(mz_mag->input_dev, EV_ABS, ABS_Y);
	input_set_capability(mz_mag->input_dev, EV_ABS, ABS_Z);

	/* x-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(mz_mag->input_dev, ABS_X, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* y-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(mz_mag->input_dev, ABS_Y, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* z-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(mz_mag->input_dev, ABS_Z, ABSMIN_MAG, ABSMAX_MAG, 0, 0);

	/* Set name */
	mz_mag->input_dev->name       = MZ_MAG_DEV_NAME;
	mz_mag->input_dev->dev.parent = &mz_mag->client->dev;

	/* Register */
	ret = input_register_device(mz_mag->input_dev);
	if (ret) {
		dev_err(&mz_mag->client->dev, "Failed to register input device\n");
		goto err1;
	}
	input_set_drvdata(mz_mag->input_dev, mz_mag);

	return 0;

err1:
	input_free_device(mz_mag->input_dev);
	mz_mag->input_dev = NULL;

err0:
	return ret;
}

static int mz_mag_setup(struct mz_mag_data *mz_mag)
{
	int ret;

	dev_dbg(&mz_mag->client->dev, "setup magnetometer sensor!!\n");

	ret = akm09911_hw_init(mz_mag);
	if (ret == 0)
		return 0;

	ret = st480_hw_init(mz_mag);
	if (ret == 0)
		return 0;

	dev_err(&mz_mag->client->dev, "Failed to setup magnetometer sensor\n");

	return ret;
}


int mz_mag_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct mz_mag_data *mz_mag;

	pr_info("%s mz_mag_probe\n", MZ_MAG_DEV_NAME);

	mz_mag = kzalloc(sizeof(struct mz_mag_data), GFP_KERNEL);
	if (!mz_mag) {
		pr_err("%s mz_mag_probe: memory allocation failed.\n", MZ_MAG_DEV_NAME);
		ret = -ENOMEM;
		goto err0;
	}

	mz_mag->client   = client;
	mz_mag->range    = 2;
	mz_mag->pollrate = 100;
	atomic_set(&mz_mag->enabled, 0);
	mutex_init(&mz_mag->lock);

	i2c_set_clientdata(client, mz_mag);

	ret = mz_mag_setup(mz_mag);
	if(ret < 0) 
		goto err1;

    ret = mz_mag_input_init(mz_mag);
    if (ret < 0)
    	goto err2;

	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0)
		goto err3;

	dev_dbg(&mz_mag->client->dev, "mz mag probe done\n");

	return 0;

err3:
err2:
err1:
	kfree(mz_mag);
err0:
	pr_err("%s mz_mag_probe failed.\n", MZ_MAG_DEV_NAME);
	return ret;

}

static const struct i2c_device_id mz_mag_id[] = {
	{MZ_MAG_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver mz_mag_driver = {
	.probe     = mz_mag_probe,
	.id_table  = mz_mag_id,
	.driver = {
		.name  = MZ_MAG_DEV_NAME,
		.owner = THIS_MODULE,
	},
};

static struct i2c_board_info mz_mag_i2c[] = {
	{
		I2C_BOARD_INFO(MZ_MAG_DEV_NAME, 0x0c),
	},
};

static int __init mz_mag_init(void)
{	
	int ret;

	pr_info("%s mz_mag_init\n", MZ_MAG_DEV_NAME);

	ret = i2c_register_board_info(1, mz_mag_i2c, 1);
	if (ret < 0) {
		pr_err("%s i2c_register_board_info failed\n", MZ_MAG_DEV_NAME);
	}

	pr_info("%s i2c_register_board_info success\n", MZ_MAG_DEV_NAME);

	return i2c_add_driver(&mz_mag_driver);
}

static void __exit mz_mag_exit(void)
{
	i2c_del_driver(&mz_mag_driver);
}

module_init(mz_mag_init);
module_exit(mz_mag_exit);

MODULE_AUTHOR("ZhangJiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Meizu M79 Magnetometer linux driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
