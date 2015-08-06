#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (768)
#define FRAME_HEIGHT (1280)
#define REGFLAG_DELAY 0xffe
#define REGFLAG_END_OF_TABLE 0xfff

#ifndef BUILD_LK
#define lcm_print(fmt, args...) printk(fmt, ##args)
#else
#define lcm_print(fmt, args...) printf(fmt, ##args)
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

//#define POWER_MODE_5


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table truly_init_code[] = {
    {0x11,    0,    {}},
    {REGFLAG_DELAY, 200,{}}, //wait more than 180ms
    {0x29,    0,    {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}},
};

static struct LCM_setting_table truly_slpin_dispoff[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 16, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++) {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {
        case REGFLAG_DELAY :
			if (table[i].count == 0)
				break;
            if(table[i].count <= 10) 
                UDELAY(table[i].count * 1000);
            else
                MDELAY(table[i].count);
            break;

        case REGFLAG_END_OF_TABLE :
            break;

        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

static void lcm_power_switch_1st_stage(bool enable)
{
	if (enable) 
	{
		SET_RESET_PIN(1);
		MDELAY(10);

		SET_RESET_PIN(0);
		MDELAY(8);
		SET_RESET_PIN(1);
		MDELAY(15);
		//VSP5V
        mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
		MDELAY(15);
        //VSN5V
        mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
		MDELAY(180);
		//SET_RESET_PIN(1);
		//MDELAY(60);
	} 
	else 
	{
		SET_RESET_PIN(0);
		MDELAY(1);
	    //VSN5V
        mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
        MDELAY(15);
        //VSP5V
        mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
		MDELAY(100);
	}
}

static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
		//params->physical_width = 70;//69.984
		//params->physical_height = 117;//116.64

		params->dsi.mode   = BURST_VDO_MODE;

		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.packet_size = 256;
		//params->dsi.clk_lp_per_line_enable = 1;
		// Video mode setting
		params->dsi.intermediat_buffer_num = 0;
		params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count = FRAME_WIDTH * 3;
		
		params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 6;
		params->dsi.vertical_frontporch					= 14;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;
		params->dsi.horizontal_sync_active				= 8; 
		params->dsi.horizontal_backporch				= 32;
		params->dsi.horizontal_frontporch				= 88;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.PLL_CLOCK = 220;
		params->dsi.ssc_disable=1;
}

static void lcm_init(void)
{

	lcm_power_switch_1st_stage(true);
	push_table(truly_init_code,
		sizeof(truly_init_code)/sizeof(struct LCM_setting_table), 1);
}

//#define LCM_POWER_ENABLE
static void lcm_suspend(void)
{
	push_table(truly_slpin_dispoff,
		sizeof(truly_slpin_dispoff)/sizeof(struct LCM_setting_table), 1);
#ifdef LCM_POWER_ENABLE
	lcm_power_switch_1st_stage(false);
#else
	MDELAY(60);
#endif
}

static void lcm_suspend_power(void)
{
	lcm_power_switch_1st_stage(false);
}
static void lcm_resume(void)
{
	lcm_print("resume @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	lcm_init();
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{

	return 0;
  	unsigned int ret = 0;

	ret = mt_get_gpio_in(GPIO_LCD_ID_PIN);

	return (ret == 0)?1:0; 
}

LCM_DRIVER hx8394d_hd_dsi_vdo_truly_lcm_drv = 
{
    .name			= "hx8394d_hd_dsi_vdo_truly",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
#ifdef BUILD_LK
	.suspend_power  = lcm_suspend_power,
#endif
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
