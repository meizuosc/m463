#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <string.h>
#include <platform/mt_i2c.h> 
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#ifndef LCM_DEBUG_LOG
#define LCM_DEBUG_LOG
#endif
#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)
#define REGFLAG_DELAY 0xffe
#define REGFLAG_END_OF_TABLE 0xfff

#ifdef LCM_DEBUG_LOG
#ifndef BUILD_LK
#define lcm_print(fmt, args...) printk(fmt, ##args)
#else
#define lcm_print(fmt, args...) printf(fmt, ##args)
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
#else
#ifndef BUILD_LK
#define lcm_print(fmt, args...)
#else
#define lcm_print(fmt, args...)
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;
//extern unsigned int mz_get_hw_version(void);

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

//#define POWER_MODE_5
#ifndef ESD_SUPPORT
#define ESD_SUPPORT
#endif
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
//static unsigned int lcm_esd_check(void);

#define   LCM_DSI_CMD_MODE							0
static unsigned int lcm_compare_id(void);
struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table sharp_init_code[] = {
	{REGFLAG_DELAY, 15,  {}}, //2 wait more than 10ms
		{0xFF,	1,	{0x00}},
		{0xFB,	1,	{0x01}},
	{REGFLAG_DELAY, 22,	{}}, //2 wait more than 20ms
		{0xFF,	1,	{0x00}},
		{0xD3,	1,	{0x08}},//VBP=8
		{0xD4,	1,	{0x0E}},//VFP=14
		{0xFF,	1,	{0x01}},
		{0xFB,	1,	{0x01}},
		{0x16,	1,	{0x11}},
		{0x17,	1,	{0x11}},
		{0xFF,	1,	{0x00}},
		{0x11,	0,	{}},
	{REGFLAG_DELAY, 110,{}}, //2 wait more than 100ms
		{0x29,	0,	{}},
	{REGFLAG_DELAY, 44,	{}}, //2 wait more than 40ms
		{0x51,	1,	{0xFF}},//Set Display Brightness 100%
		{0x53,	1,	{0x2C}},
		{0x55,	1,	{0x00}},//CABC OFF
	{REGFLAG_END_OF_TABLE, 0x00, {}}		
};

static struct LCM_setting_table sharp_slpout[] = {
	/*sleep out*/
	{0x11, 0, {}},
	{REGFLAG_DELAY, 120, {}},
	/*TE on*/
	{0xFF,1,{0x05}},
	{0xFB,1,{0x01}},
	{0xD6,1,{0x22}},
	{0xFF,1,{0x00}},
	{0x35, 1, {0}},
	{REGFLAG_END_OF_TABLE, 0, {}},
};
static struct LCM_setting_table sharp_dispon[] = {
	/*display on*/
	{0x29, 0, {}},
	{REGFLAG_DELAY, 0, {}},
	{REGFLAG_END_OF_TABLE, 0, {}},
};

static struct LCM_setting_table sharp_slpin_dispoff[] = {
    // Display off sequence
    {0x53, 1, {0x00}},
    {REGFLAG_DELAY, 2, {}},
    {0x28, 0, {}},
    //{REGFLAG_DELAY, 50, {}},
   // Sleep Mode On
    {0x10, 0, {}},
    {REGFLAG_DELAY, 120, {}},
	{0xFF, 1, {0x00}},
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

static void lcm_power_switch_V2_stage(bool enable)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	
	//lcm_reset_pin_set_mode();
	//SET_RESET_PIN(0);
	//MDELAY(10);//test
	if (enable) 
		{
		//SET_RESET_PIN(0);
		//MDELAY(10);//MDELAY(2);//test
		//VSP5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
		//VSN5V
		MDELAY(1);
		mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
		MDELAY(10);
		SET_RESET_PIN(1);
		MDELAY(1);
		SET_RESET_PIN(0);
		MDELAY(1);
		SET_RESET_PIN(1);
		MDELAY(20);
		} 
	else 
		{
		SET_RESET_PIN(0);
		MDELAY(1);
		//VSN5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
		MDELAY(1);
		//VSP5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
		MDELAY(1);
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

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		//params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;
		params->dsi.mode   = BURST_VDO_MODE;
#endif
		//params->dsi.esd_check_enable = 1;  //for esd test.

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.packet_size = 256;
		//params->dsi.clk_lp_per_line_enable = 1;
		// Video mode setting
		params->dsi.intermediat_buffer_num = 0;
		params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count = FRAME_WIDTH * 3;
		
		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 6;
		params->dsi.vertical_frontporch					= 14;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;
		//params->dsi.vertical_frontporch_for_low_power = 500; // for saving power in screen idle

		params->dsi.horizontal_sync_active				= 8; //+ backprot = 36
		params->dsi.horizontal_backporch				= 16;
		params->dsi.horizontal_frontporch				= 72;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8; 

		// Bit rate calculation
		params->dsi.PLL_CLOCK = 430;
		//1 Every lane speed
	//	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	//	params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4
	//	params->dsi.fbk_div =0x13;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
}

static void lcm_init(void)
{
#ifdef BUILD_LK
	printf("[LK]nt35532----tps65132---cmd   lcm_init\n");
#else
	printk("[kernel]nt35532----tps65132---cmd   lcm_init\n");
#endif
	lcm_power_switch_V2_stage(true);
	push_table(sharp_init_code,
	sizeof(sharp_init_code)/sizeof(struct LCM_setting_table), 1);
	push_table(sharp_slpout,
	sizeof(sharp_slpout)/sizeof(struct LCM_setting_table), 1);
	push_table(sharp_dispon,
	sizeof(sharp_dispon)/sizeof(struct LCM_setting_table), 1);
}
#define LCM_POWER_ENABLE
static void lcm_suspend(void)
{
	lcm_print("nt35532 lcm_suspend \n");
	push_table(sharp_slpin_dispoff,
		sizeof(sharp_slpin_dispoff)/sizeof(struct LCM_setting_table), 1);
#ifdef LCM_POWER_ENABLE
		lcm_power_switch_V2_stage(false);
#else
	MDELAY(60);
#endif
}

static void lcm_suspend_power(void)
{
		lcm_power_switch_V2_stage(false);
}
static void lcm_resume(void)
{
	lcm_print("nt35532 lcm_resume \n");
#ifdef LCM_POWER_ENABLE
	lcm_init();
#else
	push_table(sharp_slpout,
		sizeof(sharp_slpout)/sizeof(struct LCM_setting_table), 1);
	push_table(sharp_dispon,
		sizeof(sharp_dispon)/sizeof(struct LCM_setting_table), 1);
#endif
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

#ifdef ESD_SUPPORT 
static unsigned int lcm_esd_check(void)
{
    unsigned int result = TRUE;
    unsigned int data_array[16];
    unsigned char buffer[16] = {0};

    data_array[0] = 0x00013700;
    dsi_set_cmdq(data_array, 1, 1);

    read_reg_v2(0x0A, buffer, 1);
    if (buffer[0] == 0x9C)
        result = FALSE;
//lcm_print("nt35532 lcm_esd_check %d---\n",buffer[0]);
    return result;
}

static unsigned int lcm_esd_recover(void)
{
    lcm_init();

    return TRUE;
}
#endif

static unsigned int lcm_compare_id(void)
{
  	unsigned int ret = 0;
	mt_set_gpio_mode(GPIO113, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO113, GPIO_DIR_IN);

	ret = mt_get_gpio_in(GPIO113);
#ifdef BUILD_LK
	dprintf("------nt35532 lcm_compare_id ret=%d \n",ret);
#else
	printk("------nt35532 lcm_compare_id ret=%d \n",ret);
#endif
	return (ret == 1)?1:0; 
}

LCM_DRIVER nt35532_fhd_dsi_vdo_m71_tps65132_lcm_drv = 
{
    .name			= "nt35532_fhd_dsi_vdo_m71_tps65132",
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

#ifdef ESD_SUPPORT
    .esd_check      = lcm_esd_check,
    .esd_recover    = lcm_esd_recover,
#endif  

};
