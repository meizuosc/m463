#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

//LiChao @2014.09.17 add for lcd backlight begin
#ifdef MEIZU_M71
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#define LM3695_I2C_BUSNUM  0//for I2C channel 0
#define I2C_ID_NAME "lm3695"
#define LM3695_ADDR 0x63
#endif
//LiChao @2014.09.17 add for lcd backlight end

//extern int mtkfb_set_backlight_level(unsigned int level);
//extern int mtkfb_set_backlight_pwm(int div);
extern int disp_bls_set_backlight(unsigned int level);

// Only support 64 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT 64 
// Support 256 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_256_SUPPORT 256 

// Configure the support type "BACKLIGHT_LEVEL_PWM_256_SUPPORT" or "BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT" !!
#define BACKLIGHT_LEVEL_PWM_MODE_CONFIG BACKLIGHT_LEVEL_PWM_256_SUPPORT

unsigned int Cust_GetBacklightLevelSupport_byPWM(void)
{
	return BACKLIGHT_LEVEL_PWM_MODE_CONFIG;
}

unsigned int brightness_mapping(unsigned int level)
{
    unsigned int mapped_level;
    
    mapped_level = level;
       
	return mapped_level;
}
/*

 * To explain How to set these para for cust_led_list[] of led/backlight
 * "name" para: led or backlight
 * "mode" para:which mode for led/backlight
 *	such as:
 *			MT65XX_LED_MODE_NONE,	
 *			MT65XX_LED_MODE_PWM,	
 *			MT65XX_LED_MODE_GPIO,	
 *			MT65XX_LED_MODE_PMIC,	
 *			MT65XX_LED_MODE_CUST_LCM,	
 *			MT65XX_LED_MODE_CUST_BLS_PWM
 *
 *"data" para: control methord for led/backlight
 *   such as:
 *			MT65XX_LED_PMIC_LCD_ISINK=0,	
 *			MT65XX_LED_PMIC_NLED_ISINK0,
 *			MT65XX_LED_PMIC_NLED_ISINK1,
 *			MT65XX_LED_PMIC_NLED_ISINK2,
 *			MT65XX_LED_PMIC_NLED_ISINK3
 * 
 *"PWM_config" para:PWM(AP side Or BLS module), by default setting{0,0,0,0,0} Or {0}
 *struct PWM_config {	 
 *  int clock_source;
 *  int div; 
 *  int low_duration;
 *  int High_duration;
 *  BOOL pmic_pad;//AP side PWM pin in PMIC chip (only 89 needs confirm); 1:yes 0:no(default)
 *};
 *-------------------------------------------------------------------------------------------
 *   for AP PWM setting as follow:
 *1.	 PWM config data
 *  clock_source: clock source frequency, can be 0/1
 *  div: clock division, can be any value within 0~7 (i.e. 1/2^(div) = /1, /2, /4, /8, /16, /32, /64, /128)
 *  low_duration: only for BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT
 *  High_duration: only for BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT
 *
 *2.	 PWM freq.
 * If BACKLIGHT_LEVEL_PWM_MODE_CONFIG = BACKLIGHT_LEVEL_PWM_256_SUPPORT,
 *	 PWM freq. = clock source / 2^(div) / 256  
 *
 * If BACKLIGHT_LEVEL_PWM_MODE_CONFIG = BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT,
 *	 PWM freq. = clock source / 2^(div) / [(High_duration+1)(Level')+(low_duration+1)(64 - Level')]
 *	           = clock source / 2^(div) / [(High_duration+1)*64]     (when low_duration = High_duration)
 *Clock source: 
 *	 0: block clock/1625 = 26M/1625 = 16K (MT6571)
 *	 1: block clock = 26M (MT6571)
 *Div: 0~7
 *
 *For example, in MT6571, PWM_config = {1,1,0,0,0} 
 *	 ==> PWM freq. = 26M/2^1/256 	 =	50.78 KHz ( when BACKLIGHT_LEVEL_PWM_256_SUPPORT )
 *	 ==> PWM freq. = 26M/2^1/(0+1)*64 = 203.13 KHz ( when BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT )
 *-------------------------------------------------------------------------------------------
 *   for BLS PWM setting as follow:
 *1.	 PWM config data
 *	 clock_source: clock source frequency, can be 0/1/2/3
 *	 div: clock division, can be any value within 0~1023
 *	 low_duration: non-use
 *	 High_duration: non-use
 *	 pmic_pad: non-use
 *
 *2.	 PWM freq.= clock source / (div + 1) /1024
 *Clock source: 
 *	 0: 26 MHz
 *	 1: 104 MHz
 *	 2: 124.8 MHz
 *	 3: 156 MHz
 *Div: 0~1023
 *
 *By default, clock_source = 0 and div = 0 => PWM freq. = 26 KHz 
 *-------------------------------------------------------------------------------------------
 */

//LiChao @2014.09.17 add for lcd backlight begin
 #ifdef MEIZU_M71
/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata lm3695_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, LM3695_ADDR)};
static struct i2c_client *lm3695_i2c_client = NULL;
/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int lm3695_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lm3695_remove(struct i2c_client *client);
static int lm3695_write_bytes(unsigned char addr, unsigned char value);

/***************************************************************************** 
 * Data Structure
 *****************************************************************************/
 struct lm3695_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id lm3695_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

static struct i2c_driver lm3695_iic_driver = {
	.id_table	= lm3695_id,
	.probe		= lm3695_probe,
	.remove		= lm3695_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "lm3695",
	},
 
};

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int lm3695_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk( "lm3695_iic_probe\n");
	printk("LM3695: info==>name=%s addr=0x%x\n",client->name,client->addr);
	lm3695_i2c_client  = client;	
	return 0;      
}

static int lm3695_remove(struct i2c_client *client)
{  	
  printk( "lm3695_remove\n");
  lm3695_i2c_client = NULL;
   i2c_unregister_device(client);
  return 0;
}

 static int lm3695_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = lm3695_i2c_client;
	char write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
	printk("lm3695 write data fail !!\n");	
	return ret ;
}

static int __init lm3695_iic_init(void)
{
   printk( "lm3695_iic_init\n");
   if(i2c_register_board_info(LM3695_I2C_BUSNUM, &lm3695_board_info, 1))
   	{
   printk( "lm3695_iic_init i2c_register_board_info fail\n");
   	}
   else
   	{
   	printk( "lm3695_iic_init i2c_register_board_info success\n");
   	}
   if(i2c_add_driver(&lm3695_iic_driver))
   	{
	   printk( "lm3695_iic_init i2c_add_driver fail\n");
   	}
   else
   	{
   printk( "lm3695_iic_init i2c_add_driver success\n");
   	}
   return 0;
}

static void __exit lm3695_iic_exit(void)
{
  printk( "lm3695_iic_exit\n");
  i2c_del_driver(&lm3695_iic_driver);  
}

module_init(lm3695_iic_init);
module_exit(lm3695_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK LM3695 I2C Driver");
MODULE_LICENSE("GPL"); 
#ifdef MTK_AAL_SUPPORT
static unsigned int BL_Map[1025]=
{
488,489,
490,491,492,493,494,495,496,497,498,499,
500,501,502,503,504,505,506,507,508,509,
510,511,512,513,514,515,516,517,518,519,
520,521,522,523,524,525,526,527,528,529,
530,531,532,533,534,535,536,537,538,539,
540,541,542,543,544,545,546,547,548,549,
550,551,552,553,554,555,556,557,558,559,
560,561,562,563,564,565,566,567,568,569,
570,571,572,573,574,575,576,577,578,579,
580,581,582,583,584,585,586,587,588,589,
590,591,592,593,594,595,596,597,598,599,
600,601,602,603,604,605,606,607,608,609,
610,611,612,613,614,615,616,617,618,619,
620,621,622,623,624,625,626,627,628,629,
630,631,632,633,634,635,636,637,638,639,
640,641,642,643,644,645,646,647,648,649,
650,651,652,653,654,655,656,657,658,659,
660,661,662,663,664,665,666,667,668,669,
670,671,672,673,674,675,676,677,678,679,
680,681,682,683,684,685,686,687,688,689,
690,691,692,693,694,695,696,697,698,699,
700,701,702,703,704,705,706,707,708,709,
710,711,712,713,714,715,716,717,718,719,
720,721,722,723,724,725,726,727,728,729,
730,731,732,733,734,735,736,737,738,739,
740,741,742,743,744,745,746,747,748,749,
750,751,752,753,754,755,756,757,758,759,
760,761,762,763,764,765,766,767,768,769,
770,771,772,773,774,775,776,777,778,779,
780,781,782,783,784,785,786,787,788,789,
790,791,792,793,794,795,796,797,798,799,
800,801,802,803,804,806,807,808,810,811,812,813,815,816,817,818,820,821,822,823,825,826,827,828,830,831,832,833,834,836,837,838,839,
841,842,843,844,845,847,848,849,
850,851,853,854,855,856,857,859,
860,861,862,863,865,866,867,868,869,
870,872,873,874,875,876,878,879,
880,881,882,883,884,886,887,888,889,
890,891,893,894,895,896,897,898,899,
901,902,903,904,905,906,907,908,910,911,912,913,914,915,916,917,919,
920,921,922,923,924,925,926,927,929,
930,931,932,933,934,935,936,937,938,940,941,942,943,944,945,946,947,948,949,
950,951,953,954,955,956,957,958,959,
960,961,962,963,964,965,966,967,968,970,971,972,973,974,975,976,977,978,979,
980,981,982,983,984,985,986,987,988,989,
990,991,993,994,995,996,997,998,999,
1000,1001,1002,1003,1004,1005,1006,1007,1008,1009,
1010,1011,1012,1013,1014,1015,1016,1017,1018,1019,
1020,1021,1022,1023,1024,1069,
1088,1102,1115,1125,1135,1144,1152,1160,1167,1174,1181,1187,1193,1199,
1205,1211,1216,1221,1226,1231,1236,1241,1246,1250,1255,1259,
1263,1268,1272,1276,1280,1284,1288,1292,1296,1299,
1303,1307,1310,1314,1317,1321,1324,1328,1331,1334,1338,1341,1344,1347,1350,1353,1357,1360,1363,1366,1369,
1372,1375,1377,1380,1383,1386,1389,
1392,1394,1397,1400,1403,1405,1408,1411,1413,1416,1419,
1421,1424,1426,1429,
1431,1434,1436,1439,
1441,1444,1446,1449,
1451,1453,1456,1458,1460,1463,1465,1467,1470,1472,1474,1477,1479,
1481,1483,1486,1488,1490,1492,1494,1496,1499,
1501,1503,1505,1507,1509,
1511,1514,1516,1518,1520,1522,1524,1526,1528,1530,1532,1534,1536,1538,1540,1542,1544,1546,1548,1550,1552,1554,1556,1558,1559,
1561,1563,1565,1567,1569,
1571,1573,1575,1576,1578,1580,1582,1584,1586,1587,1589,
1591,1593,1595,1596,1598,1600,1602,1604,1605,1607,1609,
1611,1612,1614,1616,1618,1619,
1621,1623,1624,1626,1628,1629,
1631,1633,1635,1636,1638,1640,1641,1643,1645,1646,1648,1649,
1651,1653,1654,1656,1658,1659,
1661,1662,1664,1666,1667,1669,
1670,1672,1674,1675,1677,1678,1680,1681,1683,1684,1686,1688,1689,
1691,1692,1694,1695,1697,1698,1700,1701,1703,1704,1706,1707,1709,
1710,1712,1713,1715,1716,1718,1719,
1721,1722,1724,1725,1727,1728,1729,
1731,1732,1734,1735,1737,1738,1740,1741,1742,1744,1745,1747,1748,1749,
1751,1752,1754,1755,1757,1758,1759,
1761,1762,1763,1765,1766,1768,1769,
1770,1772,1773,1774,1776,1777,1779,
1780,1781,1783,1784,1785,1787,1788,1789,
1791,1792,1793,1795,1796,1797,1799,
1800,1801,1803,1804,1805,1807,1808,1809,
1810,1812,1813,1814,1816,1817,1818,1820,1821,1822,1823,1825,1826,1827,1828,1830,1831,1832,1834,1835,1836,1837,1839,
1840,1841,1842,1844,1845,1846,1847,1849,
1850,1851,1852,1854,1855,1856,1857,1858,1860,1861,1862,1863,1865,1866,1867,1868,1869,
1871,1872,1873,1874,1875,1877,1878,1879,
1880,1881,1883,1884,1885,1886,1887,1889,
1890,1891,1892,1893,1894,1896,1897,1898,1899,
1900,1902,1903,1904,1905,1906,1907,1908,1910,1911,1912,1913,1914,1915,1917,1918,1919,
1920,1921,1922,1923,1925,1926,1927,1928,1929,
1930,1931,1932,1934,1935,1936,1937,1938,1939,
1940,1941,1943,1944,1945,1946,1947,1948,1949,
1950,1951,1953,1954,1955,1956,1957,1958,1959,
1960,1961,1962,1964,1965,1966,1967,1968,1969,
1970,1971,1972,1973,1974,1975,1977,1978,1979,
1980,1981,1982,1983,1984,1985,1986,1987,1988,1989,
1990,1991,1992,1994,1995,1996,1997,1998,1999,
2000,2001,2002,2003,2004,2005,2006,2007,2008,2009,
2010,2011,2012,2013,2014,2015,2017,2018,2019,
2020,2021,2022,2023,2024,2025,2026,2027,2028,2029,
2030,2031,2032,2033,2034,2035,2036,2037,2038,2039,
2040,2041,2042,2043,2044,2045,2046,2047,
};
#else
//set backlight brightness by level
static unsigned int BL_Map[257]=
{
0,128,181,221,256,286,313,338,362,384,404,424,443,461,478,495,512,527,543,557,572,
586,600,613,627,640,652,665,677,689,701,712,724,735,746,757,768,778,789,799,809,
819,829,839,849,858,868,877,886,896,905,914,923,931,940,949,957,966,974,983,991,
999,1007,1015,1024,1031,1039,1047,1055,1063,1070,1078,1086,1093,1101,1108,1115,1123,
1130,1137,1144,1152,1159,1166,1173,1180,1187,1193,1200,1207,1214,1221,1227,1234,1241,
1247,1254,1260,1267,1273,1280,1286,1292,1299,1305,1311,1317,1324,1330,1336,1342,1348,
1354,1360,1366,1372,1378,1384,1390,1396,1402,1408,1413,1419,1425,1431,1436,1442,1448,
1453,1459,1465,1470,1476,1481,1487,1492,1498,1503,1509,1514,1519,1525,1530,1536,1541,
1546,1551,1557,1562,1567,1572,1578,1583,1588,1593,1598,1603,1608,1614,1619,1624,1629,
1634,1639,1644,1649,1654,1659,1664,1668,1673,1678,1683,1688,1693,1698,1702,1707,1712,
1717,1722,1726,1731,1736,1740,1745,1750,1755,1759,1764,1768,1773,1778,1782,1787,1792,
1796,1801,1805,1810,1814,1819,1823,1828,1832,1837,1841,1846,1850,1854,1859,1863,1868,
1872,1876,1881,1885,1889,1894,1898,1902,1907,1911,1915,1920,1924,1928,1932,1936,1941,
1945,1949,1953,1958,1962,1966,1970,1974,1978,1982,1987,1991,1995,1999,2003,2007,2011,
2015,2019,2023,2027,2031,2035,2039,2043	
};
#endif
//extern unsigned int mz_get_hw_version(void);
unsigned int Cust_SetBacklight(unsigned int level)
{
	unsigned int lm_level=0;
	unsigned char lm_lsb=0;
	unsigned char lm_msb=0;

//printk("--------Cust_SetBacklight---level=%d---\n",level);
#ifdef MTK_AAL_SUPPORT
		if(level>1024)
		{
			level=1024;
		}	
#else
		if(level>255)
		{
			level=255;
		}
#endif		
		lm_level=BL_Map[level];
		lm_lsb=lm_level&0x07;
		lm_msb=(lm_level>>3)&0xFF;
		//printk("--------Cust_SetBacklight---level=%d---lm_level=%d---lsb=%X--msb=%X-----\n",level,lm_level,lm_lsb,lm_msb);
		lm3695_write_bytes(0x10, 0x17);
		/*set brightness*/
		lm3695_write_bytes(0x13, lm_lsb);
		lm3695_write_bytes(0x14, lm_msb);
		
}
#endif
//LiChao @2014.09.17 add for lcd backlight end

static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_NONE, -1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
	/* Yin ShunQing @2014-9-26 add for LED begin */
#ifdef MEIZU_M71
	{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
#else
	{"button-backlight",  MT65XX_LED_MODE_NONE, -1,{0}},
#endif
	/* Yin ShunQing @2014-9-26 add for LED end */
	//LiChao @2014.09.17 add for lcd backlight begin
#ifndef MEIZU_M71
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (int)disp_bls_set_backlight,{0}},
#else
	{"lcd-backlight",	  MT65XX_LED_MODE_CUST_LCM, (int)Cust_SetBacklight,{0}},
#endif
	//LiChao @2014.09.17 add for lcd backlight end
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

