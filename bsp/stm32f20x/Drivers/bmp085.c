/*
 * File      : bmp085.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 * 2013 www.biotrump.com
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-01-05     thomas tsai      First version
 */
#include <rtconfig.h>
#ifdef BMP085_SUPPORT
#include <rtthread.h>
#include "i2c.h"

#define BMP085_I2C_ADDR 	(0xEE)

#define BMP085_ULTRALOWPOWER 	(0)
#define BMP085_STANDARD      	(1)
#define BMP085_HIGHRES       	(2)
#define BMP085_ULTRAHIGHRES  	(3)
#define BMP085_CAL_AC1          (0xAA)  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2          (0xAC)  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3          (0xAE)  // R   Calibration data (16 bits)
#define BMP085_CAL_AC4          (0xB0)  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5          (0xB2)  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6          (0xB4)  // R   Calibration data (16 bits)
#define BMP085_CAL_B1           (0xB6)  // R   Calibration data (16 bits)
#define BMP085_CAL_B2           (0xB8)  // R   Calibration data (16 bits)
#define BMP085_CAL_MB           (0xBA)  // R   Calibration data (16 bits)
#define BMP085_CAL_MC           (0xBC)  // R   Calibration data (16 bits)
#define BMP085_CAL_MD           (0xBE)  // R   Calibration data (16 bits)

#define BMP085_CONTROL          (0xF4)
#define BMP085_READTEMPCMD      (0x2E)
#define BMP085_READPRESSURECMD  (0x34)	/* osrs=0 */

#define BMP085_TEMPDATA			(0xF6)
#define BMP085_PRESSUREDATA     (0xF6)
#define	printk	rt_kprintf

rt_uint8_t control_data= (0x34); /* default is pressure measurement */
const uint8_t BMP_PRESSURE_OSRS_VAL[]={0x34, 0x74, 0xB4, 0xF4};

struct rt_bmp085_dev{
	struct rt_device dev;
	rt_int16_t ac1;
	rt_int16_t ac2;
	rt_int16_t ac3;
	rt_uint16_t ac4;
	rt_uint16_t ac5;
	rt_uint16_t ac6;
	rt_int16_t b1;
	rt_int16_t b2;
	rt_int16_t mb;
	rt_int16_t mc;
	rt_int16_t md;
	uint8_t bmp085_osrs;
};

static struct rt_bmp085_dev bmp085_dev;

void usDelay(uint32_t d)
{
	int i;
	while(d--)
		for (i =0; i < 120; i++); /* 120mhz , 1 us delay */
}

/* normal res 2 bytes:0xF6,0xF7
 * extra 	3 bytes		0xF8
 */
static uint32_t bmp085_ReadBuffer(void *pBuffer, rt_off_t ReadAddr, rt_size_t NumByteToRead)
{
	return I2C_IORW(I2C1, (uint8_t *)pBuffer, (uint16_t)NumByteToRead, (uint16_t)ReadAddr, BMP085_I2C_ADDR | 0x01, I2C_MEM_1Byte );
}

/* control register : 0xF4
*/
uint32_t bmp085_WriteByte(void *pBuffer, uint16_t WriteAddr)
{
	I2C_IORW(I2C1, (uint8_t *)pBuffer, 1 , WriteAddr, BMP085_I2C_ADDR, I2C_MEM_1Byte );

	/*if( I2C_AcknowledgePolling(I2C1 , BMP085_I2C_ADDR) == Error )
		printk("EE ACK failed\n");*/
	//rt_thread_delay(1);

	return 0;
}

static rt_err_t bmp085_init(rt_device_t dev)
{
	return RT_EOK;
}

/*
* normal  2 bytes
* high res 3 bytes
*/
static rt_size_t bmp085_read(rt_device_t dev, rt_off_t pos, void *buf, rt_size_t size)
{
	struct rt_bmp085_dev *bmp085_dev = (struct rt_bmp085_dev *)dev;
	if(dev == (rt_device_t)0) return -1;

	/* start conversion */
	bmp085_WriteByte((void *)&BMP_PRESSURE_OSRS_VAL[bmp085_dev->bmp085_osrs], BMP085_CONTROL);

	size = (size > 2)?3:size;
	if (bmp085_ReadBuffer(buf, pos, size) == Success)
		return size;
	else
		return -1;
}

static rt_size_t bmp085_write(rt_device_t dev, rt_off_t pos, const void *buf, rt_size_t size)
{
/*	if (bmp085_WriteBuffer(buf, pos, size) == Success)
		return size;
	else*/
		return -1;
}

static rt_err_t bmp085_open(rt_device_t dev, rt_uint16_t oflag)
{
	/* start conversion */
	return RT_EOK;
}

static rt_err_t bmp085_close(rt_device_t dev)
{
	/* stop conversion */
	return RT_EOK;
}

static rt_err_t bmp085_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
/* control register 0xf4, conversion by the following setting
temperature			0x2e	4.5ms
pressure(osrs=0)	0x34	4.5ms
pressure(osrs=1)	0x74	7.5ms
pressure(osrs=2)	0xb4	13.5ms
pressure(osrs=3)	0xf4	25.5
*/
	/*mutex lock */
	switch(cmd){
	case 0:
		control_data = 0x2E;
		break;
	case 2:
		control_data = 0x74;
		break;
	case 3:
		control_data = 0xB4;
		break;
	case 4:
		control_data = 0xF4;
		break;
	case 1:
	default:
		control_data = 0x34;
		break;
	}
	/*mutex unlock */
	return RT_EOK;
}

/**
  * @brief  Returns the temperature value (in 16 bit format).
  * @param  None
  * @retval The temperature value.
  */
uint16_t BMP085_GetRawTemperature(struct rt_bmp085_dev *dev)
{
 	uint16_t tmp = 0;
	printk(">>>BMP085_GetRawTemperature\r\n");
  	/* temperature operation enable */
	tmp = BMP085_READTEMPCMD;
  	bmp085_WriteByte((void *)&tmp, BMP085_CONTROL);
	rt_thread_delay(1);

  	bmp085_ReadBuffer(&tmp, BMP085_TEMPDATA, 2);
	printk("<<<BMP085_GetRawTemperature = %d \r\n", tmp);
  	/* return the temperature row value */
  	return tmp;
}

/**
  * @brief  Returns the pressure value (in 16 bit format).
  * @param  None
  * @retval The pressure value.
  */
uint32_t BMP085_GetRawPressure(struct rt_bmp085_dev *dev)
{
 	uint16_t pressure = 0;
	printk(">>>BMP085_GetRawPressure\r\n");
  	/* pressure operation enable */
  	bmp085_WriteByte((void *)&BMP_PRESSURE_OSRS_VAL[dev->bmp085_osrs], BMP085_CONTROL);
	rt_thread_delay(1);
	/* 2 bytes */
  	bmp085_ReadBuffer(&pressure, BMP085_PRESSUREDATA, 2);
	printk("<<<BMP085_GetRawPressure = 0x%x\r\n", pressure);
  	/* return the temperature row value */
  	return (uint32_t)pressure;
}

/*
 * in 0.1C
 *
 */
uint16_t BMP085_CalTemperature(struct rt_bmp085_dev *dev)
{
	int32_t X1, X2, B5;
	uint16_t ut = BMP085_GetRawTemperature(dev);
	int32_t temperature=0;

	X1 = ((int32_t)ut - dev->ac6) * dev->ac5 >> 15;
	X2 = ((int32_t)dev->mc << 11) / (X1 + dev->md);
	B5 = X1 + X2;
	temperature = (B5 + 8) >> 4;	/* in 0.1 C unit */
	printk("BMP085_CalTemperature = %ld/10 C\r\n", temperature);
	return temperature;
}

/*
 * pressure in Pascal (Pa)
 * 1 Pa	กÝ 1 Newton/m^2
 * 1 hPa = 100Pa
 * 1 bar = 10^5 Pa = 1000hPa
 * 1 atm = 1.01325 กั10^5 Pa = 1013.325 hPa
 */
int32_t BMP085_CalPressure(struct rt_bmp085_dev *dev)
{
	int32_t pressure;
	int32_t temperature;
	int32_t X1, X2, B5, B6, X3, B3, p;
	uint32_t B4, B7;
	int32_t ut = BMP085_GetRawTemperature(dev);
	int32_t up = BMP085_GetRawPressure(dev);

	printk(">>>BMP085_CalPressure ut=%d,up=%d\r\n", ut, up);
	X1 = ((int32_t)ut - dev->ac6) * dev->ac5 >> 15;
	X2 = ((int32_t)dev->mc << 11) / (X1 + dev->md);
	B5 = X1 + X2;
	temperature = (B5 + 8) >> 4;	/* in 0.1 C unit */

	B6 = B5 - 4000;
	X1 = (dev->b2 * (B6 * B6 >> 12)) >> 11;
	X2 = dev->ac2 * B6 >> 11;
	X3 = X1 + X2;
	B3 = (((int32_t)dev->ac1 * 4 + X3) + 2)/4;
	X1 = dev->ac3 * B6 >> 13;
	X2 = (dev->b1 * (B6 * B6 >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = (dev->ac4 * (uint32_t) (X3 + 32768)) >> 15;
	B7 = ((uint32_t) up - B3) * (50000 >> dev->bmp085_osrs);
	if( B7 < 0x80000000)
	    p = (B7 * 2) / B4 ;
	else
		p = (B7 / B4) * 2;

	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;
	pressure = p + ((X1 + X2 + 3791) >> 4);

	printk("<<<BMP085_CalPressure %ld Pa\r\n", pressure);
	return pressure;
}

static void init_param(void)
{
	bmp085_ReadBuffer(&bmp085_dev.ac1, BMP085_CAL_AC1, 2);
	printk("\nac1=0x%x\n",bmp085_dev.ac1);
	bmp085_ReadBuffer(&bmp085_dev.ac2, BMP085_CAL_AC2, 2 );
	printk("ac2=0x%x\n",bmp085_dev.ac2);
	bmp085_ReadBuffer(&bmp085_dev.ac3, BMP085_CAL_AC3, 2 );
	printk("ac3=0x%x\n",bmp085_dev.ac3);
	bmp085_ReadBuffer(&bmp085_dev.ac4, BMP085_CAL_AC4, 2 );
	printk("ac4=0x%x\n",bmp085_dev.ac4);
	bmp085_ReadBuffer(&bmp085_dev.ac5, BMP085_CAL_AC5, 2 );
	printk("ac5=0x%x\n",bmp085_dev.ac5);
	bmp085_ReadBuffer(&bmp085_dev.ac6, BMP085_CAL_AC6, 2 );
	printk("ac6=0x%x\n",bmp085_dev.ac6);
	bmp085_ReadBuffer(&bmp085_dev.b1, BMP085_CAL_B1, 2 );
	printk("b1=0x%x\n",bmp085_dev.b1);
	bmp085_ReadBuffer(&bmp085_dev.b2, BMP085_CAL_B2, 2 );
	printk("b2=0x%x\n",bmp085_dev.b2);
	bmp085_ReadBuffer(&bmp085_dev.mb, BMP085_CAL_MB, 2 );
	printk("mb=0x%x\n",bmp085_dev.mb);
	bmp085_ReadBuffer(&bmp085_dev.mc, BMP085_CAL_MC, 2 );
	printk("dev->mc=0x%x\n",bmp085_dev.mc);
	bmp085_ReadBuffer(&bmp085_dev.md, BMP085_CAL_MD, 2 );
	printk("dev->md=0x%x\n",bmp085_dev.md);
}

void bmp085_hw_init(void)
{
	I2C1_INIT();

	bmp085_dev.dev.init 	= bmp085_init;
	bmp085_dev.dev.open 	= bmp085_open;
	bmp085_dev.dev.close	= bmp085_close;
	bmp085_dev.dev.read 	= bmp085_read;
	bmp085_dev.dev.write	= bmp085_write;
	bmp085_dev.dev.control	= bmp085_control;
	bmp085_dev.dev.type 	= RT_Device_Class_Unknown;

	init_param();
	BMP085_CalPressure(&bmp085_dev);

	rt_device_register(&bmp085_dev.dev, "bmp085", RT_DEVICE_FLAG_RDWR);
}

void dump_bmp085(void)
{
	init_param();
}

void bmp085_reset(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	/* XCLR, PB5, to reset bmp085, active low */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* pull low > 5 us */
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	usDelay(10);
	/* pull high*/
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
}

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(bmp085_reset, bmp085 bosch air presure);
FINSH_FUNCTION_EXPORT(dump_bmp085, bosch bmp085 air pressure);
#endif

#endif
