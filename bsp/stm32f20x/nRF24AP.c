#include <stdbool.h>
#include "stm32f2xx.h"

#include "board.h"
#include "nRF24AP.h"

#include <rtthread.h>
#include <rtgui/event.h>
#include <rtgui/kbddef.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>

/* mcu side , hw Slave mode
MOSI 		PA7
MISO 		PA6
CLK  		PA5

nSRDY   	PA4	 O		software NSS
nMRDY		PF7	 O
nSE			PF8	 I
nRESET		PF9	 O
nPOWER_EN	PF10 O
*/

#define   CS_0()          GPIO_ResetBits(GPIOC,GPIO_Pin_4)
#define   CS_1()          GPIO_SetBits(GPIOC,GPIO_Pin_4)

struct rtgui_touch_device
{
    struct rt_device parent;

    rt_timer_t poll_timer;
    rt_uint16_t x, y;

    rt_bool_t calibrating;
    rt_touch_calibration_func_t calibration_func;

    rt_uint16_t min_x, max_x;
    rt_uint16_t min_y, max_y;
};
static struct rtgui_touch_device *touch = RT_NULL;

extern unsigned char SPI_WriteByte(unsigned char data);
rt_inline void EXTI_Enable(rt_uint32_t enable);

struct rt_semaphore spi1_lock;	/* TODO TODO : why not MUTEX */

void rt_hw_spi1_baud_rate(uint16_t SPI_BaudRatePrescaler)
{
    SPI1->CR1 &= ~SPI_BaudRatePrescaler_256;
    SPI1->CR1 |= SPI_BaudRatePrescaler;
}

uint8_t SPI_WriteByte(unsigned char data)
{
    //Wait until the transmit buffer is empty
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    // Send the byte
    SPI_I2S_SendData(SPI1, data);

    //Wait until a data is received
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    // Get the received data
    data = SPI_I2S_ReceiveData(SPI1);

    // Return the shifted data
    return data;
}

//SPI写数据
static void WriteDataTo7843(unsigned char num)
{
    SPI_WriteByte(num);
}

#define X_WIDTH 240
#define Y_WIDTH 320

static void rtgui_touch_calculate()
{
}

void touch_timeout(void* parameter)
{
    static unsigned int touched_down = 0;
    struct rtgui_event_mouse emouse;
    static struct _touch_previous
    {
        rt_uint32_t x;
        rt_uint32_t y;
    } touch_previous;

    /* touch time is too short and we lost the position already. */
    if ((!touched_down) && GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) != 0)
        return;

    if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) != 0)
    {
        int tmer = RT_TICK_PER_SECOND/8 ;
        EXTI_Enable(1);
        emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
        emouse.button = (RTGUI_MOUSE_BUTTON_LEFT |RTGUI_MOUSE_BUTTON_UP);

        /* use old value */
        emouse.x = touch->x;
        emouse.y = touch->y;

        /* stop timer */
        rt_timer_stop(touch->poll_timer);
        rt_kprintf("touch up: (%d, %d)\n", emouse.x, emouse.y);
        touched_down = 0;

        if ((touch->calibrating == RT_TRUE) && (touch->calibration_func != RT_NULL))
        {
            /* callback function */
            touch->calibration_func(emouse.x, emouse.y);
        }
        rt_timer_control(touch->poll_timer , RT_TIMER_CTRL_SET_TIME , &tmer);
    }
    else
    {
        if(touched_down == 0)
        {
            int tmer = RT_TICK_PER_SECOND/20 ;
            /* calculation */
            rtgui_touch_calculate();

            /* send mouse event */
            emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
            emouse.parent.sender = RT_NULL;

            emouse.x = touch->x;
            emouse.y = touch->y;

            touch_previous.x = touch->x;
            touch_previous.y = touch->y;

            /* init mouse button */
            emouse.button = (RTGUI_MOUSE_BUTTON_LEFT |RTGUI_MOUSE_BUTTON_DOWN);

//            rt_kprintf("touch down: (%d, %d)\n", emouse.x, emouse.y);
            touched_down = 1;
            rt_timer_control(touch->poll_timer , RT_TIMER_CTRL_SET_TIME , &tmer);
        }
        else
        {
            /* calculation */
            rtgui_touch_calculate();

#define previous_keep      8
            //判断移动距离是否小于previous_keep,减少误动作.
            if(
                (touch_previous.x<touch->x+previous_keep)
                && (touch_previous.x>touch->x-previous_keep)
                && (touch_previous.y<touch->y+previous_keep)
                && (touch_previous.y>touch->y-previous_keep)  )
            {
                return;
            }

            touch_previous.x = touch->x;
            touch_previous.y = touch->y;

            /* send mouse event */
            emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON ;
            emouse.parent.sender = RT_NULL;

            emouse.x = touch->x;
            emouse.y = touch->y;

            /* init mouse button */
            emouse.button = (RTGUI_MOUSE_BUTTON_RIGHT |RTGUI_MOUSE_BUTTON_DOWN);
//            rt_kprintf("touch motion: (%d, %d)\n", emouse.x, emouse.y);
        }
    }

    /* send event to server */
    if (touch->calibrating != RT_TRUE)
        rtgui_server_post_event(&emouse.parent, sizeof(struct rtgui_event_mouse));
}

static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

rt_inline void EXTI_Enable(rt_uint32_t enable)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure  EXTI  */
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//Falling下降沿 Rising上升

    if (enable)
    {
        /* enable */
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    }
    else
    {
        /* disable */
        EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    }

    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(EXTI_Line1);
}

static void EXTI_Configuration(void)
{
    /* PB1 touch INT */
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOB,&GPIO_InitStructure);
    }

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

    /* Configure  EXTI  */
    EXTI_Enable(1);
}

/* RT-Thread Device Interface */
static rt_err_t nRF24AP_init (rt_device_t dev)
{
    NVIC_Configuration();
    EXTI_Configuration();

    /* PC4 touch CS */
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
        GPIO_Init(GPIOC,&GPIO_InitStructure);
        CS_1();
    }

    CS_0();
    WriteDataTo7843( 1<<7 ); /* 打开中断 */
    CS_1();

    return RT_EOK;
}

static rt_err_t rtgui_touch_control (rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
    case RT_TOUCH_CALIBRATION:
        touch->calibrating = RT_TRUE;
        touch->calibration_func = (rt_touch_calibration_func_t)args;
        break;

    case RT_TOUCH_NORMAL:
        touch->calibrating = RT_FALSE;
        break;

    case RT_TOUCH_CALIBRATION_DATA:
    {
        struct calibration_data* data;

        data = (struct calibration_data*) args;

        //update
        touch->min_x = data->min_x;
        touch->max_x = data->max_x;
        touch->min_y = data->min_y;
        touch->max_y = data->max_y;
    }
    break;
    }

    return RT_EOK;
}

void EXTI1_IRQHandler(void)
{
    /* disable interrupt */
    EXTI_Enable(0);

    /* start timer */
    rt_timer_start(touch->poll_timer);

    EXTI_ClearITPendingBit(EXTI_Line1);
}

static void SPI_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	/* Enable SPI1 Periph clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,
	                       ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_SPI1,
							ENABLE);

	/* Configure SPI1 pins: PA5-SCK, PA6-MISO and PA7-MOSI
	 * Connect SPI pins to AF5 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*spi1, slave mode3, polarity1*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; /* 120mhz / 256, it's slave mode, so it may not need this setting */
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;	/* */
	SPI_I2S_DeInit(SPI1);
	SPI_Init(SPI1, &SPI_InitStructure);

	/* Enable slave */
	SPI_Cmd(SPI1, ENABLE);
	/*SPI_CalculateCRC(SPI1, DISABLE);no need for slave mode*/
	/* Enable the Rx buffer not empty interrupt */
	SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);
}

rt_inline void EXTI_Enable(rt_uint32_t enable)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure  EXTI  */
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

    if (enable)
    {
        /* enable */
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    }
    else
    {
        /* disable */
        EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    }

    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(EXTI_Line1);
}

static void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
/*
nSRDY   	PF4	 O		software NSS
nMRDY		PF7	 O
nSE			PF8	 I
nRESET		PF9	 O
nPOWER_EN	PF10 O
*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/* Configure PF8 pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/*GPIO_Mode_IPU*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/* Connect ANT_nSE to PF8 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource8);

	/* Configure EXTI ANT_nSE */
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void nRF24AP_hw_init(void)
{
	SPI_Config();
	GPIO_Config();


	if (rt_sem_init(&spi1_lock, "spi1lock", 1, RT_IPC_FLAG_FIFO) != RT_EOK)
	{
	    rt_kprintf("init spi1 lock semaphore failed\n");
	}
    touch = (struct rtgui_touch_device*)rt_malloc (sizeof(struct rtgui_touch_device));
    if (touch == RT_NULL) return; /* no memory yet */

    /* clear device structure */
    rt_memset(&(touch->parent), 0, sizeof(struct rt_device));
    touch->calibrating = false;

    /* init device structure */
    touch->parent.type = RT_Device_Class_Unknown;
    touch->parent.init = nRF24AP_init;
    touch->parent.control = rtgui_touch_control;
    touch->parent.user_data = RT_NULL;


    /* register touch device to RT-Thread */
    rt_device_register(&(touch->parent), "touch", RT_DEVICE_FLAG_RDWR);
}

#ifdef RT_USING_FINSH
#include <finsh.h>

void touch_t( rt_uint16_t x , rt_uint16_t y )
{
    struct rtgui_event_mouse emouse ;
    emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
    emouse.parent.sender = RT_NULL;

    emouse.x = x ;
    emouse.y = y ;
    /* init mouse button */
    emouse.button = (RTGUI_MOUSE_BUTTON_LEFT |RTGUI_MOUSE_BUTTON_DOWN );
    rtgui_server_post_event(&emouse.parent, sizeof(struct rtgui_event_mouse));

    rt_thread_delay(2) ;
    emouse.button = (RTGUI_MOUSE_BUTTON_LEFT |RTGUI_MOUSE_BUTTON_UP );
    rtgui_server_post_event(&emouse.parent, sizeof(struct rtgui_event_mouse));
}

FINSH_FUNCTION_EXPORT(touch_t, x & y ) ;
#endif
