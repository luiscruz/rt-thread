/*******************************************************************************
// GY-51 LSM303DLH IIC���Գ���
// ʹ�õ�Ƭ��STM32F103C8T6
// ����8.00M
// ��ʾ��pc��������	�����ʣ�115200
// ���뻷�� Keil uVision4
// ʱ�䣺2011��9��1��
// QQ��531389319
// ��ģ������ GPIOB6->SCL GPIOB7->SDA
// ʹ��	STM32F103C8T6����1
*******************************************************************************/

#include "stm32f10x_lib.h"
#include  <math.h>    //Keil library
//#include  <stdio.h>   //Keil library
GPIO_InitTypeDef GPIO_InitStructure;
ErrorStatus HSEStartUpStatus;

#define   uchar unsigned char
#define   uint unsigned int

 //�ų��ڲ��Ĵ���***********************************
/* Magnetometer registers */
#define CRA_REG_M		0x00	/* Configuration register A */
#define CRB_REG_M		0x01	/* Configuration register B */
#define MR_REG_M		0x02	/* Mode register */
/* resume state index */
#define RES_CRA_REG_M		0	/* Configuration register A */
#define RES_CRB_REG_M		1	/* Configuration register B */
#define RES_MR_REG_M		2	/* Mode register */
/* Output register start address*/
#define OUT_X_M			    0x03
/* Magnetic Sensor Operation Mode */
#define NORMAL_MODE     	0x00
#define POS_BIAS         	0x01
#define NEG_BIAS         	0x02
#define CC_MODE          	0x00
#define SC_MODE			    0x01
#define SLEEP_MODE		    0x03
 //���ٶ��ڲ��Ĵ���***********************************
#define AXISDATA_REG	  	    0x28
#define WHOAMI_LSM303DLH_ACC	0x32	/*	Expctd content for WAI	*/
/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define CTRL_REG1		0x20	/*				*/
#define CTRL_REG2		0x21	/*				*/
#define CTRL_REG3		0x22	/*				*/
#define CTRL_REG4		0x23	/*				*/
#define	CTRL_REG5		0x24	/*				*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/

#define	INT_CFG2		0x34	/*	interrupt 2 config	*/
#define	INT_SRC2		0x35	/*	interrupt 2 source	*/
#define	INT_THS2		0x36	/*	interrupt 2 threshold	*/
#define	INT_DUR2		0x37	/*	interrupt 2 duration	*/

#define	M_SlaveAddress    0x3C
#define	A_SlaveAddress    0x30

unsigned char TX_DATA[5]; 		  //���ڷ�������
unsigned char BUF[8];             //�������ݻ�����
char  test=0; 					  //I2Cʹ��
short M_x,M_y,M_z,A_x,A_y,A_z;	  //�ų�����ٶȵ�ԭʼֵ
float angle;					  //�ų�����ĽǶ�
 //************************************++++++++++++++++++++++++++++++++
/*ģ��IIC�˿�������붨��*/
#define SCL_H         GPIOB->BSRR = GPIO_Pin_6
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6

#define SDA_H         GPIOB->BSRR = GPIO_Pin_7
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7

#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GPIOB->IDR  & GPIO_Pin_7

/* �������� -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART1_Configuration(void);
void WWDG_Configuration(void);
void Delay(u32 nTime);
void Delayms(vu32 m);

/* �������� ----------------------------------------------*/

  /*******************************/
void DATA_printf(uchar *s,short temp_data)
{
	if(temp_data<0){
	temp_data=-temp_data;
    *s='-';
	}
	else *s=' ';
	*++s =temp_data/1000+0x30;
    temp_data=temp_data%1000;     //ȡ������
    *++s =temp_data/100+0x30;
    temp_data=temp_data%100;     //ȡ������
    *++s =temp_data/10+0x30;
    temp_data=temp_data%10;      //ȡ������
    *++s =temp_data+0x30;
}
/*******************************************************************************
* Function Name  : I2C_GPIO_Config
* Description    : Configration Simulation IIC GPIO
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  /* Configure I2C1 pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{

   u8 i=30; //��������Ż��ٶ�	����������͵�5����д��
   while(i)
   {
     i--;
   }
}

void delay5ms(void)
{

   int i=5000;
   while(i)
   {
     i--;
   }
}
/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather	 Start
****************************************************************************** */
bool I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return FALSE;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	SDA_L;
	I2C_delay();
	if(SDA_read) return FALSE;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	SDA_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
bool I2C_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
{
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
      SCL_L;
	  I2C_delay();
      return FALSE;
	}
	SCL_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(u8 SendByte) //���ݴӸ�λ����λ//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;
      else
        SDA_L;
        SendByte<<=1;
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
}
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //���ݴӸ�λ����λ//
{
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;
    while(i--)
    {
      ReceiveByte<<=1;
      SCL_L;
      I2C_delay();
	  SCL_H;
      I2C_delay();
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
}
//ZRX
//���ֽ�д��*******************************************

bool Single_WriteLSM303D(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//���ø���ʼ��ַ+������ַ
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte(REG_Address );   //���õ���ʼ��ַ
    I2C_WaitAck();
    I2C_SendByte(REG_data);
    I2C_WaitAck();
    I2C_Stop();
    return TRUE;

}
 //���ֽڶ�ȡ*****************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   unsigned char REG_data;
	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//���ø���ʼ��ַ+������ַ
    if(!I2C_WaitAck()){I2C_Stop();test=1; return FALSE;}
    I2C_SendByte((u8) REG_Address);   //���õ���ʼ��ַ
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

	REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
	return REG_data;
}

//������ȡ6������*****************************************
Multiple_read(unsigned char SlaveAddress,unsigned char REG_Address)
{   char i;
	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//���ø���ʼ��ַ+������ַ
    if(!I2C_WaitAck()){I2C_Stop();test=1; return FALSE;}
    I2C_SendByte((u8) REG_Address);   //������ʼ��ַ
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

	for (i=0; i<6; i++)                //������ȡ6����ַ���ݣ��洢��BUF
    {
        BUF[i] = I2C_RadeByte();       //BUF[]�洢
        if (i == 5)I2C_NoAck();        //���һ��������Ҫ��NOACK
          else I2C_Ack();              //��ӦACK
    }
    I2C_Stop();
    //return TRUE;
}

//��ʼ��LSM303D��������Ҫ��ο������ֲᣬ�����޸�************************
void InitLSM303D(void)
{
	Single_WriteLSM303D(M_SlaveAddress,0x00,0x14);   //
    Single_WriteLSM303D(M_SlaveAddress,0x02,0x00);   //
	Single_WriteLSM303D(A_SlaveAddress,0x20,0x27);   //������Χ,����2g��16λģʽ
}

/*
********************************************************************************
** �������� �� RCC_Configuration(void)
** �������� �� ʱ�ӳ�ʼ��
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
void RCC_Configuration(void)
{
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
   /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO  , ENABLE);
}
/*
********************************************************************************
** �������� �� GPIO_Configuration(void)
** �������� �� �˿ڳ�ʼ��
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE  );
   /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //	ѡ�йܽ�9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		 // �����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // ����������50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);				 // ѡ��A�˿�

  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			  //ѡ�йܽ�10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);				  //ѡ��A�˿�

}

/*
********************************************************************************
** �������� �� USART1_Configuration(void)
** �������� �� ����1��ʼ��
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
void USART1_Configuration(void)
{

USART_InitTypeDef USART_InitStructure;
USART_ClockInitTypeDef  USART_ClockInitStructure;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 |RCC_APB2Periph_USART1, ENABLE  );

USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;			// ʱ�ӵ͵�ƽ�
USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;				// ʱ�ӵ͵�ƽ
USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;				// ʱ�ӵڶ������ؽ������ݲ���
USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;		// ���һλ���ݵ�ʱ�����岻��SCLK���
/* Configure the USART1 synchronous paramters */
USART_ClockInit(USART1, &USART_ClockInitStructure);					// ʱ�Ӳ�����ʼ������

USART_InitStructure.USART_BaudRate = 115200;						  // ������Ϊ��115200
USART_InitStructure.USART_WordLength = USART_WordLength_8b;			  // 8λ����
USART_InitStructure.USART_StopBits = USART_StopBits_1;				  // ��֡��β����1��ֹͣλ
USART_InitStructure.USART_Parity = USART_Parity_No ;				  // ��żʧ��
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// Ӳ��������ʧ��

USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		  // ����ʹ��+����ʹ��
/* Configure USART1 basic and asynchronous paramters */
USART_Init(USART1, &USART_InitStructure);

  /* Enable USART1 */
USART_ClearFlag(USART1, USART_IT_RXNE); 			//���жϣ�����һ�����жϺ����������ж�
USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);		//ʹ��USART1�ж�Դ
USART_Cmd(USART1, ENABLE);							//USART1�ܿ��أ�����
}

/*
********************************************************************************
** �������� �� NVIC_Configuration(void)
** �������� �� �жϳ�ʼ��
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);

}

 /*
********************************************************************************
** �������� �� WWDG_Configuration(void)
** �������� �� ���Ź���ʼ��
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
void WWDG_Configuration(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
  WWDG_SetPrescaler(WWDG_Prescaler_8);	              //  WWDG clock counter = (PCLK1/4096)/8 = 244 Hz (~4 ms)
  WWDG_SetWindowValue(0x41);		                 // Set Window value to 0x41
  WWDG_Enable(0x50);		       // Enable WWDG and set counter value to 0x7F, WWDG timeout = ~4 ms * 64 = 262 ms
  WWDG_ClearFlag();			       // Clear EWI flag
  WWDG_EnableIT();			       // Enable EW interrupt
}

/*
********************************************************************************
** �������� �� Delay(vu32 nCount)
** �������� �� ��ʱ����
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
 void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}

/*
********************************************************************************
** �������� �� void Delayms(vu32 m)
** �������� �� ����ʱ����	 m=1,��ʱ1ms
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
 void Delayms(vu32 m)
{
  u32 i;

  for(; m != 0; m--)
       for (i=0; i<50000; i++);
}

/*
********************************************************************************
** �������� �� WWDG_IRQHandler(void)
** �������� �� ������ǰ�����ж�
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/

void WWDG_IRQHandler(void)
{
  /* Update WWDG counter */
  WWDG_SetCounter(0x50);

  /* Clear EWI flag */
  WWDG_ClearFlag();

}
//***********�������ڷ�������*****************************
void  USART1_SendData(uchar SendData)
{
USART_SendData(USART1, SendData);
Delayms(1);
}
 //********���ڷ�������***********************************
 void Send_data(uchar axis)
 {uchar i;
  USART1_SendData(axis);
  USART1_SendData(':');
  for(i=0;i<5;i++)USART1_SendData(TX_DATA[i]);
  USART1_SendData(' ');
  USART1_SendData(' ');
 }

/*
********************************************************************************
** �������� �� main(void)
** �������� �� ������
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
int main(void)
{
  RCC_Configuration();
  GPIO_Configuration();
  USART1_Configuration();
  I2C_GPIO_Config();
  InitLSM303D();
  while(1)
  {
	 //�ų����ݶ�ȡ�뷢��
      Multiple_read(M_SlaveAddress,0x03);//OUT_X_M
      M_x=(BUF[0] << 8) | BUF[1]; //�ϳ�16λ����
      M_y=(BUF[2] << 8) | BUF[3]; //�ϳ�16λ����
	  M_z=(BUF[4] << 8) | BUF[5]; //�ϳ�16λ����
      angle= atan2(M_y,M_x) * (180 / 3.14159265) + 180; // angle in degrees
	  DATA_printf(TX_DATA,angle);//ת���Ƕ������ݵ�����
	  Send_data('M');	         //�������ݣ�M���ų�0-360

	  //���ٶ����ݶ�ȡ�뷢��
	  BUF[0]=Single_Read(A_SlaveAddress,0x28);//OUT_X_A
	  BUF[1]=Single_Read(A_SlaveAddress,0x29);//OUT_X_A

	  BUF[2]=Single_Read(A_SlaveAddress,0x2A);//OUT_Y_A
	  BUF[3]=Single_Read(A_SlaveAddress,0x2B);//OUT_Y_A

	  BUF[4]=Single_Read(A_SlaveAddress,0x2C);//OUT_Z_A
	  BUF[5]=Single_Read(A_SlaveAddress,0x2D);//OUT_Z_A

	  A_x=(BUF[1] << 8) | BUF[0]; //�ϳ�16λ����
      A_y=(BUF[3] << 8) | BUF[2]; //�ϳ�16λ����
	  A_z=(BUF[5] << 8) | BUF[4]; //�ϳ�16λ����
	  A_x/=16.383;  //mg
	  A_y/=16.383;  //mg
	  A_z/=16.383;  //mg
	  DATA_printf(TX_DATA,A_x); //ת��X�����ݵ�����
	  Send_data('x');		   	//�������ݣ�mgΪ��λ
	  DATA_printf(TX_DATA,A_y); //ת��X�����ݵ�����
	  Send_data('y');			//�������ݣ�mgΪ��λ
	  DATA_printf(TX_DATA,A_z); //ת��X�����ݵ�����
	  Send_data('z');			//�������ݣ�mgΪ��λ
	  USART1_SendData(0X0D);	//����
	  USART1_SendData(0X0A);	//�س�
	  Delayms(1);
  }
}

/*************����***************/
