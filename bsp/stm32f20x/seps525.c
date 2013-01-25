#include "seps525.h"
/* Thomas Tsai, thomas@life100.cc
 * www.biotrump.com
 */
//inline by compiler
#ifdef __CC_ARM                			 /* ARM Compiler,Keil MDK 	*/
#define lcd_inline   				static __inline
#elif defined (__ICCARM__)        		/* for IAR Compiler */
#define lcd_inline 					inline
#elif defined (__GNUC__)        		/* GNU GCC Compiler */
#define lcd_inline 					static __inline
#else
#define lcd_inline                  static
#endif
/********* control ***********/
#include "stm32f2xx.h"
#include "board.h"

//io redirect
#define printf               rt_kprintf
//#define printf(...)


/* RS=A0, cmd==> index port */
lcd_inline void write_cmd(rt_uint8_t cmd)
{
  /* Write 8-bit Index, then Write Reg */
  LCD->LCD_REG = cmd;
}

/* data ==> data port */
lcd_inline rt_uint8_t read_data(void)
{
	return LCD->LCD_RAM;
}

lcd_inline void write_data(rt_uint8_t data_code )
{
	/* Write 8-bit GRAM Reg */
  	LCD->LCD_RAM = data_code;
}

lcd_inline void write_reg(rt_uint8_t reg_addr, rt_uint8_t reg_val)
{
    write_cmd(reg_addr);
    write_data(reg_val);
}

lcd_inline unsigned short read_reg(rt_uint8_t reg_addr)
{
    unsigned short val=0;
    write_cmd(reg_addr);
    val = read_data();
    return (val);
}

/**
  * @brief  Prepare to write to the LCD RAM.
  * @param  None
  * @retval None
  */
lcd_inline void rw_data_prepare(void)
{
  write_cmd(LCD_REG_34);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Delay Time : 120MHZ => 1cycle = 8ns
// 1us = 120 cycles
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void uDelay(rt_uint16_t u)
{
	while(u--)
	{
		rt_uint16_t i;
		for(i=0;i<120;i++);
	};
}

/********* control  ***********/

static unsigned short deviceid=0;

/* =========================================================
 *	F01
 * =========================================================
 */

lcd_inline void Set_Clock_Control(unsigned char d)
{
				// Set Oscillator Control (OSC_CTL)
					//   Default => 0xC0
						//     EXPORT1 Pin at Low Level
						//     Oscillator operates with internal resister.
						//     Clock Off
	write_reg(LCD_REG_2,d);
}


lcd_inline void Set_Display_Clock(unsigned char d)
{
				// Set Display Frequency Divide Ration / Oscillator Frequency (CLOCK_DIV)
					//   Default => 0x30 (90 Hz / Divide Ration = 1)
						//     D[3:0] => Display Frequency Divide Ration
						//     D[7:4] => Oscillator Frequency
	write_reg(LCD_REG_3,d);
}


lcd_inline void Set_Power_Save(unsigned char d)
{
	write_reg(LCD_REG_4,d);// Set Power Save Mode (REDUCE_CURRENT)
														//   Default => 0x00
														//     Normal Driving Current
														//     Disable Oscillator Power Down
														//     Disable Power Save Mode
}

lcd_inline void Software_Reset(unsigned char d)
{
	write_reg(LCD_REG_5,d);	// Software Reset (SOFT_RST)
															//   Default => 0x00
															//     0x00 => Normal Mode
															//     0x01 => All internal register values will be default.
}

lcd_inline void Set_Display_On_Off(unsigned char d)
{
	write_reg(LCD_REG_6,d);			// Set Display On/Off (DISP_ON_OFF)
																	//   Default => 0x00
						//     Scan signal is high level at pre-charge period.
						//     Display Off
}

lcd_inline void Set_Precharge_Period(unsigned char a, unsigned char b, unsigned char c)
{
	write_reg(LCD_REG_8,a);			// Set Pre-Charge Time of Red (PRECHARGE_TIME_R)
	//   Default => 0x00
	write_reg(LCD_REG_9,b);			// Set Pre-Charge Time of Green (PRECHARGE_TIME_G)
					//   Default => 0x00
	write_reg(LCD_REG_10,c);			// Set Pre-Charge Time of Blue (PRECHARGE_TIME_B)
					//   Default => 0x00
}

lcd_inline void Set_Precharge_Current(unsigned char a, unsigned char b, unsigned char c)
{
	write_reg(LCD_REG_11,a);			// Set Pre-Charge Current of Red (PRECHARGE_CURRENT_R)
					//   Default => 0x00
	write_reg(LCD_REG_12,b);			// Set Pre-Charge Current of Green (PRECHARGE_CURRENT_G)
					//   Default => 0x00
	write_reg(LCD_REG_13,c);			// Set Pre-Charge Current of Blue (PRECHARGE_CURRENT_B)
				//   Default => 0x00
}

lcd_inline void Set_Driving_Current(unsigned char a, unsigned char b, unsigned char c)
{
	write_reg(LCD_REG_16,a);			// Set Driving Current of Red (DRIVING_CURRENT_R)
	//   Default => 0x00
	write_reg(LCD_REG_17,b);			// Set Driving Current of Green (DRIVING_CURRENT_G)
					//   Default => 0x00
	write_reg(LCD_REG_18,c);			// Set Driving Current of Blue (DRIVING_CURRENT_B)
	//   Default => 0x00
}

lcd_inline void Set_Display_Mode(unsigned char d)
{
	write_reg(LCD_REG_19,d);			// Set Column Data Display Control / Re-Map Format (DISPLAY_MODE_SET)
	//   Default => 0x00
						//     Color Sequence: R => G => B
						//     Alternative Gate Pin Configuration
						//     Scan from G0 to G[N-1]
						//     Column Address 0 Mapped to S0
						//     One Screen Mode
						//     Normal Display
}

lcd_inline void Set_RGB_IF(unsigned char d)
{
	write_reg(LCD_REG_20,d);			// Set Interface Mode (RGB_IF)
										//   Default => 0x11 (MCU Interface Mode)
}

lcd_inline void Set_RGB_POL(unsigned char d)
{
	write_reg(LCD_REG_21,d);			// Set RGB Interface Polarity (RGB_POL)
					//   Default => 0x00
						//     Enable Polarity as Active Low
						//     Dot Clock Polarity Sampled as Rising Edge
						//     Disable Vertical Synchronization Output on VSYNCO Pin
}

lcd_inline void Set_Pixel_Format(unsigned char d)
{
	write_reg(LCD_REG_21,d);			// Set Memory Access Control / Interface Pixel Format (MEMORY_WRITE_MODE)
	//   Default => 0x06
						//     Enable 18-bit Bus Interface
						//     262,144 Colors
						//     Horizontal address counter is increased.
						//     Vertical address counter is increased.
						//     The data is continuously written horizontally.
}

/*
 * MX1[7:0] / MX2[7:0] : reg23,reg24
 * Specify the horizontal start/end position of a window for access in memory. Data can be written to
 * DDRAM from the address specified by MX1[7:0] to the address specified by MX2[7:0].
 * MY1[7:0] / MY2[7:0] : reg25, reg26
 * Specify the vertical start/end position of a window for access in memory. Data can be written to
 * DDRAM from the address specified by MY1[7:0] to the address specified by MY2[7:0].
 */
lcd_inline void Set_Column_Address(unsigned char mx1, unsigned char mx2)
{
	write_reg(LCD_REG_23,mx1);		// Set Column Address of Start (MX1_ADDR)
									//   Default => 0x00
	write_reg(LCD_REG_24,mx2);		// Set Column Address of End (MX2_ADDR)
									//   Default => 0x9F
}

lcd_inline void Set_Row_Address(unsigned char my1, unsigned char my2)
{
	write_reg(LCD_REG_25,my1);	// Set Row Address of Start (MY1_ADDR)
								//   Default => 0x00
	write_reg(LCD_REG_26,my2);	// Set Row Address of End (MY2_ADDR)
								//   Default => 0x7F
}

/* Specify the Horizontal Start Position of a Window for Written in Memory (MEMORY_ACCESSPOINTER X)
 * Specify the Vertical Start Position of a Window for Written in Memory (MEMORY_ACCESSPOINTER Y)
 */
lcd_inline void Set_Display_Offset(unsigned char x, unsigned char y)
{
	write_reg(LCD_REG_32,x); //   Default => 0x00
	write_reg(LCD_REG_33,y); //   Default => 0x00
}

/*After index register 22h is select, Internal DDRAM memory can be accessed.*/
lcd_inline void Set_Write_RAM()
{
	rw_data_prepare(); 		// Internal DDRAM Memory Access (DDRAM_DATA_ACCESS_PORT)
}

lcd_inline void Set_Multiplex_Ratio(unsigned char d)
{
	write_reg(LCD_REG_40,d);			// Display Duty Ratio (DUTY)
	//   Default => 0x7F (1/128 Duty)
}

lcd_inline void Set_Start_Line(unsigned char d)
{
	write_reg(LCD_REG_41,d);			// Set Display Start Line (DSL)
										//   Default => 0x00
}

lcd_inline void Set_IREF(unsigned char d)
{
	write_reg(LCD_REG_128,d);			// Control Reference Voltage Generation (IREF)
					//   Default => 0x00 (Reference Voltage Controlled by External Resister)
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  First Screen Active Range (Partial or Full Screen)
//
//    a: Column Address of Start
//    b: Column Address of End
//    c: Row Address of Start
//    d: Row Address of End
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static void First_Screen(unsigned char a, unsigned char b, unsigned char c, unsigned char d)
{
				// Set Column Address of Start Point (D1_DDRAM_FAC)
					//   Default => 0x00
	write_reg(0x2E,a);

				// Set Row Address of Start Point (D1_DDRAM_FAR)
					//   Default => 0x9F
	write_reg(0x2F,c);

				// Set Column Address of Start (SCR1_FX1)
					//   Default => 0x00
	write_reg(LCD_REG_51,a);

				// Set Column Address of End (SCR1_FX2)
					//   Default => 0x9F
	write_reg(LCD_REG_52,b);

				// Set Row Address of Start (SCR1_FY1)
					//   Default => 0x00
	write_reg(LCD_REG_53,c);

				// Set Row Address of End (SCR1_FY2)
					//   Default => 0x7F
	write_reg(LCD_REG_54,d);
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Second Screen Active Range (Partial or Full Screen)
//
//    a: Column Address of Start
//    b: Column Address of End
//    c: Row Address of Start
//    d: Row Address of End
//    e: Column Address of Start for the Image Box
//    f: Column Address of End for the Image Box
//    g: Row Address of Start for the Image Box
//    h: Row Address of End for the Image Box
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static void Second_Screen(unsigned char a, unsigned char b, unsigned char c, unsigned char d, unsigned char e, unsigned char f, unsigned char g, unsigned char h)
{
				// Set Column Address of Start Point (D2_DDRAM_FAC)
					//   Default => 0x00
	write_reg(LCD_REG_49,a);

				// Set Row Address of Start Point (D2_DDRAM_FAR)
					//   Default => 0x9F
	write_reg(LCD_REG_50,c);

				// Set Column Address of Start (SCR2_SX1)
					//   Default => 0x00
	write_reg(LCD_REG_55,a);

				// Set Column Address of End (SCR2_SX2)
					//   Default => 0x9F
	write_reg(LCD_REG_56,b);

				// Set Row Address of Start (SCR2_SY1)
					//   Default => 0x00
	write_reg(LCD_REG_57,c);

				// Set Row Address of End (SCR2_SY2)
					//   Default => 0x7F
	write_reg(LCD_REG_58,d);

			// Set Column Address of Start for the Image Box (SS_SCR2_SX1)
				//   Default => 0x00
	write_reg(LCD_REG_71,e);

				// Set Column Address of End for the Image Box (SS_SCR2_SX2)
					//   Default => 0x00
	write_reg(LCD_REG_72,f);

			// Set Row Address of Start for the Image Box (SS_SCR2_SY1)
					//   Default => 0x00
	write_reg(LCD_REG_73,g);

				// Set Row Address of End for the Image Box (SS_SCR2_SY2)
					//   Default => 0x00
	write_reg(LCD_REG_74,h);
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Screen Saver Control (Partial or Full Screen)
//
//    a: Screen Saver Mode On/Off
//       "0x00" (Deactivate Screen Saver)
//       "0x01" (Activate Screen Saver)
//    b: Screen Saver On/Off Control
//       "0x00" (Disable First & Second Screen Saver Both)
//       "0x01" (Enable First Screen Saver Only)
//       "0x02" (Enable Second Screen Saver Only)
//       "0x03" (Enable First & Second Screen Saver Both)
//    c: Screen Saver Mode Select for the First Screen
//       "0x01" (Left Panning)
//       "0x02" (Right Panning)
//    d: Set Numbers of Scroll per Step for the First Screen
//    e: Set Time Interval between Each Step in Terms of Frame Frequency for the First Screen
//    f: Screen Saver Mode Select for the Second Screen
//       "0x00" (Box Move)
//       "0x01" (Log On)
//       "0x03" (Wrap Around)
//    g: Set Numbers of Scroll per Step for the Second Screen
//    h: Scrolling Direction for the Second Screen
//       "0x00" (Up & Leftward)
//       "0x01" (Up & Rightward)
//       "0x02" (Down & Leftward)
//       "0x03" (Down & Rightward)
//    i: Set Time Interval between Each Step in Terms of Frame Frequency for the Second Screen
//    j: Screen Saver Automatic Sleep
//       "0x00" (Disable)
//       "0x01" (First Screen Only)
//       "0x02" (Second Screen Only)
//       "0x03" (First & Second Screen Both)
//    k: Set Sleep Timer in Terms of 64 Frames Synchronization
//    l: rt_thread_delay Time
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static void Screen_Saver_Control(unsigned char a, unsigned char b, unsigned char c, unsigned char d, unsigned char e, unsigned char f, unsigned char g, unsigned char h, unsigned char i, unsigned char j, unsigned char k, unsigned char l)
{
				// Update Timer Based on Frame Synchronization (SS_SCR1_FU)
					//   Default => 0x00
	write_reg(LCD_REG_62,e);

				// Moving Step (SS_SCR1_MXY)
					//   Default => 0x00
						//     D[3:0] => Horizontal Moving Step
	write_reg(LCD_REG_63,d);

				// Update Timer Based on Frame Synchronization (SS_SCR2_FU)
					//   Default => 0x00
	write_reg(LCD_REG_64,i);

				// Moving Step (SS_SCR2_MXY)
					//   Default => 0x00
						//     D[3:0] => Horizontal Moving Step
						//     D[7:4] => Vertical Moving Step
	write_reg(LCD_REG_65,g);

				// Screen Moving Direction (MOVING_DIRECTION)
			//   Default => 0x00
	write_reg(LCD_REG_66,(h<<4)&0x30);

				// Screen Saver Sleep Timer Based on 64 Frames Synchronization (SS_SLEEP_TIMER)
					//   Default => 0x00
	write_reg(LCD_REG_60,k);

				// Screen Saver Control (SCREEN_SAVER_MODE)
			//   Default => 0x00
	write_reg(LCD_REG_61,((f<<4)&0x30)|c);

	if(b < 2)
	{
		Set_Display_Mode(0x00);
	}
	else
	{
		Set_Display_Mode(0x04);
	}

				// Screen Saver Control (SCREEN_SAVER_CONTEROL)
							//   Default => 0x00
	write_reg(LCD_REG_59,((j<<5)&0x60)|((b<<2)&0x0C)|a);

	rt_thread_delay(l);
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Fade In (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static void Fade_In()
{
	Set_Display_On_Off(0x01);
	Set_Power_Save(0x04);
	rt_thread_delay(4);		/* 40 ms, 1 tick= 10ms */
	Set_Power_Save(0x00);
	rt_thread_delay(4);		/* 40 ms, 1 tick= 10ms */
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Fade Out (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static void Fade_Out()
{
	Set_Power_Save(0x00);
	rt_thread_delay(4);		/* 40 ms, 1 tick= 10ms */
	Set_Power_Save(0x04);
	rt_thread_delay(4);		/* 40 ms, 1 tick= 10ms */
	Set_Display_On_Off(0x00);

#if 0
	WISECHIP_OLED_VCC_DIS;	/*Power Down VCC */

	rt_thread_delay(100);
	WISECHIP_OLED_VDD_DIS;
	WISECHIP_OLED_VDDIO_DIS;
#endif
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Sleep Mode
//
//    "0x00" Enter Sleep Mode
//    "0x01" Exit Sleep Mode
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static void Sleep(unsigned char a)
{
	switch(a)
	{
		case 0:
			Set_Display_On_Off(0x00);
			Set_Display_Mode(0x02);
			Set_Power_Save(0x01);
			uDelay(200);
			break;
		case 1:
			Set_Power_Save(0x00);
			uDelay(200);
			Set_Power_Save(0x01);
			uDelay(200);
			Set_Power_Save(0x00);
			uDelay(200);
			Set_Display_Mode(0x00);
			Set_Display_On_Off(0x01);
			break;
	}
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Gray Scale Table Setting (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static void Set_Gray_Scale_Table()
{
	int i=0;
	uint8_t params[]={
	0x50,
	0x00,			// Gray Scale Level 1 of Red
	0x51,
	0x01,
	0x50,
	0x01,			// Gray Scale Level 3 of Red
	0x51,
	0x04,
	0x50,
	0x02,			// Gray Scale Level 5 of Red
	0x51,
	0x06,
	0x50,
	0x03,			// Gray Scale Level 7 of Red
	0x51,
	0x0E,
	0x50,
	0x04,			// Gray Scale Level 9 of Red
	0x51,
	0x13,
	0x50,
	0x05,			// Gray Scale Level 11 of Red
	0x51,
	0x16,
	0x50,
	0x06,			// Gray Scale Level 13 of Red
	0x51,
	0x1B,
	0x50,
	0x07,			// Gray Scale Level 15 of Red
	0x51,
	0x20,
	0x50,
	0x08,			// Gray Scale Level 17 of Red
	0x51,
	0x24,
	0x50,
	0x09,			// Gray Scale Level 19 of Red
	0x51,
	0x29,
	0x50,
	0x0A,			// Gray Scale Level 21 of Red
	0x51,
	0x2D,
	0x50,
	0x0B,			// Gray Scale Level 23 of Red
	0x51,
	0x32,
	0x50,
	0x0C,			// Gray Scale Level 25 of Red
	0x51,
	0x37,
	0x50,
	0x0D,			// Gray Scale Level 27 of Red
	0x51,
	0x3C,
	0x50,
	0x0E,			// Gray Scale Level 29 of Red
	0x51,
	0x41,
	0x50,
	0x0F,			// Gray Scale Level 31 of Red
	0x51,
	0x47,
	0x50,
	0x10,			// Gray Scale Level 33 of Red
	0x51,
	0x4B,
	0x50,
	0x11,			// Gray Scale Level 35 of Red
	0x51,
	0x50,
	0x50,
	0x12,			// Gray Scale Level 37 of Red
	0x51,
	0x55,
	0x50,
	0x13,			// Gray Scale Level 39 of Red
	0x51,
	0x5A,
	0x50,
	0x14,			// Gray Scale Level 41 of Red
	0x51,
	0x60,
	0x50,
	0x15,			// Gray Scale Level 43 of Red
	0x51,
	0x64,
	0x50,
	0x16,			// Gray Scale Level 45 of Red
	0x51,
	0x69,
	0x50,
	0x17,			// Gray Scale Level 47 of Red
	0x51,
	0x6D,
	0x50,
	0x18,			// Gray Scale Level 49 of Red
	0x51,
	0x70,
	0x50,
	0x19,			// Gray Scale Level 51 of Red
	0x51,
	0x73,
	0x50,
	0x1A,			// Gray Scale Level 53 of Red
	0x51,
	0x76,
	0x50,
	0x1B,			// Gray Scale Level 55 of Red
	0x51,
	0x78,
	0x50,
	0x1C,			// Gray Scale Level 57 of Red
	0x51,
	0x79,
	0x50,
	0x1D,			// Gray Scale Level 59 of Red
	0x51,
	0x7A,
	0x50,
	0x1E,			// Gray Scale Level 61 of Red
	0x51,
	0x7B,
	0x50,
	0x1F,			// Gray Scale Level 63 of Red
	0x51,
	0x7D,
	0x50,
	0x20,			// Gray Scale Level 1 of Green
	0x51,
	0x01,
	0x50,
	0x21,			// Gray Scale Level 3 of Green
	0x51,
	0x02,
	0x50,
	0x22,			// Gray Scale Level 5 of Green
	0x51,
	0x03,
	0x50,
	0x23,			// Gray Scale Level 7 of Green
	0x51,
	0x05,
	0x50,
	0x24,			// Gray Scale Level 9 of Green
	0x51,
	0x0A,
	0x50,
	0x25,			// Gray Scale Level 11 of Green
	0x51,
	0x0F,
	0x50,
	0x26,			// Gray Scale Level 13 of Green
	0x51,
	0x14,
	0x50,
	0x27,			// Gray Scale Level 15 of Green
	0x51,
	0x19,
	0x50,
	0x28,			// Gray Scale Level 17 of Green
	0x51,
	0x1F,
	0x50,
	0x29,			// Gray Scale Level 19 of Green
	0x51,
	0x24,
	0x50,
	0x2A,			// Gray Scale Level 21 of Green
	0x51,
	0x2A,
	0x50,
	0x2B,			// Gray Scale Level 23 of Green
	0x51,
	0x2F,
	0x50,
	0x2C,			// Gray Scale Level 25 of Green
	0x51,
	0x34,
	0x50,
	0x2D,			// Gray Scale Level 27 of Green
	0x51,
	0x3A,
	0x50,
	0x2E,			// Gray Scale Level 29 of Green
	0x51,
	0x3F,
	0x50,
	0x2F,			// Gray Scale Level 31 of Green
	0x51,
	0x45,
	0x50,
	0x30,			// Gray Scale Level 33 of Green
	0x51,
	0x4B,
	0x50,
	0x31,			// Gray Scale Level 35 of Green
	0x51,
	0x50,
	0x50,
	0x32,			// Gray Scale Level 37 of Green
	0x51,
	0x55,
	0x50,
	0x33,			// Gray Scale Level 39 of Green
	0x51,
	0x5A,
	0x50,
	0x34,			// Gray Scale Level 41 of Green
	0x51,
	0x60,
	0x50,
	0x35,			// Gray Scale Level 43 of Green
	0x51,
	0x64,
	0x50,
	0x36,			// Gray Scale Level 45 of Green
	0x51,
	0x69,
	0x50,
	0x37,			// Gray Scale Level 47 of Green
	0x51,
	0x6D,
	0x50,
	0x38,			// Gray Scale Level 49 of Green
	0x51,
	0x70,
	0x50,
	0x39,			// Gray Scale Level 51 of Green
	0x51,
	0x73,
	0x50,
	0x3A,			// Gray Scale Level 53 of Green
	0x51,
	0x76,
	0x50,
	0x3B,			// Gray Scale Level 55 of Green
	0x51,
	0x78,
	0x50,
	0x3C,			// Gray Scale Level 57 of Green
	0x51,
	0x79,
	0x50,
	0x3D,			// Gray Scale Level 59 of Green
	0x51,
	0x7A,
	0x50,
	0x3E,			// Gray Scale Level 61 of Green
	0x51,
	0x7C,
	0x50,
	0x3F,			// Gray Scale Level 63 of Green
	0x51,
	0x7D,
	0x50,
	0x40,			// Gray Scale Level 1 of Blue
	0x51,
	0x01,
	0x50,
	0x41,			// Gray Scale Level 3 of Blue
	0x51,
	0x03,
	0x50,
	0x42,			// Gray Scale Level 5 of Blue
	0x51,
	0x05,
	0x50,
	0x43,			// Gray Scale Level 7 of Blue
	0x51,
	0x07,
	0x50,
	0x44,			// Gray Scale Level 9 of Blue
	0x51,
	0x0C,
	0x50,
	0x45,			// Gray Scale Level 11 of Blue
	0x51,
	0x0F,
	0x50,
	0x46,			// Gray Scale Level 13 of Blue
	0x51,
	0x13,
	0x50,
	0x47,			// Gray Scale Level 15 of Blue
	0x51,
	0x18,
	0x50,
	0x48,			// Gray Scale Level 17 of Blue
	0x51,
	0x1C,
	0x50,
	0x49,			// Gray Scale Level 19 of Blue
	0x51,
	0x21,
	0x50,
	0x4A,			// Gray Scale Level 21 of Blue
	0x51,
	0x25,
	0x50,
	0x4B,			// Gray Scale Level 23 of Blue
	0x51,
	0x2A,
	0x50,
	0x4C,			// Gray Scale Level 25 of Blue
	0x51,
	0x2F,
	0x50,
	0x4D,			// Gray Scale Level 27 of Blue
	0x51,
	0x32,
	0x50,
	0x4E,			// Gray Scale Level 29 of Blue
	0x51,
	0x37,
	0x50,
	0x4F,			// Gray Scale Level 31 of Blue
	0x51,
	0x3D,
	0x50,
	0x50,			// Gray Scale Level 33 of Blue
	0x51,
	0x41,
	0x50,
	0x51,			// Gray Scale Level 35 of Blue
	0x51,
	0x46,
	0x50,
	0x52,			// Gray Scale Level 37 of Blue
	0x51,
	0x4B,
	0x50,
	0x53,			// Gray Scale Level 39 of Blue
	0x51,
	0x50,
	0x50,
	0x54,			// Gray Scale Level 41 of Blue
	0x51,
	0x56,
	0x50,
	0x55,			// Gray Scale Level 43 of Blue
	0x51,
	0x5A,
	0x50,
	0x56,			// Gray Scale Level 45 of Blue
	0x51,
	0x60,
	0x50,
	0x57,			// Gray Scale Level 47 of Blue
	0x51,
	0x64,
	0x50,
	0x58,			// Gray Scale Level 49 of Blue
	0x51,
	0x69,
	0x50,
	0x59,			// Gray Scale Level 51 of Blue
	0x51,
	0x6D,
	0x50,
	0x5A,			// Gray Scale Level 53 of Blue
	0x51,
	0x71,
	0x50,
	0x5B,			// Gray Scale Level 55 of Blue
	0x51,
	0x74,
	0x50,
	0x5C,			// Gray Scale Level 57 of Blue
	0x51,
	0x76,
	0x50,
	0x5D,			// Gray Scale Level 59 of Blue
	0x51,
	0x79,
	0x50,
	0x5E,			// Gray Scale Level 61 of Blue
	0x51,
	0x7A,
	0x50,
	0x5F,			// Gray Scale Level 63 of Blue
	0x51,
	0x7D,
	0xFF};

	do{
		write_reg(params[i],params[i+1]);
		i+=2;
	}while(params[i] != 0xFF);
}



//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Regular Pattern (Full Screen)
//
//    a: RRRRRGGG
//    b: GGGBBBBB
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static void Fill_RAM(unsigned char a, unsigned char b)
{
unsigned char i,j;

	Set_Display_Offset(0x00,0x00);
	Set_Column_Address(0x00,LCD_WIDTH-1);
	Set_Row_Address(0x00,LCD_HEIGHT-1);
	Set_Write_RAM();

	for(i=0;i<LCD_HEIGHT;i++)
	{
		for(j=0;j<LCD_WIDTH;j++)
		{
			write_data(a);
			write_data(b);
		}
	}
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Regular Pattern (Partial or Full Screen)
//
//    a: Column Address of Start
//    b: Column Address of End
//    c: Row Address of Start
//    d: Row Address of End
//    e: RRRRRGGG
//    f: GGGBBBBB
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static void Fill_Block(unsigned char a, unsigned char b, unsigned char c, unsigned char d, unsigned char e, unsigned char f)
{
unsigned char i,j;

	Set_Display_Offset(a,c);
	Set_Column_Address(a,b);
	Set_Row_Address(c,d);
	Set_Write_RAM();

	for(i=0;i<(d-c+1);i++)
	{
		for(j=0;j<(b-a+1);j++)
		{
			write_data(e);
			write_data(f);
		}
	}
}

static void f01_test(void)
{
	while(1){
	// Fade In/Out (Full Screen)
		Fade_Out();
		Fade_In();
		Fade_Out();
		Fade_In();
		rt_thread_delay(10);

	// Scrolling (Partial or Full Screen)
		Second_Screen(0x00,0x9F,0x00,0x7F,0x00,0x9F,0x00,0x7F);
		Screen_Saver_Control(0x01,0x02,0x00,0x00,0x00,0x03,0x10,0x00,0x01,0x00,0x08,0x01);
						// Upward - Full Screen
		Screen_Saver_Control(0x01,0x02,0x00,0x00,0x00,0x03,0x10,0x02,0x01,0x00,0x08,0x01);
						// Downward - Full Screen
		Screen_Saver_Control(0x01,0x02,0x01,0x01,0x01,0x03,0x01,0x00,0x01,0x00,0x08,0x01);
						// Leftward - Full Screen
		First_Screen(0x00,0x9F,0x00,0x3B);
		Second_Screen(0x00,0x9F,0x3C,0x7F,0x20,0x7F,0x3C,0x5F);
		Screen_Saver_Control(0x01,0x01,0x01,0x01,0x01,0x03,0x01,0x00,0x01,0x00,0x08,0x01);
						// Leftward - Top Area
		Screen_Saver_Control(0x01,0x03,0x01,0x01,0x01,0x00,0x11,0x00,0x01,0x00,0x08,0x01);
						// Leftward - Top Area
						// Box Move - Bottom Area
		Screen_Saver_Control(0x01,0x03,0x01,0x01,0x01,0x01,0x11,0x00,0x04,0x00,0x08,0x01);
						// Leftward - Top Area
						// Log On - Bottom Area
		Screen_Saver_Control(0x01,0x03,0x02,0x01,0x01,0x03,0x11,0x02,0x01,0x00,0x08,0x01);
						// Rightward - Top Area
						// Wrap Around (Down & Leftward) - Bottom Area
		Screen_Saver_Control(0x01,0x03,0x02,0x01,0x01,0x03,0x11,0x03,0x01,0x00,0x08,0x01);
						// Rightward - Top Area
						// Wrap Around (Down & Rightward) - Bottom Area
		Screen_Saver_Control(0x01,0x01,0x02,0x01,0x01,0x03,0x01,0x01,0x01,0x00,0x08,0x01);
						// Rightward - Top Area
		First_Screen(0x00,0x9F,0x00,0x7F);
		Screen_Saver_Control(0x01,0x01,0x02,0x01,0x01,0x03,0x01,0x01,0x01,0x00,0x08,0x01);
						// Rightward - Full Screen
		Screen_Saver_Control(0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

	// All Pixels On (Test Pattern)
		Fill_RAM(0xFF,0xFF);
		rt_thread_delay(1);

	// Checkerboard (Test Pattern)
	//	Checkerboard();
		rt_thread_delay(1);

	// Color Bar (Test Pattern)
		//Rainbow();
		rt_thread_delay(1);

	// Show Pattern - Frame (Test Pattern)
		Fill_RAM(0x00,0x00);		// Clear Screen
//		Draw_Rectangle(0x00,Max_Column,0x00,Max_Row,0x01,0xFF,0xFF);
		rt_thread_delay(1);
		//Draw_Rectangle(0x10,0x8F,0x10,0x6F,0x01,0xF8,0x00);
		rt_thread_delay(1);
		//Draw_Rectangle(0x20,0x7F,0x20,0x5F,0x01,0x07,0xE0);
		rt_thread_delay(1);
		//Draw_Rectangle(0x30,0x6F,0x30,0x4F,0x01,0x00,0x1F);
		rt_thread_delay(1);
	}
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
unsigned int wisechip_f01_init(unsigned int lcdid)
{
	/* Power up VDD and VDDIO */
	WISECHIP_OLED_VDD_EN;
	WISECHIP_OLED_VDDIO_EN;

	/* reset LCD controller, RESETB active low */
	WISECHIP_OLED_F01_RESETB_EN;
    /* ?? tick */
   	rt_thread_delay(1);
    WISECHIP_OLED_F01_RESETB_DIS;

	Set_Power_Save(0x01);			// Set Normal Driving Current
	uDelay(200);					//     Disable Oscillator Power Down
									//     Enable Power Save Mode
	Set_Power_Save(0x00);			// Set Normal Driving Current
	uDelay(200);					//     Disable Oscillator Power Down
									//     Disable Power Save Mode
	Software_Reset(0x00);			// Set All Internal Register Value as Normal Mode
	Set_Display_On_Off(0x00);		// Display Off (0x00/0x01)
	Set_Clock_Control(0x01);		// Set EXPORT1 Pin at Internal Clock
						//     Oscillator operates with external resister.
						//     Internal Oscillator On
	Set_Display_Clock(0x30);		// Set Clock as 90 Frames/Sec
	Set_Multiplex_Ratio(0x7F);		// 1/128 Duty (0x0F~0x7F)
	Set_Display_Offset(0x00,0x00);		// Shift Mapping RAM Counter
	Set_Start_Line(0x00);			// Set Mapping RAM Display Start Line (0x00~0x7F)
	Set_RGB_IF(0x31);			// Set MCU Interface Mode
	Set_RGB_POL(0x00);			// Set RGB Interface Polarity as Active Low
						//     Dot Clock Polarity Sampled as Rising Edge
						//     Disable Vertical Synchronization Output on VSYNCO Pin
	Set_Display_Mode(0x00);			// Set Color Sequence D[15:0]=[RRRRR:GGGGGG:BBBBB]
						//     Alternative Gate Pin Configuration
						//     Scan from G0 to G127
						//     Column Address 0 Mapped to S0
						//     One Screen Mode
						//     Normal Display
	Set_Pixel_Format(0x66|((BPP<<4)&0x70));
						// Set Horizontal Address Increment
						//     Vertical Address Increment
						//     The data is continuously written horizontally.
						//     Enable 8-bit Bus Interface
						//     65,536 Colors Mode (0x66)
						//     * 262,144 Colors Mode (0x76)
	Set_Driving_Current(0x32,0x27,0x2B);	// Set Driving Current of Red
						// Set Driving Current of Green
						// Set Driving Current of Blue
	Set_Gray_Scale_Table();			// Set Pulse Width for Gamma Table
	Set_Precharge_Period(0x01,0x01,0x02);	// Set Pre-Charge Time of Red
						// Set Pre-Charge Time of Green
						// Set Pre-Charge Time of Blue
	Set_Precharge_Current(0x0C,0x19,0x15);	// Set Pre-Charge Current of Red
						// Set Pre-Charge Current of Green
						// Set Pre-Charge Current of Blue
	Set_IREF(0x00);				// Set Reference Voltage Controlled by External Resister
	First_Screen(0x00,0x9F,0x00,0x7F);

	Fill_RAM(0x00,0x00);			// Clear Screen

	/*Power up VCC*/
	WISECHIP_OLED_VCC_EN;	//VDDH
	rt_thread_delay(10);	/* 100ms */

	Set_Display_On_Off(0x01);		// Display On (0x00/0x01)
	return 0;
}


/* ======================================================== */
void seps525_init(void)
{
    deviceid = read_reg(LCD_REG_0);
    /* deviceid check */
    printf("\r\nLCD Device ID : %04X ",deviceid);
	wisechip_f01_init(deviceid); /* wisechip F01 oled panel */

//	f01_test();
//    lcd_gram_test();
}


//static unsigned short BGR2RGB(unsigned short c)
//{
//    u16  r, g, b, rgb;
//
//    b = (c>>0)  & 0x1f;
//    g = (c>>5)  & 0x3f;
//    r = (c>>11) & 0x1f;
//
//    rgb =  (b<<11) + (g<<5) + (r<<0);
//
//    return( rgb );
//}

/**
  * @brief  Configures LCD Control lines (FSMC Pins) in alternate function mode.
  * @param  None
  * @retval None
  */
void LCD_CtrlLinesConfig(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;

  	/* Enable FSMC clock */
  	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
  	/* Enable GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOE |
                         RCC_AHB1Periph_GPIOF, ENABLE);

	/*-- GPIO Configuration ------------------------------------------------------*/
  	/* SRAM Data lines,  NOE and NWE configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_14 | GPIO_Pin_15 |
                                GPIO_Pin_4 |GPIO_Pin_5;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);

	/* SRAM Address lines configuration, PF0 = A0 ==>LCD_RS */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_FSMC);

	/* NE4 configuration,PG12 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_FSMC);
}

/**
  * @brief  Configures the Parallel interface (FSMC) for LCD(Parallel mode)
  * @param  None
  * @retval None
  */
void LCD_FSMCConfig(void)
{
  	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  	FSMC_NORSRAMTimingInitTypeDef  Timing_read, Timing_write;

	/*-- FSMC Configuration ------------------------------------------------------*/
	/*----------------------- SRAM Bank 4 ----------------------------------------*/
    //FSMC_NORSRAMStructInit(&FSMC_NORSRAMInitStructure); /* !!! WEIRD! this will make system hardware fault !!! */
  	/* FSMC_Bank1_NORSRAM4 configuration */
  	Timing_read.FSMC_AddressSetupTime = 0x8;
  	Timing_read.FSMC_AddressHoldTime = 0;
  	Timing_read.FSMC_DataSetupTime = 0x8;
  	Timing_read.FSMC_BusTurnAroundDuration = 0;
  	Timing_read.FSMC_CLKDivision = 0;
  	Timing_read.FSMC_DataLatency = 0;
  	Timing_read.FSMC_AccessMode = FSMC_AccessMode_B;

  	Timing_write.FSMC_AddressSetupTime = 0x1;
  	Timing_write.FSMC_AddressHoldTime = 0;
  	Timing_write.FSMC_DataSetupTime = 0x2;
  	Timing_write.FSMC_BusTurnAroundDuration = 0;
  	Timing_write.FSMC_CLKDivision = 0;
  	Timing_write.FSMC_DataLatency = 0;
  	Timing_write.FSMC_AccessMode = FSMC_AccessMode_B;

  	/* Color LCD configuration ------------------------------------
  	   LCD configured as follow:
  	      - Data/Address MUX = Disable
  	      - Memory Type = SRAM
  	      - Data Width = 16bit
  	      - Write Operation = Enable
  	      - Extended Mode = Enable
  	      - Asynchronous Wait = Disable */

  	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
  	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;//FSMC_ExtendedMode_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &Timing_read;
  	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &Timing_write;

  	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);
  	/* BANK 4 (of NOR/SRAM Bank 1~4) is enabled */
  	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

/* setup FSMC pins and timing */
static void LCD_FSMC_init(void)
{
	LCD_CtrlLinesConfig();	/* enabling FSMC pins*/
	LCD_FSMCConfig();     	/*-- FSMC timing Configuration */

}

static void lcd_port_init(void)
{
    LCD_FSMC_init();

    seps525_init();	/* OLED controller and panel */
}


static unsigned short lcd_read_gram(unsigned int x,unsigned int y)
{
    unsigned short temp;

	Set_Display_Offset(x,y);	/* x,y to write */
	Set_Column_Address(x,x);	/* window x1,x2*/
	Set_Row_Address(y,y);		/* window y1,y2 */
	Set_Write_RAM();			/* start to wrie gram */

	temp =((read_data() << 8) | read_data());
	return temp;
}

#if 0
static void lcd_data_bus_test(void)
{

    unsigned short temp1;
    unsigned short temp2;

    /* wirte */
    lcd_SetCursor(0,0);
    rw_data_prepare();
    write_data(0x5555);

    lcd_SetCursor(1,0);
    rw_data_prepare();
    write_data(0xAAAA);

    /* read */
    lcd_SetCursor(0,0);
    temp1 = lcd_read_gram(0,0);
    temp2 = lcd_read_gram(1,0);

    if( (temp1 == 0x5555) && (temp2 == 0xAAAA) )
    {
        printf(" data bus test pass!\r\n");
    }
    else
    {
        printf(" data bus test error: %04X %04X\r\n",temp1,temp2);
    }

}
#endif

/* color,X,Y
 * Color Sequence D[15:0]=[RRRRR:GGGGGG:BBBBB]
 * Don't use this routine, because the very high overhead to locate a pos(x,y),
 * unless these is only a pixel to write.
 */
void seps525_lcd_set_pixel(const char* pixel, int x, int y)
{
	rt_uint16_t p=*(rt_uint16_t*)pixel;
//    lcd_SetCursor(x,y);
	Set_Display_Offset(x,y);	/* x,y to write */
	Set_Column_Address(x,x);	/* window x1,x2*/
	Set_Row_Address(y,y);		/* window y1,y2 */
	Set_Write_RAM();			/* start to wrie gram */

	/* 8 bits bus, so 16bits pixels need 2 operations */
	write_data((p>>8)&0x00FF);	// Line Color - RRRRRGGG
	write_data(*pixel);	// Line Color - GGGBBBBB
}

void seps525_lcd_get_pixel(char* pixel, int x, int y)
{
	*(rt_uint16_t*)pixel = lcd_read_gram(x, y);
}

void seps525_lcd_draw_hline(const char* pixel, int x1, int x2, int y)
{
	rt_uint16_t p=*(rt_uint16_t*)pixel;

	Set_Display_Offset(x1, y);	/*start pos to write */
	Set_Column_Address(x1, x2);	/* */
	Set_Row_Address(y,y);
	Set_Write_RAM();			/* write enabling */
    while (x1 <= x2){
	   	write_data((p>>8)&0x00FF);	// high byte : Line Color - RRRRRGGG
		write_data(*pixel);	// low byte :Line Color - GGGBBBBB
        x1++;
	}
}

void seps525_lcd_draw_vline(const char* pixel, int x, int y1, int y2)
{
	rt_uint16_t p=*(rt_uint16_t*)pixel;

	Set_Display_Offset(x, y1);	/*start pos to write */
	Set_Column_Address(x, x);	/* */
	Set_Row_Address(y1,y2);
	Set_Write_RAM();			/* write enabling */
    while (y1++ <= y2){
	   	write_data((p>>8)&0x00FF);	/* high byte : Line Color - RRRRRGGG*/
		write_data(*pixel);			/* low byte :Line Color - GGGBBBBB*/
	}
}

/* blit a line */
void seps525_lcd_blit_line(const char* pixels, int x, int y, rt_size_t size)
{
	rt_uint16_t *ptr;
	ptr = (rt_uint16_t*)pixels;

	if( (x < LCD_WIDTH) && (y < LCD_HEIGHT) ){
		size = ((x+size -1) < LCD_WIDTH) ? (size):(LCD_WIDTH-x-1);
		Set_Display_Offset(x, y);	/*start pos to write */
		Set_Column_Address(x, x+size-1);	/* */
		Set_Row_Address(y,y);
		Set_Write_RAM();			/* write enabling */
    	while (size--){
		   	write_data( (*ptr>>8)&0x00FF);	// high byte : Line Color - RRRRRGGG
			write_data( *ptr++ &0x00FF);	// low byte :Line Color - GGGBBBBB
		}
	}
}

struct rt_device_graphic_ops seps525_ops =
{
	seps525_lcd_set_pixel,
	seps525_lcd_get_pixel,
	seps525_lcd_draw_hline,
	seps525_lcd_draw_vline,
	seps525_lcd_blit_line
};


struct rt_device _lcd_device;

static rt_err_t lcd_init(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t lcd_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t lcd_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t lcd_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	switch (cmd)
	{
	case RTGRAPHIC_CTRL_GET_INFO:
		{
			struct rt_device_graphic_info *info;

			info = (struct rt_device_graphic_info*) args;
			RT_ASSERT(info != RT_NULL);

			info->bits_per_pixel = BPP;
			info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565P;
			info->framebuffer = RT_NULL;
			info->width = LCD_WIDTH;
			info->height = LCD_HEIGHT;
		}
		break;

	case RTGRAPHIC_CTRL_RECT_UPDATE:
		/* nothong to be done */
		break;

	default:
		break;
	}

	return RT_EOK;
}

void rt_hw_lcd_init(void)
{
	/* GPIO control pins
	PC0		O	LCD_VCC_ENABLE
	PC1 	O	LCD_VDD_ENABLE
	PC13	O	LCD_VDDIO_ENABLE (optional)
	PC4 	O	LCD_RESETB				active low
	Enable GPIOC clocks */
 	GPIO_InitTypeDef GPIO_InitStructure;
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_13 | GPIO_Pin_4 ;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);

	lcd_port_init();

	/* register lcd device */
	_lcd_device.type  = RT_Device_Class_Graphic;
	_lcd_device.init  = lcd_init;
	_lcd_device.open  = lcd_open;
	_lcd_device.close = lcd_close;
	_lcd_device.control = lcd_control;
	_lcd_device.read  = RT_NULL;
	_lcd_device.write = RT_NULL;

	_lcd_device.user_data = &seps525_ops;
    /* register graphic device driver */
	rt_device_register(&_lcd_device, "lcd",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
}


