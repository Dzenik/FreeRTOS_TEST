
#include "stm32f10x.h"
#include "stm32f10x_fsmc.h"
#include "ministm32_lcd.h"
#include "lcd_font.h"

u16 LCD_DeviceCode;

/* LCD is connected to the FSMC_Bank1_NOR/SRAM2 and NE2 is used as ship select signal */
/* RS <==> A2 */
#define LCD_REG              (*((volatile unsigned short *) 0x6C000000)) /* RS = 0 */
#define LCD_RAM              (*((volatile unsigned short *) 0x6C000002)) /* RS = 1 */


static void fsmc_gpio_init(void)
/* FSMC GPIO configure */
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF
                           | RCC_APB2Periph_GPIOG, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /*
    FSMC_D0 ~ FSMC_D3
    PD14 FSMC_D0   PD15 FSMC_D1   PD0  FSMC_D2   PD1  FSMC_D3
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD,&GPIO_InitStructure);

    /*
    FSMC_D4 ~ FSMC_D12
    PE7 ~ PE15  FSMC_D4 ~ FSMC_D12
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10
                                  | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOE,&GPIO_InitStructure);

    /* FSMC_D13 ~ FSMC_D15   PD8 ~ PD10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOD,&GPIO_InitStructure);

    /*
    FSMC_A0 ~ FSMC_A5   FSMC_A6 ~ FSMC_A9
    PF0     ~ PF5       PF12    ~ PF15
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
                                  | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOF,&GPIO_InitStructure);

    /* FSMC_A10 ~ FSMC_A15  PG0 ~ PG5 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOG,&GPIO_InitStructure);

    /* FSMC_A16 ~ FSMC_A18  PD11 ~ PD13 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOD,&GPIO_InitStructure);

    /* RD-PD4 WR-PD5 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOD,&GPIO_InitStructure);

    /* NBL0-PE0 NBL1-PE1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOE,&GPIO_InitStructure);

    /* NE1/NCE2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOD,&GPIO_InitStructure);
    /* NE2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOG,&GPIO_InitStructure);
    /* NE3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOG,&GPIO_InitStructure);
    /* NE4 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOG,&GPIO_InitStructure);
}

void LCD_FSMCConfig(void)
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  Timing_read,Timing_write;

    /*-- FSMC Configuration -------------------------------------------------*/
    Timing_read.FSMC_AddressSetupTime = 30;             /* µØÖ·½¨Á¢Ê±¼ä  */
    Timing_read.FSMC_DataSetupTime = 30;                /* Êý¾Ý½¨Á¢Ê±¼ä  */
    Timing_read.FSMC_AccessMode = FSMC_AccessMode_A;    /* FSMC ·ÃÎÊÄ£Ê½ */

    Timing_write.FSMC_AddressSetupTime = 3;             /* µØÖ·½¨Á¢Ê±¼ä  */
    Timing_write.FSMC_DataSetupTime = 3;                /* Êý¾Ý½¨Á¢Ê±¼ä  */
    Timing_write.FSMC_AccessMode = FSMC_AccessMode_A;   /* FSMC ·ÃÎÊÄ£Ê½ */

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
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &Timing_read;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &Timing_write;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

    /* BANK 4 (of NOR/SRAM Bank 1~4) is enabled */
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

void LCD_Configuration(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Open the corresponding clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);  
	/* All Lcd pin configured as push-pull output */
	/* 16-bit data low 8 bits */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* High 8-bit 16-bit data */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* Pin control */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* Backlight control */
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	Lcd_Light_ON;
} 

/****************************************************************************
* Name: void LCD_Initializtion ()
* Function: initialize ILI9320 controller
* Entrance Parameters: None
* Output Parameters: None
* Note:
* Call the method: LCD_Initializtion ();
****************************************************************************/
void LCD_Initializtion()
{
  /*****************************
  **  the hardware connection **
  ** STM32         ili9320    **
  ** PE0~15 <----> DB0~15     **
  ** PD15   <----> nRD        **
  ** PD14   <----> RS         **
  ** PD13   <----> nWR        **
  ** PD12   <----> nCS        **
  ** PD11   <----> nReset     **
  ** PC0    <----> BK_LED     **
  ******************************/
 	u16 i;

    /* Configure the LCD Control pins --------------------------------------------*/
    fsmc_gpio_init();

    /* Configure the FSMC Parallel interface -------------------------------------*/
    LCD_FSMCConfig();

    LCD_WriteData(0xffff);
  	Set_Rst;
	Set_nWr;
	Set_Cs;
	Set_Rs;
	Set_nRd;
	Set_Rst;
  LCD_Reset();   // reset LCD_Reset
  for(i=50000;i>0;i--);
  for(i=50000;i>0;i--);
  for(i=50000;i>0;i--);
  LCD_WriteRegister(0x0000,0x0001);LCD_Delay(10000);
  for(i=50000;i>0;i--);
  for(i=50000;i>0;i--);
  LCD_DeviceCode = LCD_ReadRegister(0x0000);

  	if(LCD_DeviceCode==0x9325||LCD_DeviceCode==0x9328)
	{
  		LCD_WriteRegister(0x00e7,0x0010);      
        LCD_WriteRegister(0x0000,0x0001);  			//start internal osc
        LCD_WriteRegister(0x0001,0x0100);     
        LCD_WriteRegister(0x0002,0x0700); 				//power on sequence                     
        LCD_WriteRegister(0x0003,(1<<12)|(1<<5)|(1<<4) ); 	//65K 
        LCD_WriteRegister(0x0004,0x0000);                                   
        LCD_WriteRegister(0x0008,0x0207);	           
        LCD_WriteRegister(0x0009,0x0000);         
        LCD_WriteRegister(0x000a,0x0000); 				//display setting         
        LCD_WriteRegister(0x000c,0x0001);				//display setting          
        LCD_WriteRegister(0x000d,0x0000); 				//0f3c          
        LCD_WriteRegister(0x000f,0x0000);
		//Power On sequence //
        LCD_WriteRegister(0x0010,0x0000);   
        LCD_WriteRegister(0x0011,0x0007);
        LCD_WriteRegister(0x0012,0x0000);                                                                 
        LCD_WriteRegister(0x0013,0x0000);                 
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteRegister(0x0010,0x1590);   
        LCD_WriteRegister(0x0011,0x0227);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteRegister(0x0012,0x009c);                  
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteRegister(0x0013,0x1900);   
        LCD_WriteRegister(0x0029,0x0023);
        LCD_WriteRegister(0x002b,0x000e);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteRegister(0x0020,0x0000);                                                            
        LCD_WriteRegister(0x0021,0x0000);           
///////////////////////////////////////////////////////      
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteRegister(0x0030,0x0007); 
        LCD_WriteRegister(0x0031,0x0707);   
        LCD_WriteRegister(0x0032,0x0006);
        LCD_WriteRegister(0x0035,0x0704);
        LCD_WriteRegister(0x0036,0x1f04); 
        LCD_WriteRegister(0x0037,0x0004);
        LCD_WriteRegister(0x0038,0x0000);        
        LCD_WriteRegister(0x0039,0x0706);     
        LCD_WriteRegister(0x003c,0x0701);
        LCD_WriteRegister(0x003d,0x000f);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteRegister(0x0050,0x0000);        
        LCD_WriteRegister(0x0051,0x00ef);   
        LCD_WriteRegister(0x0052,0x0000);     
        LCD_WriteRegister(0x0053,0x013f);
        LCD_WriteRegister(0x0060,0xa700);        
        LCD_WriteRegister(0x0061,0x0001); 
        LCD_WriteRegister(0x006a,0x0000);
        LCD_WriteRegister(0x0080,0x0000);
        LCD_WriteRegister(0x0081,0x0000);
        LCD_WriteRegister(0x0082,0x0000);
        LCD_WriteRegister(0x0083,0x0000);
        LCD_WriteRegister(0x0084,0x0000);
        LCD_WriteRegister(0x0085,0x0000);
      
        LCD_WriteRegister(0x0090,0x0010);     
        LCD_WriteRegister(0x0092,0x0000);  
        LCD_WriteRegister(0x0093,0x0003);
        LCD_WriteRegister(0x0095,0x0110);
        LCD_WriteRegister(0x0097,0x0000);        
        LCD_WriteRegister(0x0098,0x0000);  
         //display on sequence     
        LCD_WriteRegister(0x0007,0x0133);
    
        LCD_WriteRegister(0x0020,0x0000);                                                            
        LCD_WriteRegister(0x0021,0x0000);
	}
	else if(LCD_DeviceCode==0x9320)
	{
		LCD_WriteRegister(0x00,0x0000);
		LCD_WriteRegister(0x01,0x0100);	//Driver Output Contral.
		LCD_WriteRegister(0x02,0x0700);	//LCD Driver Waveform Contral.
//		LCD_WriteRegister(0x03,0x1030);	//Entry Mode Set.
		LCD_WriteRegister(0x03,0x1018);	//Entry Mode Set.
	
		LCD_WriteRegister(0x04,0x0000);	//Scalling Contral.
		LCD_WriteRegister(0x08,0x0202);	//Display Contral 2.(0x0207)
		LCD_WriteRegister(0x09,0x0000);	//Display Contral 3.(0x0000)
		LCD_WriteRegister(0x0a,0x0000);	//Frame Cycle Contal.(0x0000)
		LCD_WriteRegister(0x0c,(1<<0));	//Extern Display Interface Contral 1.(0x0000)
		LCD_WriteRegister(0x0d,0x0000);	//Frame Maker Position.
		LCD_WriteRegister(0x0f,0x0000);	//Extern Display Interface Contral 2.
	
		for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
		LCD_WriteRegister(0x07,0x0101);	//Display Contral.
		for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
	
		LCD_WriteRegister(0x10,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));	//Power Control 1.(0x16b0)
		LCD_WriteRegister(0x11,0x0007);								//Power Control 2.(0x0001)
		LCD_WriteRegister(0x12,(1<<8)|(1<<4)|(0<<0));					//Power Control 3.(0x0138)
		LCD_WriteRegister(0x13,0x0b00);								//Power Control 4.
		LCD_WriteRegister(0x29,0x0000);								//Power Control 7.
	
		LCD_WriteRegister(0x2b,(1<<14)|(1<<4));
		
		LCD_WriteRegister(0x50,0);		//Set X Start.
		LCD_WriteRegister(0x51,239);	//Set X End.
		LCD_WriteRegister(0x52,0);		//Set Y Start.
		LCD_WriteRegister(0x53,319);	//Set Y End.
	
		LCD_WriteRegister(0x60,0x2700);	//Driver Output Control.
		LCD_WriteRegister(0x61,0x0001);	//Driver Output Control.
		LCD_WriteRegister(0x6a,0x0000);	//Vertical Srcoll Control.
	
		LCD_WriteRegister(0x80,0x0000);	//Display Position? Partial Display 1.
		LCD_WriteRegister(0x81,0x0000);	//RAM Address Start? Partial Display 1.
		LCD_WriteRegister(0x82,0x0000);	//RAM Address End-Partial Display 1.
		LCD_WriteRegister(0x83,0x0000);	//Displsy Position? Partial Display 2.
		LCD_WriteRegister(0x84,0x0000);	//RAM Address Start? Partial Display 2.
		LCD_WriteRegister(0x85,0x0000);	//RAM Address End? Partial Display 2.
	
		LCD_WriteRegister(0x90,(0<<7)|(16<<0));	//Frame Cycle Contral.(0x0013)
		LCD_WriteRegister(0x92,0x0000);	//Panel Interface Contral 2.(0x0000)
		LCD_WriteRegister(0x93,0x0001);	//Panel Interface Contral 3.
		LCD_WriteRegister(0x95,0x0110);	//Frame Cycle Contral.(0x0110)
		LCD_WriteRegister(0x97,(0<<8));	//
		LCD_WriteRegister(0x98,0x0000);	//Frame Cycle Contral.

	
		LCD_WriteRegister(0x07,0x0173);	//(0x0173)
	}
	else if(LCD_DeviceCode==0x9919)
	{
		//------POWER ON &RESET DISPLAY OFF
		 LCD_WriteRegister(0x28,0x0006);
		
		 LCD_WriteRegister(0x00,0x0001);
		
		 LCD_WriteRegister(0x10,0x0000);
		
		 LCD_WriteRegister(0x01,0x72ef);

		 LCD_WriteRegister(0x02,0x0600);

		 LCD_WriteRegister(0x03,0x6a38);
		
		 LCD_WriteRegister(0x11,0x6874);//70
		
		 
		     //  RAM WRITE DATA MASK
		 LCD_WriteRegister(0x0f,0x0000); 
		    //  RAM WRITE DATA MASK
		 LCD_WriteRegister(0x0b,0x5308); 
		
		 LCD_WriteRegister(0x0c,0x0003);
		
		 LCD_WriteRegister(0x0d,0x000a);
		
		 LCD_WriteRegister(0x0e,0x2e00);  //0030
		
		 LCD_WriteRegister(0x1e,0x00be);
		
		 LCD_WriteRegister(0x25,0x8000);
		
		 LCD_WriteRegister(0x26,0x7800);
		
		 LCD_WriteRegister(0x27,0x0078);
		
		 LCD_WriteRegister(0x4e,0x0000);
		
		 LCD_WriteRegister(0x4f,0x0000);
		
		 LCD_WriteRegister(0x12,0x08d9);
		
		 // -----------------Adjust the Gamma Curve----//
		 LCD_WriteRegister(0x30,0x0000);	 //0007
		
		 LCD_WriteRegister(0x31,0x0104);	   //0203
		
		 LCD_WriteRegister(0x32,0x0100);		//0001

		 LCD_WriteRegister(0x33,0x0305);	  //0007

		 LCD_WriteRegister(0x34,0x0505);	  //0007
		
		 LCD_WriteRegister(0x35,0x0305);		 //0407
		
		 LCD_WriteRegister(0x36,0x0707);		 //0407
		
		 LCD_WriteRegister(0x37,0x0300);		  //0607
		
		 LCD_WriteRegister(0x3a,0x1200);		 //0106
		
		 LCD_WriteRegister(0x3b,0x0800);		 

		 LCD_WriteRegister(0x07,0x0033);
	} 

	else if(LCD_DeviceCode==0x4531)	   // Mini-STM32 LCD controller that have
	{		
		// Setup display
		LCD_WriteRegister(0x00,0x0001);
	    LCD_WriteRegister(0x10,0x0628);
	    LCD_WriteRegister(0x12,0x0006);
	    LCD_WriteRegister(0x13,0x0A32);
	    LCD_WriteRegister(0x11,0x0040);
	    LCD_WriteRegister(0x15,0x0050);
	    LCD_WriteRegister(0x12,0x0016);
	    LCD_Delay(15);
	    LCD_WriteRegister(0x10,0x5660);
	    LCD_Delay(15);
	    LCD_WriteRegister(0x13,0x2A4E);
	    LCD_WriteRegister(0x01,0x0100);
	    LCD_WriteRegister(0x02,0x0300);
	
	    LCD_WriteRegister(0x03,0x1030);
//	    LCD_WriteRegister(0x03,0x1018);		// freeRTOS demo
	
	    LCD_WriteRegister(0x08,0x0202);
	    LCD_WriteRegister(0x0A,0x0000);
	    LCD_WriteRegister(0x30,0x0000);
	    LCD_WriteRegister(0x31,0x0402);
	    LCD_WriteRegister(0x32,0x0106);
	    LCD_WriteRegister(0x33,0x0700);
	    LCD_WriteRegister(0x34,0x0104);
	    LCD_WriteRegister(0x35,0x0301);
	    LCD_WriteRegister(0x36,0x0707);
	    LCD_WriteRegister(0x37,0x0305);
	    LCD_WriteRegister(0x38,0x0208);
	    LCD_WriteRegister(0x39,0x0F0B);
	    LCD_Delay(15);
	    LCD_WriteRegister(0x41,0x0002);
	    LCD_WriteRegister(0x60,0x2700);
	    LCD_WriteRegister(0x61,0x0001);
	    LCD_WriteRegister(0x90,0x0119);
	    LCD_WriteRegister(0x92,0x010A);
	    LCD_WriteRegister(0x93,0x0004);
	    LCD_WriteRegister(0xA0,0x0100);
//	    LCD_WriteRegister(0x07,0x0001);
	    LCD_Delay(15);
//	    LCD_WriteRegister(0x07,0x0021); 
	    LCD_Delay(15);
//	    LCD_WriteRegister(0x07,0x0023);
	    LCD_Delay(15);
//	    LCD_WriteRegister(0x07,0x0033);
	    LCD_Delay(15);
	    LCD_WriteRegister(0x07,0x0133);
	    LCD_Delay(15);
	    LCD_WriteRegister(0xA0,0x0000);
	    LCD_Delay(20);
	} 									   
	else if(LCD_DeviceCode==0x7783)
	{
		// Start Initial Sequence
		LCD_WriteRegister(0x00FF,0x0001);
		LCD_WriteRegister(0x00F3,0x0008);
		LCD_WriteRegister(0x0001,0x0100);
		LCD_WriteRegister(0x0002,0x0700);
		LCD_WriteRegister(0x0003,0x1030);  //0x1030
		LCD_WriteRegister(0x0008,0x0302);
		LCD_WriteRegister(0x0008,0x0207);
		LCD_WriteRegister(0x0009,0x0000);
		LCD_WriteRegister(0x000A,0x0000);
		LCD_WriteRegister(0x0010,0x0000);  //0x0790
		LCD_WriteRegister(0x0011,0x0005);
		LCD_WriteRegister(0x0012,0x0000);
		LCD_WriteRegister(0x0013,0x0000);
		LCD_Delay(50);
		LCD_WriteRegister(0x0010,0x12B0);
		LCD_Delay(50);
		LCD_WriteRegister(0x0011,0x0007);
		LCD_Delay(50);
		LCD_WriteRegister(0x0012,0x008B);
		LCD_Delay(50);
		LCD_WriteRegister(0x0013,0x1700);
		LCD_Delay(50);
		LCD_WriteRegister(0x0029,0x0022);
		
		//################# void Gamma_Set(void) ####################//
		LCD_WriteRegister(0x0030,0x0000);
		LCD_WriteRegister(0x0031,0x0707);
		LCD_WriteRegister(0x0032,0x0505);
		LCD_WriteRegister(0x0035,0x0107);
		LCD_WriteRegister(0x0036,0x0008);
		LCD_WriteRegister(0x0037,0x0000);
		LCD_WriteRegister(0x0038,0x0202);
		LCD_WriteRegister(0x0039,0x0106);
		LCD_WriteRegister(0x003C,0x0202);
		LCD_WriteRegister(0x003D,0x0408);
		LCD_Delay(50);
		
		
		LCD_WriteRegister(0x0050,0x0000);		
		LCD_WriteRegister(0x0051,0x00EF);		
		LCD_WriteRegister(0x0052,0x0000);		
		LCD_WriteRegister(0x0053,0x013F);		
		LCD_WriteRegister(0x0060,0xA700);		
		LCD_WriteRegister(0x0061,0x0001);
		LCD_WriteRegister(0x0090,0x0033);				
		LCD_WriteRegister(0x002B,0x000B);		
		LCD_WriteRegister(0x0007,0x0133);
		LCD_Delay(50);
	}  
	
  for(i=50000;i>0;i--);
  LCD_Clear(0x00f8);
  LCD_Clear(0xf800);
}

/****************************************************************************
* Name: void LCD_SetCursor (u16 x, u16 y)
* Function: Set the screen coordinates
* Entry parameters: x-line coordinates
* 					y column coordinate
* Output Parameters: None
* Note:
* Call the method: LCD_SetCursor (10,10);
****************************************************************************/
__inline void LCD_SetCursor(u16 x,u16 y)
{
	if(LCD_DeviceCode==0x8989)
	{
	 	LCD_WriteRegister(0x004e,y);        //ok
    	LCD_WriteRegister(0x004f,0x13f-x);  //columns
	}
	else if(LCD_DeviceCode==0x9919)
	{
		LCD_WriteRegister(0x004e,x); // ok
  		LCD_WriteRegister(0x004f,y); // columns	
	}
	else
	{
  		LCD_WriteRegister(0x0020,y); // ok
  		LCD_WriteRegister(0x0021,0x13f-x); // columns
	}
}

/****************************************************************************
* Name: void LCD_SetWindows (u16 StartX, u16 StartY, u16 EndX, u16 EndY)
* Function: Set the window region
* Entry parameters: StartX line start coordinates
* StartY out initial coordinates
* EndX line end coordinates
* EndY column coordinates of the end
* Output Parameters: None
* Note:
* Call the method: LCD_SetWindows (0,0,100,100);
****************************************************************************/
__inline void LCD_SetWindows(u16 StartX,u16 StartY,u16 EndX,u16 EndY)
{
  LCD_SetCursor(StartX,StartY);
  LCD_WriteRegister(0x0050, StartX);
  LCD_WriteRegister(0x0052, StartY);
  LCD_WriteRegister(0x0051, EndX);
  LCD_WriteRegister(0x0053, EndY);
}

/****************************************************************************
* Name: void LCD_Clear (u16 bkColor)
* Feature: the screen fills to the specified color, such as clear screen, then fill 0xffff
* Entry parameters: dat fill value
* Output Parameters: None
* Note:
* Call the method: LCD_Clear (0xffff);
****************************************************************************/
void LCD_Clear(u16 bkColor)
{
  u32  i;

  LCD_SetCursor(0x0000, 0x0000);  
  Clr_Cs; 
  LCD_WriteIndex(0x0022); 
  Set_Rs;   
  for(i=0;i<76800;i++)
  {
    LCD_WriteData(bkColor);
	Clr_nWr;
	Set_nWr;
  }

  Set_Cs;
}

/****************************************************************************
* Name: u16 LCD_GetPoint (u16 x, u16 y)
* Function: Get the color value of the specified coordinates
* Entry parameters: x-line coordinates
* Y column coordinate
* Export parameters: the current color value coordinates
* Note:
* Call the method: i = LCD_GetPoint (10,10);
****************************************************************************/
//u16 LCD_GetPoint(u16 x,u16 y)
//{
//  LCD_SetCursor(x,y);
//  return (LCD_BGR2RGB(LCD_ReadRegister(0x0022)));
//}


u16 LCD_GetPoint(u16 x,u16 y)
{
  u16 temp;
  LCD_SetCursor(x,y);
  Clr_Cs;
  LCD_WriteIndex(0x0022);
   Clr_nRd;  
	temp = LCD_ReadData(); //dummy
   Set_nRd;   
   Clr_nRd;  
	temp = LCD_ReadData(); 
   Set_nRd;	
   Set_Cs;
   if(LCD_DeviceCode!=0x7783&&LCD_DeviceCode!=0x4531)
   	  temp=LCD_BGR2RGB(temp);

   return (temp);
}

/****************************************************************************
* Name: void LCD_SetPoint (u16 x, u16 y, u16 point)
* Function: draw point at the specified coordinates
* Entry parameters: x-line coordinates
* Y column coordinate
* Point point color
* Output Parameters: None
* Note:
* Call the method: LCD_SetPoint (10,10,0 x0fe0);
****************************************************************************/
void LCD_SetPoint(u16 x,u16 y,u16 point)
{
  if ( (x>320)||(y>240) ) return;
  LCD_SetCursor(x,y);

  Clr_Cs;
  LCD_WriteIndex(0x0022);
  Set_Rs;
  LCD_WriteData(point);
  Clr_nWr;Set_nWr;
  Set_Cs;
}

/****************************************************************************
* Name: void LCD_DrawPicture (u16 StartX, u16 StartY, u16 EndX, u16 EndY, u16 * pic)
* Function: the scope of the specified coordinates display a picture
* Entry parameters: StartX line start coordinates
* StartY out initial coordinates of
* EndX line end coordinates
* EndY column coordinates of the end
            pic picture head pointer
* Output Parameters: None
* Note: the level of picture taking mode format scanning, 16-bit color mode
* Call the method: LCD_DrawPicture (0,0,100,100, (u16 *) demo);
****************************************************************************/
void LCD_DrawPicture(u16 StartX,u16 StartY,u16 EndX,u16 EndY,u16 *pic)
{
  u16  i;
  LCD_SetWindows(StartX,StartY,EndX,EndY);
  LCD_SetCursor(StartX,StartY);
  
  Clr_Cs;

  LCD_WriteIndex(0x0022);
  Set_Rs;
  for (i=0;i<(EndX*EndY);i++)
  {
      LCD_WriteData(*pic++);
	  Clr_nWr;Set_nWr;
  }
      
  Set_Cs;
}

/*******************************************************************************
* Function Name  : LCD_DrawMonoPict
* Description    : Displays a monocolor picture.
* Input          : - Pict: pointer to the picture array.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawMonoPict(unsigned char  *Pict)
{
  u32 index = 0, i = 0;

  LCD_SetWindows(0, 0, 239, 319);
  LCD_SetCursor(0, 0);

	LCD_WriteRegister(0x03,0x1018);		// freeRTOS demo  ( LCD_Message.h format )

  Clr_Cs;
  LCD_WriteIndex(0x0022);  
  Set_Rs;

  for(index = 0; index < 9600; index++)
  {
    for(i = 0; i < 8; i++)
    {
		if((Pict[index] & (1 << i)) == 0x00){
			LCD_WriteData(LCD_Blue);	// Font Color
		}
		else{	LCD_WriteData(LCD_Yellow);	// Background Color
			
		}
		Clr_nWr;Set_nWr;
    }
  }
 //LCD_WriteRegister(0x03,0x1030);	// original scan direction
  Set_Cs;
}

/****************************************************************************
* Name: void LCD_PutChar (u16 x, u16 y, u8 c, u16 charColor, u16 bkColor)
* Function: specifies coordinates in a 8x16 matrix display of ascii characters
* Entry parameters: x-line coordinates
* Y column coordinate
* CharColor the color of characters
* BkColor character background color
* Output Parameters: None
* Note: Display range is limited to display the ascii code
* Call the method: LCD_PutChar (10,10, 'a', 0x0000, 0xffff);
****************************************************************************/
void LCD_PutChar(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor)
{
  u16 i=0;
  u16 j=0;
  
  u8 tmp_char=0;

  for (i=0;i<16;i++)
  {
    tmp_char=ascii_8x16[((c-0x20)*16)+i];
    for (j=0;j<8;j++)
    {
      if ( (tmp_char >> (7-j)) & (0x01 == 0x01))
        {
          LCD_SetPoint(x+j,y+i,charColor); // character color
        }
        else
        {
          LCD_SetPoint(x+j,y+i,bkColor); // background color
        }
    }
  }
}

/****************************************************************************
* Name: void LCD_Test ()
* Function: testing LCD screen
* Entry parameters: None
* Export Parameters: None
* Note: Display color bar test LCD screen is working
* Call the method: LCD_Test ();
****************************************************************************/
void LCD_Test()
{
  u8  R_data,G_data,B_data,i,j;

	LCD_SetCursor(0x00, 0x0000);
	LCD_WriteRegister(0x0050,0x00);//GRAM horizontal start position
	LCD_WriteRegister(0x0051,239);//GRAM horizontal end position
	LCD_WriteRegister(0x0052,0);//Vertical GRAM Start position
	LCD_WriteRegister(0x0053,319);//Vertical GRAM end position
	Clr_Cs; 
	LCD_WriteIndex(0x0022);
	Set_Rs;
    R_data=0;G_data=0;B_data=0;     
    for(j=0;j<50;j++)//red crescendo Article
    {
        for(i=0;i<240;i++)
            {R_data=i/8;LCD_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;Set_nWr;}
    }
    R_data=0x1f;G_data=0x3f;B_data=0x1f;
    for(j=0;j<50;j++)
    {
        for(i=0;i<240;i++)
            {
            G_data=0x3f-(i/4);
            B_data=0x1f-(i/8);
            LCD_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;
			Set_nWr;
			}
    }
//----------------------------------
    R_data=0;G_data=0;B_data=0;
    for(j=0;j<50;j++)//green crescendo Article
    {
        for(i=0;i<240;i++)
            {G_data=i/4;
			LCD_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;
			Set_nWr;}
    }

    R_data=0x1f;G_data=0x3f;B_data=0x1f;
    for(j=0;j<50;j++)
    {
        for(i=0;i<240;i++)
            {
            R_data=0x1f-(i/8);
            B_data=0x1f-(i/8);
            LCD_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;
			Set_nWr;
		}
    }
//----------------------------------
 
    R_data=0;G_data=0;B_data=0;
    for(j=0;j<60;j++)//blue crescendo Article
    {
        for(i=0;i<240;i++)
            {B_data=i/8;LCD_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;
			Set_nWr;}
    } 

    B_data=0; 
    R_data=0x1f;G_data=0x3f;B_data=0x1f;

    for(j=0;j<60;j++)
    {
        for(i=0;i<240;i++)
            {
            G_data=0x3f-(i/4);
            R_data=0x1f-(i/8);
            LCD_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;
			Set_nWr;
		}
    }	  
	Set_Cs;
}

/****************************************************************************
* Name: u16 LCD_BGR2RGB (u16 c)
* Function: RRRRRGGGGGGBBBBB to BBBBBGGGGGGRRRRR format
* Entry parameters: c BRG color value
* Output Parameters: RGB color value
* Note: Internal function call
****************************************************************************/
u16 LCD_BGR2RGB(u16 c)
{
  u16  r, g, b, rgb;

  b = (c>>0)  & 0x1f;
  g = (c>>5)  & 0x3f;
  r = (c>>11) & 0x1f;
  
  rgb =  (b<<11) + (g<<5) + (r<<0);

  return( rgb );
}

/****************************************************************************
* Name: void LCD_WriteIndex (u16 idx)
* Function: write LCD controller register address
* Import parameter: idx register address
* Output Parameters: None
* Note: you must first select the controller before calling the internal function
* Call the method: LCD_WriteIndex (0x0000);
****************************************************************************/
__inline void LCD_WriteIndex(u16 idx)
{
    Clr_Rs;
	Set_nRd;
	LCD_WriteData(idx);
	Clr_nWr;
	Set_nWr;
	Set_Rs;
}

/****************************************************************************
* Name: void LCD_WriteData (u16 dat)
* Function: write LCD register data
* Entry parameters: dat register data
* Output Parameters: None
* Note: specify the address to write data to the controller, called the former must first write register address, the internal function
* Call the method: LCD_WriteData (0x1030)
****************************************************************************/
void LCD_WriteData(u16 data)
{
	GPIOB->ODR=((GPIOB->ODR&0x00ff)|(data<<8));
	GPIOC->ODR=((GPIOC->ODR&0xff00)|(data>>8));
}

/****************************************************************************
* Name: u16 LCD_ReadData (void)
* Function: Read controller data
* Entry parameters: None
* Output Parameters: Return to read the data
* Note: Internal function
* Calling method: i = LCD_ReadData ();
****************************************************************************/
__inline u16 LCD_ReadData(void)
{
//========================================================================
// **                                                                    **
// ** nCS       ----\__________________________________________/-------  **
// ** RS        ------\____________/-----------------------------------  **
// ** nRD       -------------------------\_____/---------------------  **
// ** nWR       --------\_______/--------------------------------------  **
// ** DB[0:15]  ---------[index]----------[data]-----------------------  **
// **                                                                    **
//========================================================================
	u16 tmp;
	GPIOB->CRH = (GPIOB->CRH & 0x00000000) | 0x44444444;
	GPIOC->CRL = (GPIOC->CRL & 0x00000000) | 0x44444444;
	tmp = (((GPIOB->IDR)>>8)|((GPIOC->IDR)<<8));
	GPIOB->CRH = (GPIOB->CRH & 0x00000000) | 0x33333333;
	GPIOC->CRL = (GPIOC->CRL & 0x00000000) | 0x33333333;
	return tmp;
}

/****************************************************************************
* Name: u16 LCD_ReadRegister (u16 index)
* Function: Read the value of the specified address register
* Entry parameters: index register address
* Output Parameters: Register values
* Note: Internal function
* Call the method: i = LCD_ReadRegister (0x0022);
****************************************************************************/
__inline u16 LCD_ReadRegister(u16 index)
{ 
  	Clr_Cs;
	LCD_WriteIndex(index);
	Clr_nRd;     
	index = LCD_ReadData(); 
	Set_nRd;      	
	Set_Cs;
	return index;
}

/****************************************************************************
* Name: void LCD_WriteRegister (u16 index, u16 dat)
* Function: write the value of the specified address register
* Entry parameters: index register address
*: Dat register value
* Output Parameters: None
* Note: Internal function
* Call the method: LCD_WriteRegister (0x0000, 0x0001);
****************************************************************************/
__inline void LCD_WriteRegister(u16 index,u16 dat)
{
 /************************************************************************
  **                                                                    **
  ** nCS       ----\__________________________________________/-------  **
  ** RS        ------\____________/-----------------------------------  **
  ** nRD       -------------------------------------------------------  **
  ** nWR       --------\_______/--------\_____/-----------------------  **
  ** DB[0:15]  ---------[index]----------[data]-----------------------  **
  **                                                                    **
  ************************************************************************/
//   u16 temp;
    Clr_Cs;
	Clr_Rs;
	Set_nRd;
	LCD_WriteData(index);
//	temp=(*((volatile unsigned long *) 0x40010C0C));
//	(*((volatile unsigned long *) 0x40010C0C))=(index<<8)|(temp&0x00ff);
//	temp=(*((volatile unsigned long *) 0x4001100C));
//	(*((volatile unsigned long *) 0x4001100C))=(index>>8)|(temp&0xff00);
	Clr_nWr;
	Set_nWr;
	Set_Rs;       
	LCD_WriteData(dat);  
//	temp=(*((volatile unsigned long *) 0x40010C0C));
//	(*((volatile unsigned long *) 0x40010C0C))=(dat<<8)|(temp&0x00ff);
//	temp=(*((volatile unsigned long *) 0x4001100C));
//	(*((volatile unsigned long *) 0x4001100C))=(dat>>8)|(temp&0xff00);     
	Clr_nWr;
	Set_nWr;
	Set_Cs;
}

/****************************************************************************
* Name: void LCD_Reset ()
* Function: reset ili9320 controller
* Entry parameters: None
* Output Parameters: None
* Description: reset the controller, the internal function
* Call the method: LCD_Reset ()
****************************************************************************/
void LCD_Reset()
{
  /***************************************
   **                                   **
   **  -------\______________/-------   **
   **         | <---Tres---> |          **
   **                                   **
   **  Tres: Min.1ms                    **
   ***************************************/
    
  	Set_Rst;;
    LCD_Delay(50000);
  	Clr_Rst;
    LCD_Delay(50000);
  	Set_Rst;
    LCD_Delay(50000);
}

/****************************************************************************
* Name: void LCD_BackLight (u8 status)
* Function: open, off LCD backlight
* Entry parameters: status 1: backlit 0: backlight off
* Output Parameters: None
* Note:
* Call Method: LCD_BackLight (1);
****************************************************************************/
void LCD_BackLight(u8 status)
{
  if ( status >= 1 )
  {
    Lcd_Light_ON;
  }
  else
  {
    Lcd_Light_OFF;
  }
}

/****************************************************************************
* Name: void LCD_Delay (vu32 nCount)
* Function: Delay
* Entry parameters: nCount delay value
* Output Parameters: None
* Note:
* Call the method: LCD_Delay (10000);
****************************************************************************/
void LCD_Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}
