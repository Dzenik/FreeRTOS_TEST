#ifndef __MINISTM32_LCD_H__
#define __MINISTM32_LCD_H__

#define Set_Cs  GPIOC->BSRR = GPIO_Pin_8;
#define Clr_Cs  GPIOC->BRR = GPIO_Pin_8;

#define Set_Rs  GPIOC->BSRR = GPIO_Pin_9;
#define Clr_Rs  GPIOC->BRR = GPIO_Pin_9;

#define Set_nWr GPIOC->BSRR = GPIO_Pin_10;
#define Clr_nWr GPIOC->BRR = GPIO_Pin_10;

#define Set_nRd GPIOC->BSRR = GPIO_Pin_11;
#define Clr_nRd GPIOC->BRR = GPIO_Pin_11;

#define Set_Rst GPIOC->BSRR = GPIO_Pin_12;
#define Clr_Rst GPIOC->BRR = GPIO_Pin_12;

#define Lcd_Light_ON   GPIOC->BSRR = GPIO_Pin_13;
#define Lcd_Light_OFF  GPIOC->BRR = GPIO_Pin_13;


/* LCD color */
#define LCD_White          0xFFFF
#define LCD_Black          0x0000
#define LCD_Blue           0x001F
#define LCD_Orange         0x051F
#define LCD_Red            0xF800
#define LCD_Magenta        0xF81F
#define LCD_Green          0x07E0
#define LCD_Cyan           0x7FFF
#define LCD_Yellow         0xFFE0

void LCD_Configuration(void);
void LCD_Initializtion(void);
void LCD_Reset(void);
void LCD_WriteRegister(u16 index,u16 dat);
void LCD_SetCursor(u16 x,u16 y);
void LCD_SetWindows(u16 StartX,u16 StartY,u16 EndX,u16 EndY);
void LCD_DrawPicture(u16 StartX,u16 StartY,u16 EndX,u16 EndY,u16 *pic);
void LCD_SetPoint(u16 x,u16 y,u16 point);
void LCD_PutChar(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor);
void LCD_Clear(u16 bkColor);
void LCD_Delay(u32 nCount);
void LCD_Test(void);
void LCD_WriteData(u16 dat);
void LCD_WriteIndex(u16 idx);

void LCD_DrawMonoPict(unsigned char *Pict);

void LCD_BackLight(u8 status);

u16 LCD_BGR2RGB(u16 c);

u16 LCD_GetPoint(u16 x,u16 y);
u16 LCD_ReadData(void);
u16 LCD_ReadRegister(u16 index);


#endif	// __MINISTM32_LCD_H__

