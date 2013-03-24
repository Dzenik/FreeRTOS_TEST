#ifndef	__MINISTM32_H__
#define __MINISTM32_H__

#define LED2_On()	GPIO_SetBits(GPIOA,GPIO_Pin_0)
#define LED2_Off()	GPIO_ResetBits(GPIOA,GPIO_Pin_0)
#define LED3_On()	GPIO_SetBits(GPIOA,GPIO_Pin_1)
#define LED3_Off()	GPIO_ResetBits(GPIOA,GPIO_Pin_1)

void Board_Init(void);
void LED_Config(void);


#endif	// __MINISTM32_H__
