
#include "hal_mcu.h"
#include "hal_GPIO.h"


/* GPIOA is at P1.5 */
#define HAL_GPIOA_PORT   P1
#define HAL_GPIOA_BIT    BV(5)
#define HAL_GPIOA_SEL    P1SEL
#define HAL_GPIOA_DIR    P1DIR

/* GPIOB is at P1.6 */
#define HAL_GPIOB_PORT   P1
#define HAL_GPIOB_BIT    BV(6)
#define HAL_GPIOB_SEL    P1SEL
#define HAL_GPIOB_DIR    P1DIR

/* GPIOC is at P1.7 */
#define HAL_GPIOC_PORT   P1
#define HAL_GPIOC_BIT    BV(7)
#define HAL_GPIOC_SEL    P1SEL
#define HAL_GPIOC_DIR    P1DIR

uint8 GPIOStatic;

void HalGPIOStaDelay(void);

void HalGPIOStaDelay(void)
{
  uint8 x;
  
  for(x = 0; x < 100; x++)
  {
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
  }
}

void HalReadGPIOSta(void)
{
  HAL_GPIOA_SEL &= ~(HAL_GPIOA_BIT);    /* Set pin function to GPIO */
  HAL_GPIOA_DIR &= ~(HAL_GPIOA_BIT);    /* Set pin direction to Input */
  
  HAL_GPIOB_SEL &= ~(HAL_GPIOB_BIT);    /* Set pin function to GPIO */
  HAL_GPIOB_DIR &= ~(HAL_GPIOB_BIT);    /* Set pin direction to Input */
  
  //HAL_GPIOC_SEL &= ~(HAL_GPIOC_BIT);    /* Set pin function to GPIO */
  //HAL_GPIOC_DIR &= ~(HAL_GPIOC_BIT);    /* Set pin direction to Input */
  
  HalGPIOStaDelay();

  if(HAL_GPIOA_PORT & HAL_GPIOA_BIT)//  High
  {
  	if(HAL_GPIOB_PORT & HAL_GPIOB_BIT)//  High
  	{
  		GPIOStatic=0x03;
  	}
	else
		GPIOStatic=0x01;
  }
  else
  {
  	if(HAL_GPIOB_PORT & HAL_GPIOB_BIT)//  High
  	{
  		GPIOStatic=0x02;
  	}
	else
		GPIOStatic=0x00;
  }
}








