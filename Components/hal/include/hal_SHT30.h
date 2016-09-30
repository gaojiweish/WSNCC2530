

#ifndef HAL_SHT30_H
#define HAL_SHT30_H


extern uint8 HalSHT30Init(void);
extern uint8 HalSHT30ReadData(void);

extern int16 Temperature;//   *1c
extern uint8 Humidity;//   0~100%

#endif