

#ifndef HAL_WATERBAT_H
#define HAL_WATERBAT_H


extern uint16 Water_V;    //  0.1v
extern uint16 Battery_V;  //  0.1v
extern void Hal_Read_Water_And_Battery(void);
extern uint16 Hal_Read_Water(void);
extern uint16 Hal_Read_Battery(void);
#endif