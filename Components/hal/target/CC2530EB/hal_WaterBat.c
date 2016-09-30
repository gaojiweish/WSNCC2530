
#include "hal_mcu.h"
#include "hal_adc.h"
#include "hal_WaterBat.h"

#define HAL_ADC_CHN_WATER    1  //     P0_1
#define HAL_ADC_CHN_BAT      6  //     P0_6

uint16 Water_V;    //  0.1v
uint16 Battery_V;  //  0.1v


void Hal_Read_Water_And_Battery(void)
{
  Water_V = HalAdcRead(HAL_ADC_CHN_WATER, HAL_ADC_RESOLUTION_8);
  Battery_V = HalAdcRead(HAL_ADC_CHN_BAT, HAL_ADC_RESOLUTION_8);
  
  Water_V = (Battery_V * 33) / 128;
  Battery_V = (uint16)( ((uint32)Battery_V*43*33)/1280 );

}

uint16 Hal_Read_Water(void)
{
  Water_V = HalAdcRead(HAL_ADC_CHN_WATER, HAL_ADC_RESOLUTION_8);
  Water_V = (Water_V * 33) / 128;
  return Water_V;
}

uint16 Hal_Read_Battery(void)
{
  Battery_V = HalAdcRead(HAL_ADC_CHN_BAT, HAL_ADC_RESOLUTION_8);
  Battery_V = (uint16)( ((uint32)Battery_V*43*33)/1280 );
  return Battery_V;
}



