
#include "hal_mcu.h"
#include "hal_SHT30.h"


/* SDA is at P0.4 */
#define HAL_SDA_PORT   P0
#define HAL_SDA_BIT    BV(4)
#define HAL_SDA_SEL    P0SEL
#define HAL_SDA_DIR    P0DIR

/* SCL is at P2.0 */
#define HAL_SCL_PORT   P2
#define HAL_SCL_BIT    BV(0)
#define HAL_SCL_SEL    P2SEL
#define HAL_SCL_DIR    P2DIR

#define HAL_SHT30_I2C_ADDR  0x88
#define HAL_SHT30_I2C_ACK 0
#define HAL_SHT30_I2C_NACK 1
#define HAL_SHT30_PERIODIC_CMD_MSB   0x21//   1 second
#define HAL_SHT30_PERIODIC_CMD_LSB   0x2D//   Low Repeatability, 2.5~3ms


int16 Temperature = 0;//   *0.1c
uint8 Humidity = 0;//   0~100%

uint8 CollectCount;
void HalI2CDelay(void);
void HalSHT30I2cStart(void);
void HalSHT30I2cStop(void);
uint8 HalSHT30I2cReadOneByte(uint8 ACK);
uint8 HalSHT30I2cWriteOneByte(uint8 Dat);
uint8 HalSHT30WriteCmd(uint8 TMP_MSB, uint8 TMP_LSB);

void HalI2CDelay(void)
{
  uint8 x;
  
  for(x = 0; x < 5; x++)
  {
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
  }
}
void HalSHT30I2cStart(void)
{
  HAL_SCL_SEL &= ~(HAL_SCL_BIT);    /* Set SCL pin function to GPIO */
  HAL_SCL_DIR &= ~(HAL_SCL_BIT);    /* Set SCL pin direction to Input */
  HAL_SDA_SEL &= ~(HAL_SDA_BIT);    /* Set SDA pin function to GPIO */
  HAL_SDA_DIR &= ~(HAL_SDA_BIT);    /* Set SDA pin direction to Input */
  
  HalI2CDelay();
  
  HAL_SDA_DIR |= HAL_SDA_BIT;    /* Set SDA pin direction to Output */
  HAL_SDA_PORT &= ~(HAL_SDA_BIT);
      
  HalI2CDelay();
  
  HAL_SCL_DIR |= HAL_SCL_BIT;    /* Set SCL pin direction to Output */
  HAL_SCL_PORT &= ~(HAL_SCL_BIT);
}

void HalSHT30I2cStop(void)
{
  HAL_SDA_DIR |= HAL_SDA_BIT;    /* Set SDA pin direction to Output */
  HAL_SDA_PORT &= ~(HAL_SDA_BIT);
      
  HalI2CDelay();
  
  HAL_SCL_DIR &= ~(HAL_SCL_BIT);    /* Set SCL pin direction to Input */
  
  HalI2CDelay();
  
  HAL_SDA_DIR &= ~(HAL_SDA_BIT);    /* Set SDA pin direction to Input */
  
}
uint8 HalSHT30I2cReadOneByte(uint8 ACK)
{
  uint8 i;
  uint8 ucRet = 0;
  
  HAL_SDA_DIR &= ~(HAL_SDA_BIT);    /* Set SDA pin direction to Input */
  for ( i = 0 ; i < 8 ;  i ++)
  {
    HAL_SCL_DIR &= ~(HAL_SCL_BIT);    /* Set SCL pin direction to Input */
    ucRet <<= 1;
    HalI2CDelay();
    if(HAL_SDA_PORT & HAL_SDA_BIT)
    {
	ucRet ++;
    }
    HAL_SCL_DIR |= HAL_SCL_BIT;    /* Set SCL pin direction to Output */
    HAL_SCL_PORT &= ~(HAL_SCL_BIT);
    HalI2CDelay();
  }
  if(ACK == HAL_SHT30_I2C_ACK)
  {
    HAL_SDA_DIR |= HAL_SDA_BIT;    /* Set SDA pin direction to Output */
    HAL_SDA_PORT &= ~(HAL_SDA_BIT);
  }
  HalI2CDelay();
  HAL_SCL_DIR &= ~(HAL_SCL_BIT);    /* Set SCL pin direction to Input */
  HalI2CDelay();
  HAL_SCL_DIR |= HAL_SCL_BIT;    /* Set SCL pin direction to Output */
  HAL_SCL_PORT &= ~(HAL_SCL_BIT);
	
  return ucRet;
}

uint8 HalSHT30I2cWriteOneByte(uint8 Dat)
{
  uint8 i;

  for(i = 0; i < 8; i++)
  {
    HAL_SCL_DIR |= HAL_SCL_BIT;    /* Set SCL pin direction to Output */
    HAL_SCL_PORT &= ~(HAL_SCL_BIT);
    HalI2CDelay();
    if(Dat & 0x80)
    {
      HAL_SDA_DIR &= ~(HAL_SDA_BIT);    /* Set SDA pin direction to Input */
    }
    else
    {
      HAL_SDA_DIR |= HAL_SDA_BIT;    /* Set SDA pin direction to Output */
      HAL_SDA_PORT &= ~(HAL_SDA_BIT);
    }
    Dat<<=1;
    HalI2CDelay();
    HAL_SCL_DIR &= ~(HAL_SCL_BIT);    /* Set SCL pin direction to Input */
    HalI2CDelay();
  }
  HAL_SCL_DIR |= HAL_SCL_BIT;    /* Set SCL pin direction to Output */
  HAL_SCL_PORT &= ~(HAL_SCL_BIT);
  HalI2CDelay();
  HAL_SDA_DIR &= ~(HAL_SDA_BIT);    /* Set SDA pin direction to Input */
  HalI2CDelay();
  HAL_SCL_DIR &= ~(HAL_SCL_BIT);    /* Set SCL pin direction to Input */
  HalI2CDelay();
  if(HAL_SDA_PORT & HAL_SDA_BIT)// err
  {
    HAL_SCL_DIR |= HAL_SCL_BIT;    /* Set SCL pin direction to Output */
    HAL_SCL_PORT &= ~(HAL_SCL_BIT);
    return(1);
  }
  HAL_SCL_DIR |= HAL_SCL_BIT;    /* Set SCL pin direction to Output */
  HAL_SCL_PORT &= ~(HAL_SCL_BIT);
  HalI2CDelay();
  return(0);// success
}

uint8 HalSHT30WriteCmd(uint8 TMP_MSB, uint8 TMP_LSB)
{
  uint8 Ret;
  
  HalSHT30I2cStart();
  
  Ret = HalSHT30I2cWriteOneByte(HAL_SHT30_I2C_ADDR);
  
  if(Ret != 0)
  {
    HalSHT30I2cStop();
    return(Ret);
  }
  
  Ret = HalSHT30I2cWriteOneByte(TMP_MSB);
  
  if(Ret != 0)
  {
    HalSHT30I2cStop();
    return(Ret);
  }
  
  Ret = HalSHT30I2cWriteOneByte(TMP_LSB);
  
  if(Ret != 0)
  {
    HalSHT30I2cStop();
    return(Ret);
  }
  
  return(0);// success
}

uint8 HalSHT30Init(void)
{
  uint8 Ret;
  
  Ret = HalSHT30WriteCmd(HAL_SHT30_PERIODIC_CMD_MSB, HAL_SHT30_PERIODIC_CMD_LSB);
  if(Ret != 0)
  {
    Ret = HalSHT30WriteCmd(HAL_SHT30_PERIODIC_CMD_MSB, HAL_SHT30_PERIODIC_CMD_LSB);
  }
  HalSHT30I2cStop();
  return(Ret);
}


uint8 HalSHT30ReadData(void)
{
    //uint8 Ret, m;
    uint8 Ret;
    uint16 T, H;
    //int16 x;

  
    Ret = HalSHT30WriteCmd(0xE0, 0x00);
    if(Ret != 0)
    {
      return(Ret);
    }
  
    HalSHT30I2cStart();
    Ret = HalSHT30I2cWriteOneByte(HAL_SHT30_I2C_ADDR | 0x01);
    if(Ret != 0)
    {
      HalSHT30I2cStop();
      return(Ret);
    }
  
    T = (uint16)HalSHT30I2cReadOneByte(HAL_SHT30_I2C_ACK);
    T <<= 8;
    T |= (uint16)HalSHT30I2cReadOneByte(HAL_SHT30_I2C_ACK);
    HalSHT30I2cReadOneByte(HAL_SHT30_I2C_ACK);//  CRC
  
    H = (uint16)HalSHT30I2cReadOneByte(HAL_SHT30_I2C_ACK);
    H <<= 8;
    H |= (uint16)HalSHT30I2cReadOneByte(HAL_SHT30_I2C_ACK);
    HalSHT30I2cReadOneByte(HAL_SHT30_I2C_NACK);//  CRC
  
    HalSHT30I2cStop();
    
    Temperature = (int16)(((uint32)T * 1750)/65535) - 450;//   *0.1c
    Humidity = (uint8)( (100 * (uint32)H) / 65535 );//  0~100%
	return 0;
    /*
    x = (int16)(((uint32)T * 175)/65535) - 45;//   *1c
    if(x<0)
    {
      Temperature[0] = '-';
      m = (uint8)(x*(-1));
    }
    else
    {
      Temperature[0] = '+';
      m = (uint8)x;
    }
    Temperature[1] = (m / 100)+'0';
    m %= 100;
    Temperature[2] = (m / 10)+'0';
    Temperature[3] = (m % 10)+'0';
    Temperature[4] = 'C';
    Temperature[5] = '\0';
    
    
    m = (uint8)( (100 * (uint32)H) / 65535 );//  0~100%
    Humidity[0] = (m / 100)+'0';
    m %= 100;
    Humidity[1] = (m / 10)+'0';
    Humidity[2] = (m % 10)+'0';
    Humidity[3] = '%';
    Humidity[4] = '\0';
    */
    
}






