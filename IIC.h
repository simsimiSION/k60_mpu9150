#ifndef _IIC_H_
#define _IIC_H_

#include "common.h"

#define I2C_HW  HW_GPIOC
#define I2C_SCL 7
#define I2C_SDA 5

void I2C_init();
//uint8_t SDA_IN(void);
bool I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NAck(void);
bool I2C_Wait_Ack(void);
void I2C_Send_Byte(uint8_t txd);
uint8_t I2C_Read_Byte(unsigned char ack);
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr,uint8_t *data);
uint8_t I2CreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t I2CwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
uint8_t I2CreadByte(uint8_t dev, uint8_t reg, uint8_t *data);
unsigned char I2CwriteByte(unsigned char dev, unsigned char reg, unsigned char data);

#endif //_IIC_H_