/**
******************************************************************************
* @file    i2c.c
* @author  YANDLD
* @version V2.5
* @date    2014.3.26
* \date    2015.10.06 FreeXc 完善了对 i2c 模块的相关注释
* @brief   www.beyondcore.net   http://upcmcu.taobao.com 
* @note    此文件为芯片I2C模块的底层功能函数
******************************************************************************
*/

#include "common.h"
#include "IIC.h"
#include "gpio.h"


#define SDA_DDR_OUT()       do {GPIO_PinConfig(I2C_HW, I2C_SDA, kOutput);}while(0)
#define SDA_DDR_IN()        do {GPIO_PinConfig(I2C_HW, I2C_SDA, kInput);}while(0)
#define SDA_H()             do {GPIO_WriteBit(I2C_HW, I2C_SDA, 1);}while(0)
#define SDA_L()             do {GPIO_WriteBit(I2C_HW, I2C_SDA, 0);}while(0)
#define SCL_H()             do {GPIO_WriteBit(I2C_HW, I2C_SCL, 1);}while(0)
#define SCL_L()             do {GPIO_WriteBit(I2C_HW, I2C_SCL, 0);}while(0)
#define I2C_DELAY()         DelayUs(4)
/**
* 
初始化I2C（自定义）
*/
void I2C_init()
{
  GPIO_QuickInit(I2C_HW, I2C_SCL, kGPIO_Mode_OOD);
  GPIO_QuickInit(I2C_HW, I2C_SDA, kGPIO_Mode_OOD);
}
/**
* \brief 读取I2C上SDA数据，Internal function
* \return SDA上的数据(1 bit)
*/
static inline uint8_t SDA_IN(void)
{
  return GPIO_ReadBit(I2C_HW, I2C_SDA);
}

/**
* \brief I2C Start，Internal function
* \retval true
*/
static bool I2C_Start(void)
{
    SDA_H();
    SCL_H();
    I2C_DELAY();

    SDA_DDR_IN();
    if(!SDA_IN())
    {
        SDA_DDR_OUT();
        return 0;   /* SDA线为低电平则总线忙,退出 */
    }
    SDA_DDR_OUT();
    SDA_L();

    I2C_DELAY();
    SCL_L();

    if(SDA_IN())
    {
        SDA_DDR_OUT();
        return 0;   /* SDA线为高电平则总线出错,退出 */
    }
    //SDA_DDR_OUT();
    //SDA_L();
    //SCCB_delay();
    return 1;
}

/**
* \brief I2C Stop，Internal function
* \retval None
*/
static void I2C_Stop(void)
{
  SCL_L();
  SDA_L();
  I2C_DELAY();
  SCL_H();
  SDA_H();
  //I2C_DELAY();
  DelayUs(8);
}

/**
* \brief I2C Ack，Internal function
* \retval None
*/
static void I2C_Ack(void)
{
  SCL_L();
  SDA_L();
  I2C_DELAY();
  SCL_H();
  I2C_DELAY();
  SCL_L();
  I2C_DELAY();
}

/**
* \brief I2C Not Ack，Internal function
* \retval None
*/
static void I2C_NAck(void)
{
  SCL_L();
  I2C_DELAY();
  SDA_H();
  I2C_DELAY();
  SCL_H();
  I2C_DELAY();
  SCL_L();
  I2C_DELAY();
}

/**
* \brief I2C Wait Ack，Internal function
* \return 应答信号
*/
static bool I2C_Wait_Ack(void)
{
  uint8_t ack;
  SDA_DDR_IN();
  SCL_L();
  
  I2C_DELAY();
  SCL_H();
  I2C_DELAY();
  ack = SDA_IN();
  if(SDA_IN())           //应答为高电平，异常，通信失败
    {
        SDA_DDR_OUT();
        SCL_L();
        return 0;
    }
  SCL_L();
  SDA_DDR_OUT();
  
  return 1;
}
/**************************实现函数********************************************
*函数原型:		void I2C_Send_Byte(uint8_t txd)
*功　　能:	    I2C发送一个字节
*******************************************************************************/		  
void I2C_Send_Byte(uint8_t txd)
{                        
  volatile uint8_t t; 
  t = 8;  
  SDA_DDR_OUT();	    
  SCL_L();//拉低时钟开始数据传输
  while(t--)
  {    
    if(txd&0x80)  SDA_H();
    else SDA_L();
    //I2C_SDA=(txd&0x80)>>7;
    txd<<=1; 	  
   DelayUs(1);
    SCL_H();
    DelayUs(1);
    SCL_L();	
   DelayUs(1);
  }	 
} 
/**************************实现函数********************************************
*函数原型:		uint8_t I2C_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
uint8_t I2C_Read_Byte(unsigned char ack)
{
  unsigned char i,receive=0;
  SDA_DDR_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
  {
    SCL_L();
    DelayUs(4);
    SCL_H();
    receive<<=1;
    //if(SDA_IN)receive++;  
    if(SDA_IN())
     {
        receive |= 0x01;
     }

    DelayUs(4);
  }					 
  if (ack)
    I2C_Ack(); //发送ACK 
  else
    I2C_NAck();//发送nACK  
  return receive;
}
/**************************实现函数********************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	I2C_Addr  目标设备地址
addr	   寄存器地址
返回   读出来的值
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr,uint8_t *data)
{
  unsigned char res=0;
  
  if(!I2C_Start()) return 0;
  I2C_Send_Byte(I2C_Addr);	   //发送写命令
  res++;
  if(!I2C_Wait_Ack()) 
  {
    I2C_Stop();
    return 0;
  }
  
  I2C_Send_Byte(addr); //res++;  //发送地址 
  I2C_Wait_Ack();	  
  I2C_Stop();//产生一个停止条件	
  
  if(!I2C_Start()) return 0;
  
  I2C_Send_Byte(I2C_Addr+1); res++;          //I2C_Addr加1  R/W = 1 读操作			   
   if(!I2C_Wait_Ack()) 
  {
    I2C_Stop();
    return 0;
  }
  *data=I2C_Read_Byte(0);	   
  I2C_Stop();//产生一个停止条件
  
  return 1;
}


/**************************实现函数********************************************
*函数原型:		uint8_t I2CreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	dev  目标设备地址
reg	  寄存器地址
length 要读的字节数
*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/ 
uint8_t I2CreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data){
  uint8_t count = 0;
  uint8_t temp;
  I2C_Start();
  I2C_Send_Byte(dev);	   //发送写命令
  I2C_Wait_Ack();
  I2C_Send_Byte(reg);   //发送地址
  I2C_Wait_Ack();	  
  I2C_Start();
  I2C_Send_Byte(dev+1);  //进入接收模式	
  I2C_Wait_Ack();
  
  for(count=0;count<length;count++){
    
    if(count!=(length-1))
      temp = I2C_Read_Byte(1);  //带ACK的读数据
    else  
      temp = I2C_Read_Byte(0);	 //最后一个字节NACK
    
    data[count] = temp;
  }
  I2C_Stop();//产生一个停止条件
  return count;
}

/**************************实现函数********************************************
*函数原型:		uint8_t I2CwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
输入	dev  目标设备地址
reg	  寄存器地址
length 要写的字节数
*data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/ 
uint8_t I2CwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data){
  
  uint8_t count = 0;
  if(!I2C_Start()) return 0;
  I2C_Send_Byte(dev);	   //发送写命令
  if(!I2C_Wait_Ack()) 
  {
    I2C_Stop();
    return 0;
  }
  I2C_Send_Byte(reg);   //发送地址
  I2C_Wait_Ack();	  
  for(count=0;count<length;count++){
    I2C_Send_Byte(data[count]); 
    I2C_Wait_Ack(); 
  }
  I2C_Stop();//产生一个停止条件
  
  return 1; //status == 0;
}

/**************************实现函数********************************************
*函数原型:		uint8_t I2CreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	dev  目标设备地址
reg	   寄存器地址
*data  读出的数据将要存放的地址
返回   1
*******************************************************************************/ 
uint8_t I2CreadByte(uint8_t dev, uint8_t reg, uint8_t *data){
   uint8_t i = 0;
    while( 0 == I2C_ReadOneByte(dev, reg, data))
    {
        i++;
        if(i == 30)
        {
            return 0 ;
        }
    }
    return 1;
}

/**************************实现函数********************************************
*函数原型:		unsigned char I2CwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:	    写入指定设备 指定寄存器一个字节
输入	dev  目标设备地址
reg	   寄存器地址
data  将要写入的字节
返回   1
*******************************************************************************/ 
unsigned char I2CwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
  
   uint8_t i = 0;
    while( 0 == I2CwriteBytes(dev, reg, 1, &data) )
    {
        i++;
        if(i == 20)
        {
            return 0 ;
        }
    }
    return 1;
}

/**************************实现函数********************************************
*函数原型:		uint8_t I2CwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
reg	   寄存器地址
bitStart  目标字节的起始位
length   位长度
data    存放改变目标字节位的值
返回   成功 为1 
失败为0
*******************************************************************************/ 
uint8_t I2CwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
{
  
  uint8_t b;
  if (I2CreadByte(dev, reg, &b) != 0) {
    uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
    data <<= (8 - length);
    data >>= (7 - bitStart);
    b &= mask;
    b |= data;
    return I2CwriteByte(dev, reg, b);
  } else {
    return 0;
  }
}

/**************************实现函数********************************************
*函数原型:		uint8_t I2CwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
失败为0
*******************************************************************************/ 
uint8_t I2CwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data){
  uint8_t b;
  
  I2CreadByte(dev, reg, &b);
  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  
  return I2CwriteByte(dev, reg, b);
}

//------------------End of File----------------------------