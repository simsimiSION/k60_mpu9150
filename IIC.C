/**
******************************************************************************
* @file    i2c.c
* @author  YANDLD
* @version V2.5
* @date    2014.3.26
* \date    2015.10.06 FreeXc �����˶� i2c ģ������ע��
* @brief   www.beyondcore.net   http://upcmcu.taobao.com 
* @note    ���ļ�ΪоƬI2Cģ��ĵײ㹦�ܺ���
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
��ʼ��I2C���Զ��壩
*/
void I2C_init()
{
  GPIO_QuickInit(I2C_HW, I2C_SCL, kGPIO_Mode_OOD);
  GPIO_QuickInit(I2C_HW, I2C_SDA, kGPIO_Mode_OOD);
}
/**
* \brief ��ȡI2C��SDA���ݣ�Internal function
* \return SDA�ϵ�����(1 bit)
*/
static inline uint8_t SDA_IN(void)
{
  return GPIO_ReadBit(I2C_HW, I2C_SDA);
}

/**
* \brief I2C Start��Internal function
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
        return 0;   /* SDA��Ϊ�͵�ƽ������æ,�˳� */
    }
    SDA_DDR_OUT();
    SDA_L();

    I2C_DELAY();
    SCL_L();

    if(SDA_IN())
    {
        SDA_DDR_OUT();
        return 0;   /* SDA��Ϊ�ߵ�ƽ�����߳���,�˳� */
    }
    //SDA_DDR_OUT();
    //SDA_L();
    //SCCB_delay();
    return 1;
}

/**
* \brief I2C Stop��Internal function
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
* \brief I2C Ack��Internal function
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
* \brief I2C Not Ack��Internal function
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
* \brief I2C Wait Ack��Internal function
* \return Ӧ���ź�
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
  if(SDA_IN())           //Ӧ��Ϊ�ߵ�ƽ���쳣��ͨ��ʧ��
    {
        SDA_DDR_OUT();
        SCL_L();
        return 0;
    }
  SCL_L();
  SDA_DDR_OUT();
  
  return 1;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void I2C_Send_Byte(uint8_t txd)
*��������:	    I2C����һ���ֽ�
*******************************************************************************/		  
void I2C_Send_Byte(uint8_t txd)
{                        
  volatile uint8_t t; 
  t = 8;  
  SDA_DDR_OUT();	    
  SCL_L();//����ʱ�ӿ�ʼ���ݴ���
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t I2C_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
uint8_t I2C_Read_Byte(unsigned char ack)
{
  unsigned char i,receive=0;
  SDA_DDR_IN();//SDA����Ϊ����
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
    I2C_Ack(); //����ACK 
  else
    I2C_NAck();//����nACK  
  return receive;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	I2C_Addr  Ŀ���豸��ַ
addr	   �Ĵ�����ַ
����   ��������ֵ
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr,uint8_t *data)
{
  unsigned char res=0;
  
  if(!I2C_Start()) return 0;
  I2C_Send_Byte(I2C_Addr);	   //����д����
  res++;
  if(!I2C_Wait_Ack()) 
  {
    I2C_Stop();
    return 0;
  }
  
  I2C_Send_Byte(addr); //res++;  //���͵�ַ 
  I2C_Wait_Ack();	  
  I2C_Stop();//����һ��ֹͣ����	
  
  if(!I2C_Start()) return 0;
  
  I2C_Send_Byte(I2C_Addr+1); res++;          //I2C_Addr��1  R/W = 1 ������			   
   if(!I2C_Wait_Ack()) 
  {
    I2C_Stop();
    return 0;
  }
  *data=I2C_Read_Byte(0);	   
  I2C_Stop();//����һ��ֹͣ����
  
  return 1;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t I2CreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
reg	  �Ĵ�����ַ
length Ҫ�����ֽ���
*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
uint8_t I2CreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data){
  uint8_t count = 0;
  uint8_t temp;
  I2C_Start();
  I2C_Send_Byte(dev);	   //����д����
  I2C_Wait_Ack();
  I2C_Send_Byte(reg);   //���͵�ַ
  I2C_Wait_Ack();	  
  I2C_Start();
  I2C_Send_Byte(dev+1);  //�������ģʽ	
  I2C_Wait_Ack();
  
  for(count=0;count<length;count++){
    
    if(count!=(length-1))
      temp = I2C_Read_Byte(1);  //��ACK�Ķ�����
    else  
      temp = I2C_Read_Byte(0);	 //���һ���ֽ�NACK
    
    data[count] = temp;
  }
  I2C_Stop();//����һ��ֹͣ����
  return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t I2CwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
reg	  �Ĵ�����ַ
length Ҫд���ֽ���
*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
uint8_t I2CwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data){
  
  uint8_t count = 0;
  if(!I2C_Start()) return 0;
  I2C_Send_Byte(dev);	   //����д����
  if(!I2C_Wait_Ack()) 
  {
    I2C_Stop();
    return 0;
  }
  I2C_Send_Byte(reg);   //���͵�ַ
  I2C_Wait_Ack();	  
  for(count=0;count<length;count++){
    I2C_Send_Byte(data[count]); 
    I2C_Wait_Ack(); 
  }
  I2C_Stop();//����һ��ֹͣ����
  
  return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t I2CreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
*data  ���������ݽ�Ҫ��ŵĵ�ַ
����   1
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char I2CwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*��������:	    д��ָ���豸 ָ���Ĵ���һ���ֽ�
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
data  ��Ҫд����ֽ�
����   1
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t I2CwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitStart  Ŀ���ֽڵ���ʼλ
length   λ����
data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
ʧ��Ϊ0
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t I2CwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
ʧ��Ϊ0
*******************************************************************************/ 
uint8_t I2CwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data){
  uint8_t b;
  
  I2CreadByte(dev, reg, &b);
  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  
  return I2CwriteByte(dev, reg, b);
}

//------------------End of File----------------------------