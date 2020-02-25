#ifndef _MPU9150_H_
#define _MPU9150_H_
#include "common.h"

#define	SMPLRT_DIV		0x19	
#define	CONFIG			0x1A	
#define	GYRO_CONFIG		0x1B	
#define	ACCEL_CONFIG	0x1C	
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	
#define	WHO_AM_I		0x75	

#define	MPU9150_Addr    0xD0
#define AK8975_Addr     0x18
#define CNTL            0X0A
#define HXL             0X03
#define HXH             0X04
#define HYL             0X05
#define HYH             0X06
#define HZL             0X07
#define HZH             0X08

typedef struct parameter{
  float g_x;
  float g_y;
  float g_z;
  float a_x;
  float a_y;
  float a_z;
  float m_x;
  float m_y;
  float m_z;
  float temp;
}mpuParam;

extern float pitch;
extern float yaw;
extern float roll;
extern void mpuInit();
extern mpuParam mpuRead();
extern void positionAdj(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
#endif