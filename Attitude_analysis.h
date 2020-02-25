#ifndef Attitude_analysis.h
#define Attitude_analysis.h

/*========================================
	describe��
		�ȵ��� analysisInit() ��mpu9150 ak89xx����ʼ��
		Ȼ��� eulerData ȥ���
    plus��
		���������pit0�жϣ�ÿ10ms��һ�����ݲɼ�
==========================================*/



/*========================================
		mpu9150 register
==========================================*/
#define	MPU9150_Addr    0xD0
#define	PWR_MGMT_1		0x6B	
#define	WHO_AM_I		0x75	
#define	SMPLRT_DIV		0x19  //���� 	
#define	CONFIG			0x1A  //����

#define	ACCEL_CONFIG	0x1C  //���ٶ�
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41  //�¶�
#define	TEMP_OUT_L		0x42

#define	GYRO_CONFIG		0x1B  //���ٶ�
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define AK8975_Addr     0x18
#define CNTL            0X0A
#define HXL             0X03  //������
#define HXH             0X04
#define HYL             0X05
#define HYH             0X06
#define HZL             0X07
#define HZH             0X08

/*=================
  mpu9105 param
===================*/
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
}mpu9150Param;
/*================
  euler angle
==================*/
typedef struct eulerangle{
	float yaw;    
    float pitch; 
	float roll;
}euler_angle;
/*================
  engle delax
==================*/
typedef struct anlgedelax{
	float delax_g_x;    
    float delax_g_y; 
	float delax_g_z;
	float delax_a_x;    
    float delax_a_y; 
	float delax_a_z;
	float delax_m_x;    
    float delax_m_y; 
	float delax_m_z;
}angleDelax;


#define Calibration    //�Ƿ����У׼
#define Pi  3.14159265358979323846

/*===============
	param
================*/
extern euler_angle eulerData;
/*===============
	func
================*/
void PIT0_irq(void);
void analysisInit();       
mpu9150Param mpuRead();
euler_angle attitudeAnalysis(float gx, float gy, float gz, 
							 float ax, float ay, float az,
							 float mx, float my, float mz);
#endif