#include "mpu9150.h"
#include "common.h"
#include "IIC.h"
#include "ahrs.h"
/*
referenced param
*/
float PI = 3.14159265358979323846f;
float pitch;
float yaw;
float roll;

/*
referenced func
*/
void mpuInit(){
	uint8_t loop = 3;
	
	I2C_init();
	while(loop --> 0){
		I2CwriteByte(MPU9150_Addr, PWR_MGMT_1  , 0x00);
		I2CwriteByte(MPU9150_Addr, SMPLRT_DIV  , 0x07);
		I2CwriteByte(MPU9150_Addr, CONFIG      , 0x06);
		I2CwriteByte(MPU9150_Addr, GYRO_CONFIG , 0x18);
		I2CwriteByte(MPU9150_Addr, ACCEL_CONFIG, 0x01);
		I2CwriteByte(MPU9150_Addr, 0x37, 0x02);
		I2CwriteByte(MPU9150_Addr, 0x6A, 0x00);
		I2CwriteByte(AK8975_Addr , 0x0A, 0x01);
	
	}
}

int16_t read_g_x(){
	uint8_t G_T_X1 = 0, G_T_X2 = 0;
	I2CreadByte(MPU9150_Addr,GYRO_XOUT_L,&G_T_X1);
	I2CreadByte(MPU9150_Addr,GYRO_XOUT_H,&G_T_X2);
	return (G_T_X2<<8)|G_T_X1;
}
int16_t read_g_y(){
	uint8_t G_T_Y1 = 0, G_T_Y2 = 0;
	I2CreadByte(MPU9150_Addr,GYRO_YOUT_L,&G_T_Y1);
	I2CreadByte(MPU9150_Addr,GYRO_YOUT_H,&G_T_Y2);
	return(G_T_Y2<<8)|G_T_Y1;
}
int16_t read_g_z(){
	uint8_t G_T_Z1 = 0, G_T_Z2 = 0;
	I2CreadByte(MPU9150_Addr,GYRO_ZOUT_L,&G_T_Z1) ;
	I2CreadByte(MPU9150_Addr,GYRO_ZOUT_H,&G_T_Z2) ;
	return(G_T_Z2<<8)|G_T_Z1; 
}
int16_t read_a_x(){
	uint8_t A_T_X1 = 0, A_T_X2 = 0;
	I2CreadByte(MPU9150_Addr,ACCEL_XOUT_L,&A_T_X1) ;
	I2CreadByte(MPU9150_Addr,ACCEL_XOUT_H,&A_T_X2) ;
	return(A_T_X2<<8)|A_T_X1;
}
int16_t read_a_y(){
	uint8_t A_T_Y1 = 0, A_T_Y2 = 0;
	I2CreadByte(MPU9150_Addr,ACCEL_YOUT_L,&A_T_Y1) ;
	I2CreadByte(MPU9150_Addr,ACCEL_YOUT_H,&A_T_Y2) ;
	return(A_T_Y2<<8)|A_T_Y1;
}
int16_t read_a_z(){
	uint8_t A_T_Z1 = 0, A_T_Z2 = 0;
	I2CreadByte(MPU9150_Addr,ACCEL_ZOUT_L,&A_T_Z1) ;
	I2CreadByte(MPU9150_Addr,ACCEL_ZOUT_H,&A_T_Z2) ;
	return(A_T_Z2<<8)|A_T_Z1;
}

int16_t read_temp(){
	uint8_t temp1 = 0, temp2 = 0;
	I2CreadByte(MPU9150_Addr,TEMP_OUT_L,&temp2) ;
	I2CreadByte(MPU9150_Addr,TEMP_OUT_H,&temp1) ;
	return(temp1<<8)|temp2;
}

int16_t read_compass_x(){
	uint8_t comp1_x = 0, comp2_x = 0;
	I2CreadByte(AK8975_Addr,HXL,&comp2_x) ;
	I2CreadByte(AK8975_Addr,HXH,&comp1_x) ;
	return(comp1_x<<8)|comp2_x;
}

int16_t read_compass_y(){
	uint8_t comp1_y = 0, comp2_y = 0;
	I2CreadByte(AK8975_Addr,HYL,&comp2_y) ;
	I2CreadByte(AK8975_Addr,HYH,&comp1_y) ;
	return(comp1_y<<8)|comp2_y;
}

int16_t read_compass_z(){
	uint8_t comp1_z = 0, comp2_z = 0;
	I2CreadByte(AK8975_Addr,HZL,&comp2_z) ;
	I2CreadByte(AK8975_Addr,HZH,&comp1_z) ;
	return(comp1_z<<8)|comp2_z;
}

mpuParam mpuRead(){
	mpuParam data;
	data.g_x = read_g_x() / 16.4; 
	data.g_y = read_g_y() / 16.4;
	data.g_z = read_g_z() / 16.4;
	data.a_x = read_a_x() / 16384.0;
	data.a_y = read_a_y() / 16384.0;
	data.a_z = read_a_z() / 16384.0;
	data.m_x = read_compass_x();
	data.m_y = read_compass_y();
	data.m_z = read_compass_z();
	I2CwriteByte(AK8975_Addr, CNTL, 0x01) ; // ¸Äak8975mode
	data.temp = read_temp() / 340 + 36.53; 
	
	return data;
}

void positionAdj(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz){
	MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
	float q[4];
	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
//	  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
//    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
//    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//	probe[0] = (2.0 * (q0*q3 + q1*q2) ) / (1 - 2 * ( q0*q0 + q1*q1));
//	probe[1] = 2.0 * (q3*q1 - q0*q2);
//	probe[2] = (2.0* (q2*q3 + q0*q1)) / (1 - 2 * (q1*q1 + q2*q2));
//	probe2[0] = atan( probe[0] );
//	probe2[1] = asin(probe[1]);
//	probe2[2] = atan(probe[2]);
	//    pitch *= 180.0f / PI;
//    yaw   *= 180.0f / PI; 
//    roll  *= 180.0f / PI;
}
