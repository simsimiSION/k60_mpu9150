#include "Attitude_analysis.h"
#include "MahonyAHRS.h"
#include "IIC.h"
#include "pit.h"
#include "common.h"
#include <math.h>


/* func:     mpuInit()
 * describe: init mpu9150
 * return:   null
 */
void analysisInit(){
	I2C_init();
	
	uint8_t loop = 3; // prevent mpu9150 init fail
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
	
	PIT_QuickInit(HW_PIT_CH0, 10 * 1000);            //定时 10ms
 	PIT_CallbackInstall(HW_PIT_CH0, PIT0_irq); 		 //注册回调函数
	PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE); //开启模块0通道中断
}

/* func:     read_g_x()
 * describe: read gravity force from x_axis
 * return:   x_axis gravity
 */
int16_t read_g_x(){
	uint8_t G_T_X1 = 0, G_T_X2 = 0;
	I2CreadByte(MPU9150_Addr,GYRO_XOUT_L,&G_T_X1);
	I2CreadByte(MPU9150_Addr,GYRO_XOUT_H,&G_T_X2);
	return (G_T_X2<<8)|G_T_X1;
}

/* func:     read_g_y()
 * describe: read gravity force from y_axis
 * return:   y_axis gravity
 */
int16_t read_g_y(){
	uint8_t G_T_Y1 = 0, G_T_Y2 = 0;
	I2CreadByte(MPU9150_Addr,GYRO_YOUT_L,&G_T_Y1);
	I2CreadByte(MPU9150_Addr,GYRO_YOUT_H,&G_T_Y2);
	return(G_T_Y2<<8)|G_T_Y1;
}

/* func:     read_g_z()
 * describe: read gravity force from z_axis
 * return:   z_axis gravity
 */
int16_t read_g_z(){
	uint8_t G_T_Z1 = 0, G_T_Z2 = 0;
	I2CreadByte(MPU9150_Addr,GYRO_ZOUT_L,&G_T_Z1) ;
	I2CreadByte(MPU9150_Addr,GYRO_ZOUT_H,&G_T_Z2) ;
	return(G_T_Z2<<8)|G_T_Z1; 
}

/* func:     read_a_x()
 * describe: read accelerated velocity from x_axis
 * return:   x_axis accelerated velocity
 */
int16_t read_a_x(){
	uint8_t A_T_X1 = 0, A_T_X2 = 0;
	I2CreadByte(MPU9150_Addr,ACCEL_XOUT_L,&A_T_X1) ;
	I2CreadByte(MPU9150_Addr,ACCEL_XOUT_H,&A_T_X2) ;
	return(A_T_X2<<8)|A_T_X1;
}

/* func:     read_a_y()
 * describe: read accelerated velocity from y_axis
 * return:   y_axis accelerated velocity
 */
int16_t read_a_y(){
	uint8_t A_T_Y1 = 0, A_T_Y2 = 0;
	I2CreadByte(MPU9150_Addr,ACCEL_YOUT_L,&A_T_Y1) ;
	I2CreadByte(MPU9150_Addr,ACCEL_YOUT_H,&A_T_Y2) ;
	return(A_T_Y2<<8)|A_T_Y1;
}

/* func:     read_a_z()
 * describe: read accelerated velocity from z_axis
 * return:   z_axis accelerated velocity
 */
int16_t read_a_z(){
	uint8_t A_T_Z1 = 0, A_T_Z2 = 0;
	I2CreadByte(MPU9150_Addr,ACCEL_ZOUT_L,&A_T_Z1) ;
	I2CreadByte(MPU9150_Addr,ACCEL_ZOUT_H,&A_T_Z2) ;
	return(A_T_Z2<<8)|A_T_Z1;
}

/* func:     read_temp()
 * describe: read temperature
 * return:   temperature
 */
int16_t read_temp(){
	uint8_t temp1 = 0, temp2 = 0;
	I2CreadByte(MPU9150_Addr,TEMP_OUT_L,&temp2) ;
	I2CreadByte(MPU9150_Addr,TEMP_OUT_H,&temp1) ;
	return(temp1<<8)|temp2;
}

/* func:     read_m_x()
 * describe: read magnetic force from x_axis
 * return:   x_axis magnetic force
 */
int16_t read_m_x(){
	uint8_t comp1_x = 0, comp2_x = 0;
	I2CreadByte(AK8975_Addr,HXL,&comp2_x) ;
	I2CreadByte(AK8975_Addr,HXH,&comp1_x) ;
	return(comp1_x<<8)|comp2_x;
}

/* func:     read_m_y()
 * describe: read magnetic force from y_axis
 * return:   y_axis magnetic force
 */
int16_t read_m_y(){
	uint8_t comp1_y = 0, comp2_y = 0;
	I2CreadByte(AK8975_Addr,HYL,&comp2_y) ;
	I2CreadByte(AK8975_Addr,HYH,&comp1_y) ;
	return(comp1_y<<8)|comp2_y;
}

/* func:     read_m_z()
 * describe: read magnetic force from z_axis
 * return:   z_axis magnetic force
 */
int16_t read_m_z(){
	uint8_t comp1_z = 0, comp2_z = 0;
	I2CreadByte(AK8975_Addr,HZL,&comp2_z) ;
	I2CreadByte(AK8975_Addr,HZH,&comp1_z) ;
	return(comp1_z<<8)|comp2_z;
}

/* func:     mpugetParam()
 * describe: get mpu pararm
 * return:   mpuParam data
 */
mpu9150Param mpugetParam(){
	mpu9150Param data;
	
	data.g_x = read_g_x() / 16.4 *Pi/180.0f; 
	data.g_y = read_g_y() / 16.4 *Pi/180.0f;
	data.g_z = read_g_z() / 16.4 *Pi/180.0f;
	data.a_x = read_a_x() / 16384.0;
	data.a_y = read_a_y() / 16384.0;
	data.a_z = read_a_z() / 16384.0;
	data.m_x = read_m_x();
	data.m_y = read_m_y();
	data.m_z = read_m_z();
	I2CwriteByte(AK8975_Addr, CNTL, 0x01) ; // 改ak8975mode
	data.temp = read_temp() / 340 + 36.53; 
	
	return data;
}

/* func:     mpuAdjust()
 * describe: mpu Adjust pararm
 * return:   Adjust pararm
 */
angleDelax mpuAdjust(){
	angleDelax   mpu_D;	
	mpu9150Param mpuTest[20] = {0};
	mpu9150Param mpuDelaxData;
	
	mpuDelaxData.g_x = 0;                      //init
	mpuDelaxData.g_y = 0;
	mpuDelaxData.g_z = 0;
	mpuDelaxData.a_x = 0;
	mpuDelaxData.a_y = 0;
	mpuDelaxData.a_z = 0;

	
	for(int i = 0; i < 20; i++){              //read param
		mpuTest[i] = mpugetParam();
		DWT_DelayMs(50);
	}
	for(int i = 0; i < 20; i++){              //param sum
		mpuDelaxData.g_x += mpuTest[i].g_x;
		mpuDelaxData.g_y += mpuTest[i].g_y;
		mpuDelaxData.g_z += mpuTest[i].g_z;
		mpuDelaxData.a_x += mpuTest[i].a_x;
		mpuDelaxData.a_y += mpuTest[i].a_y;
		mpuDelaxData.a_z += mpuTest[i].a_z;
	}
	
	mpuDelaxData.g_x = mpuDelaxData.g_x / 20;   //get delax
	mpuDelaxData.g_y = mpuDelaxData.g_y / 20;
	mpuDelaxData.g_z = mpuDelaxData.g_z / 20;
	mpuDelaxData.a_x = mpuDelaxData.a_x / 20;
	mpuDelaxData.a_y = mpuDelaxData.a_y / 20;
	mpuDelaxData.a_z = mpuDelaxData.a_z / 20;
	
	mpu_D.delax_g_x = 0 - mpuDelaxData.g_x;
	mpu_D.delax_g_y = 0 - mpuDelaxData.g_y;
	mpu_D.delax_g_z = 0 - mpuDelaxData.g_z;
	mpu_D.delax_a_x = 0 - mpuDelaxData.a_x;
	mpu_D.delax_a_y = 0 - mpuDelaxData.a_y;
	mpu_D.delax_a_z = 1 - mpuDelaxData.a_z;
	
	return mpu_D;
}

/* func:     mpuRead()
 * describe: read mpu9150 param
 * return:   mpuParam data
 */

/*********Calibration*************/
#ifdef Calibration
int Calibration_flag = 1;
#else
int Calibration_flag = 0;
#endif
angleDelax angle_D;
/********************************/
mpu9150Param mpuRead(){
	
	mpu9150Param data;
	
	if(Calibration_flag == 1){
		angle_D = mpuAdjust();
		Calibration_flag  = 2;
	}else if(Calibration_flag == 0){
		angle_D.delax_g_x = 0;
		angle_D.delax_g_y = 0;
		angle_D.delax_g_z = 0;
		angle_D.delax_a_x = 0;
		angle_D.delax_a_y = 0;
		angle_D.delax_a_z = 0;
	}
	
	data.g_x = read_g_x() / 16.4 *Pi/180.0f + angle_D.delax_g_x; 
	data.g_y = read_g_y() / 16.4 *Pi/180.0f + angle_D.delax_g_y;
	data.g_z = read_g_z() / 16.4 *Pi/180.0f + angle_D.delax_g_z;
	data.a_x = read_a_x() / 16384.0         + angle_D.delax_a_x;
	data.a_y = read_a_y() / 16384.0         + angle_D.delax_a_y;
	data.a_z = read_a_z() / 16384.0         + angle_D.delax_a_z;
	data.m_x = read_m_x()                   ;
	data.m_y = read_m_y()                   ;
	data.m_z = read_m_z()                   ;
	I2CwriteByte(AK8975_Addr, CNTL, 0x01) ; // 改ak8975mode
	data.temp = read_temp() / 340 + 36.53; 
	
	return data;
}

euler_angle attitudeAnalysis(float gx, float gy, float gz, float ax, float ay, float az,float mx, float my, float mz){
	euler_angle data;
	
	MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);  // an outstanding func
	/*change q to euler*/
	data.yaw    =  atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);   
    data.pitch  =  -asin(2.0f * (q1 * q3 - q0 * q2));
	data.roll   =  atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
	/*change euler sort*/
	data.pitch  =  data.pitch * 180.0f / Pi;   //y_axis
	data.yaw    =  data.yaw   * 180.0f / Pi;   //z_axis
	data.roll   = -data.roll  * 180.0f / Pi;   //x_axis
	
	return data;	
}

void PIT0_irq(){	
	mpu9150Param mpuData = mpuRead();
	euler_angle eular;
	eulerData = attitudeAnalysis(mpuData.g_x, mpuData.g_y, mpuData.g_z, 
							     mpuData.a_x, mpuData.a_y, mpuData.a_z,
							     mpuData.m_x, mpuData.m_y, mpuData.m_z);

}