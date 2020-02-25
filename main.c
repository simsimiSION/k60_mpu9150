#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "send.h"
#include "pit.h"
#include <math.h>
#include "Attitude_analysis.h"
#include "MahonyAHRS.h"
float data[9] = {0};
float position[3] = {0};
float qq[4] = {0};
float probe[3] = {0};
float probe2[3] = {0};
euler_angle eulerData;
int main(void)
{
	DelayInit();
	analysisInit();
	sendInit();
	while(1){
		DWT_DelayMs(20);
		position[0] = eulerData.pitch;
		position[1] = eulerData.roll;
		position[2] = eulerData.yaw;
		sendWave(position, sizeof(position));
	}
}



