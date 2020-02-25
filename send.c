#include "send.h"
#include "uart.h"
#include "common.h"

void sendInit(){
	UART_QuickInit(UART0_RX_PD06_TX_PD07, 115200);
}

void sendBuffer(uint8_t *buff, uint32_t len){
	while(len--)
	{
		UART_WriteByte(HW_UART0, *buff);
		buff++;
	}
}

void sendWave(void *wareaddr, uint32_t waresize){
	#define CMD_WARE     3
	uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
	uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令
	
	sendBuffer(cmdf               ,  sizeof(cmdf));
	sendBuffer((uint8_t *)wareaddr,  waresize);
	sendBuffer(cmdr               ,  sizeof(cmdf));
}