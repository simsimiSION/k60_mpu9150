#ifndef _SEND_H_
#define _SEND_H_
#include "common.h"

extern void sendInit();
extern void sendBuffer(uint8_t *buff, uint32_t len);
extern void sendWave(void *wareaddr, uint32_t waresize);

#endif