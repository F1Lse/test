#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#include "stm32f4xx.h"

//#define MINIBALANCE       //��Ҫʹ��VOFA+������λ������ע�ʹ˺�
#define MAX_SEND_NUM 10     //MINIBALANCE���10��ͨ����VOFA+���Գ���10��ͨ��

void DataWave(UART_HandleTypeDef* huart);

#endif

