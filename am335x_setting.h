#ifndef AM335X_SETTING_H
#define AM335X_SETTING_H

#include "global_setting.h"

extern char* buff;

int listening_uart(const char* device, int baud, char parity, int data_bit, int stop_bit);

void close_uart();

#endif	//AM335X_SETTING_H

