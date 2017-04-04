#ifndef AM335X_SETTING_H
#define AM335X_SETTING_H

#include "global_setting.h"

// Hypothetical Premise for location protection algorithm.
// 1. 65536 for one cycle.
// 2. Define: max position stride < 65536/36 = 1820 (10 degrees)
#define MAX_STRIDE		1820

extern char* buff;

int listening_uart(const char* device, int baud, char parity, int data_bit, int stop_bit);

void close_uart();

#endif	//AM335X_SETTING_H

