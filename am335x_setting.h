#ifndef AM335X_SETTING_H
#define AM335X_SETTING_H


#define E_PULSE_PER_CIRCLE      65535

struct uart_t;
typedef struct uart_t uart_t;

struct uart_t {
    char device[32];
    int baud;
    char parity;
    int data_bit;
    int stop_bit;
};

int get_encoder_position();
void set_max_left_position(int position);
void set_max_right_position(int position);
int get_max_left_position();
int get_max_right_position();

int listening_uart(const char* device, int baud, char parity, int data_bit, int stop_bit);
void close_uart();

#endif	//AM335X_SETTING_H

