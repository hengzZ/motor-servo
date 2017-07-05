#ifndef AM335X_SETTING_H
#define AM335X_SETTING_H


// 编码器的一周编码
#define E_PULSE_PER_CIRCLE      65535
// 用于极限位置判定，在其偏差范围内的都算到了极限位置
#define E_PULSE_OFFSET          20      // error offset for max location (20/65535 degree)

struct uart_t;
typedef struct uart_t uart_t;

struct uart_t {
    char device[32];
    int baud;
    char parity;
    int data_bit;
    int stop_bit;
};

// 获取当前编码器的位置
int get_encoder_position();
// 设置编码器的限位
void set_max_left_position(int position);
void set_max_right_position(int position);
// 获取编码器的限位值
int get_max_left_position();
int get_max_right_position();

int listening_uart(const char* device, int baud, char parity, int data_bit, int stop_bit);
void close_uart();

#endif	//AM335X_SETTING_H

