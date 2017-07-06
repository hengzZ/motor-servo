#ifndef CMDPARSER_H
#define CMDPARSER_H

#include <stdlib.h>
#include <stdio.h>


// 套接字参数对象格式
struct am335x_socket_t;
typedef struct am335x_socket_t am335x_socket_t;

struct am335x_socket_t{
        int server_port;
        int queue_size;
};


// 用于运动方面的反转，安装时用于调整默认方向
void set_anticlockwise(int mode);
int get_anticlockwise();
// 网络数据的收发
int m_socket_read(void *buf, size_t count);
int m_socket_write(void *buf, size_t count);

int listening_socket(int server_port, int queue_size);

#endif  // CMDPARSER_H
