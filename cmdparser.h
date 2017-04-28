#ifndef CMDPARSER_H
#define CMDPARSER_H

#include <stdlib.h>
#include <stdio.h>

struct am335x_socket_t;
typedef struct am335x_socket_t am335x_socket_t;

struct am335x_socket_t{
        int server_port;
        int queue_size;
};

int listening_socket(int server_port, int queue_size);

#endif  // CMDPARSER_H
