#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "elog.h"
#include "modbus.h"

#include "alpha_motion_control.h"


// DataType for control and data transmission
typedef enum{
    GRIGHT=1,
    GLEFT=2,
    GPOINT=4,
    GPST_CANCEL=8,
    GEMG=16,
    GSPEED=32,
    GACCE_TIME=64,
    GDECE_TIME=128,
    GMAX_POINT=256,
    GSTATUS=512,
    GUNKNOWN=1024
} GFLAGS;

typedef struct {
    GFLAGS cmd;
    int32_t v[2];
}param;

// Extern function for updating DataObject
extern void update_g_x(param x);
extern param get_g_x();

// Flag for direction
volatile int anticlockwise;

// Socket
int server_port, queue_size;
int s, b, l, sa;
int on = 1;
pthread_mutex_t mutex_cmd = PTHREAD_MUTEX_INITIALIZER;

int m_socket_read(void *buf, size_t count)
{
    pthread_mutex_lock(&mutex_cmd);
    int bytes = read(sa, buf, count);
    pthread_mutex_unlock(&mutex_cmd);
    return bytes;
}
int m_socket_write(void *buf, size_t count)
{
    pthread_mutex_lock(&mutex_cmd);
    int bytes = write(sa, buf, count);
    pthread_mutex_unlock(&mutex_cmd);
    return bytes;
}


void parsesocket(void)
{
	struct sockaddr_in channel;		// holds IP address

	// Build address structure
	memset(&channel, 0, sizeof(channel));
	channel.sin_family = AF_INET;
	channel.sin_addr.s_addr = htonl(INADDR_ANY);
	channel.sin_port = htons(server_port);

	// Passive open
	s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(s < 0) { 
		log_e("socket failed.");
		assert(0);
		exit(-1);
	}
	setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char*)&on, sizeof(on));
	b = bind(s, (struct sockaddr*)&channel, sizeof(channel));
	if(b < 0) {
		log_e("bind failed.");
		assert(0);
		exit(-1);
	}
	l = listen(s, queue_size);
	if(l < 0) {
		log_e("listen failed.");
		assert(0);
		exit(-1);
	}
	// Init Done
	int bytes = 0;
	char buf[128];
	
	int connect_loop = 1;
	while(1)
	{
		sa = accept(s, 0, 0);
		if(sa < 0) {
			log_e("listening_socket: socket accept failed.");
		}

		connect_loop = 1;
		while(connect_loop) {
			bytes = m_socket_read(buf, 127);
			if(bytes <= 0) {
			    connect_loop = 0;
			    break;
			}
			
			//fflush(sa);
			param temp_x = get_g_x();
			if(buf == strstr(buf,"cancel"))
			{
			    temp_x.cmd = GPST_CANCEL;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"stop"))
			{
			    temp_x.cmd = GEMG;
			    update_g_x(temp_x);
			    //printf("cmdparse: stop\n");
			}
			else if(buf == strstr(buf, "point"))
			{
			    double position;
			    sscanf(buf,"point %.3f",&position);
			    //char tmp_buf[1024];
			    //sprintf(tmp_buf, buf);
			    //log_e(tmp_buf);
			    //printf("cmdparse: %s\n",tmp_buf);
			    //sprintf(tmp_buf, "INF:point cmd position: %f",position);
			    //log_e(tmp_buf);
			    //printf("cmdparse: %s\n",tmp_buf);
			    if(anticlockwise) temp_x.v[0] = -position * 1000;
			    else temp_x.v[0] = position * 1000;
			    temp_x.cmd = GPOINT;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"runleft"))
			{
			    if(anticlockwise) temp_x.cmd = GRIGHT;
			    else temp_x.cmd = GLEFT;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"runright"))
			{
			    if(anticlockwise) temp_x.cmd = GLEFT;
			    else temp_x.cmd = GRIGHT;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"speed"))
			{
			    double speed;
			    sscanf(buf,"speed %.3f",&speed);
			    temp_x.v[0] = speed * 1000;
			    temp_x.cmd = GSPEED;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"acce"))
			{
			    double acce;
			    sscanf(buf, "acce %.3f",&acce);
			    temp_x.v[0] = acce;
			    temp_x.cmd = GACCE_TIME;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"dece"))
			{
			    double dece;
			    sscanf(buf, "dece %.3f",&dece);
			    temp_x.v[0] = dece;
			    temp_x.cmd = GDECE_TIME;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"maxpoint"))
			{
			    double max_left, max_right;
			    sscanf(buf, "maxpoint %.3f %.3f",&max_left, &max_right);
			    if(anticlockwise) max_left = -max_left, max_right = -max_right;
			    if(max_left > max_right){
				double temp = max_right;
				max_right = max_left;
				max_left = temp;
			    }
			    temp_x.v[0] = max_left * 1000;
			    temp_x.v[1] = max_right * 1000;
			    temp_x.cmd = GMAX_POINT;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"status"))
			{
			    int32_t status;
			    sscanf(buf, "status %d",&status);
			    temp_x.v[0] = status;
			    temp_x.cmd = GSTATUS;
			    update_g_x(temp_x);
			}
			usleep(200000);	    // 200ms
		}
		close(sa);
		usleep(200000);	    // 200ms
	}
}

int listening_socket(int _server_port, int _queue_size)
{
	server_port = _server_port;
	queue_size = _queue_size;
	pthread_t parsesocketid;

	if(0 != pthread_create(&parsesocketid,NULL,(void*)parsesocket,NULL)) return -1;
	if(0 != pthread_detach(parsesocketid)) return -1;
	return 0;
}
