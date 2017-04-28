#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>

#include "modbus.h"
#include "elog.h"

#include "alpha_motion_control.h"

// Socket
int server_port, queue_size;
int s, b, l, sa;
int on = 1;

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
			bytes = read(sa, buf, 127);
			if(bytes <= 0 ) {
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
			    printf("cmdparse: stop\n");
			}
			else if(buf == strstr(buf, "point"))
			{
			    int32_t position;
			    sscanf(buf,"point %d",&position);
			    char tmp_buf[1024];
			    sprintf(tmp_buf, buf);
			    log_e(tmp_buf);
			    printf("cmdparse: %s\n",tmp_buf);
			    sprintf(tmp_buf, "INF:point cmd position: %d",position);
			    log_e(tmp_buf);
			    printf("cmdparse: %s\n",tmp_buf);
			    temp_x.v[0] = position;
			    temp_x.cmd = GPOINT;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"runleft"))
			{
			    temp_x.cmd = GLEFT;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"runright"))
			{
			    temp_x.cmd = GRIGHT;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"speed"))
			{
			    int32_t speed;
			    sscanf(buf,"speed %d",&speed);
			    temp_x.v[0] = speed;
			    temp_x.cmd = GSPEED;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"acce"))
			{
			    int32_t acce;
			    sscanf(buf, "acce %d",&acce);
			    temp_x.v[0] = acce;
			    temp_x.cmd = GACCE_TIME;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"dece"))
			{
			    int32_t dece;
			    sscanf(buf, "dece %d",&dece);
			    temp_x.v[0] = dece;
			    temp_x.cmd = GDECE_TIME;
			    update_g_x(temp_x);
			}
			else if(buf == strstr(buf,"maxpoint"))
			{
			    int32_t max_left, max_right;
			    sscanf(buf, "maxpoint %d %d",&max_left, &max_right);
			    temp_x.v[0] = max_left;
			    temp_x.v[1] = max_right;
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

	pthread_create(&parsesocketid,NULL,(void*)parsesocket,NULL);
	pthread_detach(parsesocketid);
	return 0;
}
