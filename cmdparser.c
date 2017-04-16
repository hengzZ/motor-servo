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

// Socket
int server_port, queue_size;
int s, b, l, sa;
int on = 1;

// Contrl flags
typedef enum{
    GRIGHT=1,
    GLEFT=2,
    GCRUISE=4,
    GPAUSE=8,
    GPST_CANCEL=16,
    GFREE_ON=32,
    GEMG=64,
    GSPEED=128,
    GACCE_TIME=256,
    GDECE_TIME=512,
    GRIGHT_PST=1024,
    GLEFT_PST=2048,
    GPAUSE_OFF=4096
} GFLAGS;
extern void write_gflags(uint32_t flags);
extern uint32_t read_gflags();

void parsethread(void)
{
	char cmdstr[1024];
	fprintf(stderr,"Please Enter a command.\n");
	fprintf(stderr,"[stop] [pause_on] [pause_off] [lcruise] [rcruise] [ldirect] [rdirect] [cancel]\n");
	while(1)
	{
		if(!gets(cmdstr)) continue;
		if(0 == strcmp(cmdstr,"stop"))
		{
			uint32_t temp = GEMG;
			printf("temp: %.8d\n",temp);
			write_gflags(GEMG);
			printf("%.8d\n",read_gflags());
		}
		if(0 == strcmp(cmdstr,"pause_on"))
		{
			uint32_t temp = read_gflags();
			temp = temp & (~GPAUSE_OFF) | GPAUSE;
			printf("temp: %.8d\n",temp);
			write_gflags(temp);
			printf("%.8d\n",read_gflags());
		}
		if(0 == strcmp(cmdstr, "pause_off"))
		{
			uint32_t temp = read_gflags();
			temp = temp & (~GPAUSE) | GPAUSE_OFF;
			printf("temp: %.8d\n",temp);
			write_gflags(temp);
			printf("%.8d\n",read_gflags());
		}
		if(0 == strcmp(cmdstr,"lcruise"))
		{
			int temp = read_gflags();
			temp = temp & (~GPAUSE) & (~GRIGHT) | GPAUSE_OFF;
			temp = temp | GCRUISE | GLEFT;
			printf("temp: %.8d\n",temp);
			write_gflags(temp);
			printf("%.8d\n",read_gflags());
		}
		if(0 == strcmp(cmdstr,"rcruise"))
		{
			int temp = read_gflags();
			temp = temp & (~GPAUSE) & (~GLEFT) | GPAUSE_OFF;
			temp = temp | GCRUISE | GRIGHT;
			printf("temp: %.8d\n",temp);
			write_gflags(temp);
			printf("%.8d\n",read_gflags());
		}
		if(0 == strcmp(cmdstr,"ldirect"))
		{
			int temp = read_gflags();
			temp = temp & (~GPAUSE) & (~GCRUISE) & (~GRIGHT) | GPAUSE_OFF;
			temp = temp | GLEFT;
			printf("temp: %.8d\n",temp);
			write_gflags(temp);
			printf("%.8d\n",read_gflags());
		}
		if(0 == strcmp(cmdstr,"rdirect"))
		{
			int temp = read_gflags();
			temp = temp & (~GPAUSE) & (~GCRUISE) & (~GLEFT) | GPAUSE_OFF;
			temp = temp | GRIGHT;
			printf("temp: %.8d\n",temp);
			write_gflags(temp);
			printf("%.8d\n",read_gflags());
		}
		if(0 == strcmp(cmdstr,"cancel"))
		{
			int temp = GPST_CANCEL;
			printf("temp: %.8d\n",temp);
			write_gflags(GPST_CANCEL);
			printf("%.8d\n",read_gflags());
		}

		fprintf(stderr,"Please Enter a command.\n");
		fprintf(stderr,"[stop] [pause_on] [pause_off] [lcruise] [rcruise] [ldirect] [rdirect] [cancel]\n");
	}
}

int listening_console()
{
	int ret;
	pthread_t parserid;
	pthread_create(&parserid,NULL,(void*)parsethread,NULL);
	pthread_detach(parserid);
	return 0;
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
	}
	setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char*)&on, sizeof(on));
	b = bind(s, (struct sockaddr*)&channel, sizeof(channel));
	if(b < 0) {
		log_e("bind failed.");
		assert(0);
	}
	l = listen(s, queue_size);
	if(l < 0) {
		log_e("listen failed.");
		assert(0);
	}
	// Well Done
	
	while(1)
	{
		sa = accept(s, 0, 0);
		if(sa < 0) {
			log_e("accept failed.");
			assert(0);
		}

		int bytes = 0;
		char buf[1024];

		while(1) {
			bytes = read(sa, buf, 1024);
			if(bytes <= 0 ) continue;
			fprintf(stderr,"%s\n",buf);

			if(0 == strcmp(buf,"stop"))
			{
			    uint32_t temp = GEMG;
			    printf("temp: %.8d\n",temp);
			    write_gflags(GEMG);
			    printf("%.8d\n",read_gflags());
			}
			if(0 == strcmp(buf,"pause_on"))
			{
			    uint32_t temp = read_gflags();
			    temp = temp & (~GPAUSE_OFF) | GPAUSE;
			    printf("temp: %.8d\n",temp);
			    write_gflags(temp);
			    printf("%.8d\n",read_gflags());
			}
			if(0 == strcmp(buf, "pause_off"))
			{
			    uint32_t temp = read_gflags();
			    temp = temp & (~GPAUSE) | GPAUSE_OFF;
			    printf("temp: %.8d\n",temp);
			    write_gflags(temp);
			    printf("%.8d\n",read_gflags());
			}
			if(0 == strcmp(buf,"lcruise"))
			{
			    int temp = read_gflags();
			    temp = temp & (~GPAUSE) & (~GRIGHT) | GPAUSE_OFF;
			    temp = temp | GCRUISE | GLEFT;
			    printf("temp: %.8d\n",temp);
			    write_gflags(temp);
			    printf("%.8d\n",read_gflags());
			}
			if(0 == strcmp(buf,"rcruise"))
			{
			    int temp = read_gflags();
			    temp = temp & (~GPAUSE) & (~GLEFT) | GPAUSE_OFF;
			    temp = temp | GCRUISE | GRIGHT;
			    printf("temp: %.8d\n",temp);
			    write_gflags(temp);
			    printf("%.8d\n",read_gflags());
			}
			if(0 == strcmp(buf,"ldirect"))
			{
			    int temp = read_gflags();
			    temp = temp & (~GPAUSE) & (~GCRUISE) & (~GRIGHT) | GPAUSE_OFF;
			    temp = temp | GLEFT;
			    printf("temp: %.8d\n",temp);
			    write_gflags(temp);
			    printf("%.8d\n",read_gflags());
			}
			if(0 == strcmp(buf,"rdirect"))
			{
			    int temp = read_gflags();
			    temp = temp & (~GPAUSE) & (~GCRUISE) & (~GLEFT) | GPAUSE_OFF;
			    temp = temp | GRIGHT;
			    printf("temp: %.8d\n",temp);
			    write_gflags(temp);
			    printf("%.8d\n",read_gflags());
			}
			if(0 == strcmp(buf,"cancel"))
			{
			    int temp = GPST_CANCEL;
			    printf("temp: %.8d\n",temp);
			    write_gflags(GPST_CANCEL);
			    printf("%.8d\n",read_gflags());
			}
		}
		close(sa);
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
