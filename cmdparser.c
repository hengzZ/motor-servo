#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "modbus.h"

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
    GLEFT_PST=2048
} GFLAGS;
extern void write_gflags(uint32_t flags);
extern uint32_t read_gflags();

void parsethread(void)
{
	char cmdstr[1024];
	char* ptr;
	fprintf(stderr,"Please Enter a command.\n");
	fprintf(stderr,"[stop] [pause] [cruise] [ldirect] [rdirect]\n");
	while(1)
	{
		if(NULL == gets(cmdstr)) continue;
		if(0 == strcmp(cmdstr,"stop"))
		{
			write_gflags(GEMG);
		}
		if(0 == strcmp(cmdstr,"pause"))
		{
			write_gflags(GPAUSE);
		}
		if(0 == strcmp(cmdstr,"cruise"))
		{
			int temp = read_gflags();
			temp = temp & (~GPAUSE);
			temp = temp | GCRUISE | GLEFT;
			write_gflags(temp);
		}
		if(0 == strcmp(cmdstr,"ldirect"))
		{
			int temp = read_gflags();
			temp = temp & (~GPAUSE);
			temp = temp | GLEFT;
			write_gflags(temp);
		}
		if(0 == strcmp(cmdstr,"rdirect"))
		{
			int temp = read_gflags();
			temp = temp & (~GPAUSE);
			temp = temp | GRIGHT;
			write_gflags(temp);
		}

		fprintf(stderr,"Please Enter a command.\n");
		fprintf(stderr,"[stop] [pause] [cruise] [ldirect] [rdirect]\n");
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
