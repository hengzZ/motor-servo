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
		if(NULL == gets(cmdstr)) continue;
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
