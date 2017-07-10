/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*-  */
/*
 * main.c
 * Copyright (C) 2017  <>
 * 
 * motor-term is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * motor-term is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h> 
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <netinet/in.h>
#include <netdb.h>


#define BUFFSIZE     (64)	 
#define MY_PORT	    (2222)
#define SERVER_PORT (2223)


int nprintline=0;


const char * SERVER_IP = "127.0.0.1";
static int sock = -1;

struct sockaddr_in serveraddr;    
struct sockaddr_in myaddr;   

static int get_command();


static void  init_sock()
{
	
	 if( -1 == sock )
	 {
    
	    memset(&myaddr, 0, sizeof(myaddr));
	    myaddr.sin_family = AF_INET;
	    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	    myaddr.sin_port = htons(MY_PORT);
	    
	    
	    memset(&serveraddr, 0, sizeof(serveraddr));
	    serveraddr.sin_family = AF_INET;
	    serveraddr.sin_addr.s_addr = inet_addr(SERVER_IP);
	   
	    serveraddr.sin_port = htons(SERVER_PORT);

	    // socket
	    sock = socket(AF_INET, SOCK_DGRAM, 0);
	    if(sock < 0)
	    {
	    	perror("sock");
	        exit(-1);
	    }

	    int flags = fcntl(sock, F_GETFL, 0); // 获取当前状态
	    fcntl(sock, F_SETFL, flags|O_NONBLOCK); // 非阻塞设置

	    // bind
	    if(bind(sock, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0){
	        perror("bind");
	        exit(-1);
	    }
    }

}

int sendbuf( char* buf, size_t len )
{
	init_sock();
	
	int r = sendto(sock, buf, len, 0, (struct sockaddr *)&serveraddr, sizeof(serveraddr) );
	
	//printf( "send:%s,%d\n",buf,r );
	
	return r;

}


void recvsocket(void)
{

   	init_sock();
	// Init Done
	int bytes = 0;
	char buf[128];
	struct sockaddr_in addr; 
	int len = sizeof(addr);
	
	
	while(1)
	{
	
		bytes = recvfrom(sock, buf, 127, 0, (struct sockaddr*)&addr, &len);
		
		if( bytes > 0 )
		{
			buf[bytes]=0;
			//printf("recv:%s\n",buf);
			nprintline++;
			
			if( nprintline == 20 )
			{
				fputs("\033[2J",stderr);
				fputs("\033[0;0H",stderr);
				nprintline=0;
			}
			
			//fputs("\033[K" ,stderr);//清除从光标到行尾的内容
			
			fputs("recv:", stderr);
			fputs(buf, stderr);
			//fprintf(stderr,"recv:%s\n",buf);
		
		}	
	
		
		usleep(1000); 
	}
	
	
}

int main()
{
		

	pthread_t parsesocketid;

	if(0 != pthread_create(&parsesocketid,NULL,(void*)recvsocket,NULL)) return -1;
	if(0 != pthread_detach(parsesocketid)) return -1;
	
	for(;;)
	{
	
		get_command();
		
		usleep(1000);
		
	}
	return (0);
}

static int get_command()
{
    fd_set rfds;
    struct timeval tv;
    int n=0;
	char buf_show[BUFFSIZE];
	
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10; //timeout
    //any key
	
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
		n = read(STDIN_FILENO, buf_show, BUFFSIZE);
		buf_show[n-1]=0;		
	
		if( strcmp(buf_show,"exit" ) ==0 ||
		         strcmp(buf_show,"q" ) ==0)
        {
			printf( "exit\n");
			exit(1);
		}
		else
		{
			sendbuf( buf_show, strlen(buf_show));
			
		}
	
	}

    return 0;
}
