#include     <stdio.h> 
#include     <stdlib.h>
#include     <string.h>
#include     <unistd.h>
#include     <sys/types.h>
#include     <sys/stat.h>  
#include     <fcntl.h>  
#include     <termios.h>
#include     <errno.h> 
#include     <pthread.h> 
#include     <sys/ioctl.h> 

#include    "elog.h"

#define FALSE 1
#define TRUE 0

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

// Assume that every stride is less than half cycle(65535/2).
// Absolute stride large than 32768 means stride over zero.
#define MAX_STRIDE	32767
char sorted_buff[2];
unsigned short  cur_v;
unsigned short  pre_v;
int stride;
volatile int max_left_position;
volatile int max_right_position;
volatile int encoder_position;
pthread_mutex_t mutex_encoder = PTHREAD_MUTEX_INITIALIZER;

void update_encoder_position(int position)
{
    pthread_mutex_lock(&mutex_encoder);
    encoder_position = position;
    pthread_mutex_unlock(&mutex_encoder);
}
int get_encoder_position()
{
    pthread_mutex_lock(&mutex_encoder);
    int position = encoder_position;
    pthread_mutex_unlock(&mutex_encoder);
    return position;
}

void set_max_left_position(int position)
{
    pthread_mutex_lock(&mutex_encoder);
    max_left_position = position;
    pthread_mutex_unlock(&mutex_encoder);
}
void set_max_right_position(int position)
{
    pthread_mutex_lock(&mutex_encoder);
    max_right_position = position;
    pthread_mutex_unlock(&mutex_encoder);
}
int get_max_left_position()
{
    int position;
    pthread_mutex_lock(&mutex_encoder);
    position = max_left_position;
    pthread_mutex_unlock(&mutex_encoder);
    return position;
}
int get_max_right_position()
{
    int position;
    pthread_mutex_lock(&mutex_encoder);
    position = max_right_position;
    pthread_mutex_unlock(&mutex_encoder);
    return position;
}


int fd=-1;
char buff[512];
int speed_arr[] = {B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200};
int name_arr[] = {115200, 57600, 38400,  19200,  9600,  4800,  2400, 1200};

#define debugnum(data,len,prefix)  \
{ \
        unsigned int i;   \
        for (i = 0;i < len;i++) { \
                if(prefix)  \
                        printf("0x%02x ",data[i]); \
                else  \
                        printf("%02x ",data[i]); \
        } \
}

void set_speed(int fd, int speed)
{
    int   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
   	if (speed == name_arr[i])
   	{
   	    tcflush(fd, TCIOFLUSH);
	    cfsetispeed(&Opt, speed_arr[i]);
	    cfsetospeed(&Opt, speed_arr[i]);
	    status = tcsetattr(fd, TCSANOW, &Opt);
	    if (status != 0)
		perror("tcsetattr fd1");
	    return;
     	}
	tcflush(fd,TCIOFLUSH);
    }
}
int set_Parity(int fd,int databits,int stopbits,int parity)
{
    struct termios options;
    if  ( tcgetattr( fd,&options)  !=  0)
    {
  	perror("SetupSerial 1");
  	return(FALSE);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 7:
	options.c_cflag |= CS7;
	break;
    case 8:
	options.c_cflag |= CS8;
	break;
    default:
	fprintf(stderr,"Unsupported data size\n");
	return (FALSE);
    }
    switch (parity)
    {
    case 'n':
    case 'N':
	options.c_cflag &= ~PARENB;   
	options.c_iflag &= ~INPCK;   
	break;
    case 'o':
    case 'O':
	options.c_cflag |= (PARODD | PARENB); 
	options.c_iflag |= INPCK;           
	break;
    case 'e':
    case 'E':
	options.c_cflag |= PARENB;     
	options.c_cflag &= ~PARODD;
	options.c_iflag |= INPCK;     
	break;
    case 'S':
    case 's':  
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	break;
    default:
	fprintf(stderr,"Unsupported parity\n");
	return (FALSE);
    }
    switch (stopbits)
    {
    case 1:
	options.c_cflag &= ~CSTOPB;
	break;
    case 2:
	options.c_cflag |= CSTOPB;
	break;
    default:
	fprintf(stderr,"Unsupported stop bits\n");
	return (FALSE);
    }


	options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);

    /* Set input parity option */
    
    if (parity != 'n')
    options.c_iflag |= INPCK;
    options.c_cc[VTIME] = 150; // 15 seconds
    options.c_cc[VMIN] = 0;
    
    
    tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
	perror("SetupSerial 3");
	return (FALSE);
    }
    return (TRUE);
}


void receivethread(void)
{
    int nread;

    for(;;) {
        if((nread = read(fd,buff,100))>0) 
        {
            buff[nread]='\0';
	    memcpy(&sorted_buff[1],buff+3,1);
	    memcpy(&sorted_buff[0],buff+4,1);
	    memcpy(&cur_v,sorted_buff,2);
            pre_v = cur_v;
            break;
        }
	usleep(1000);	// 1ms
    }
    while(1) 
    {
	if((nread = read(fd,buff,100))>0) 
	{
	    buff[nread]='\0';
	    //printf("[RECEIVE LEN] is %d, content is:\n",nread);
	    //for(int i = 0; i < nread; i++){
	    //	fprintf(stderr,"%.2x ",buff[i]);
	    //}
	    memcpy(&sorted_buff[1],buff+3,1);
	    memcpy(&sorted_buff[0],buff+4,1);
	    memcpy(&cur_v,sorted_buff,2);
	    //printf("\nhex: %.4x\n",cur_v);
	    //printf("dec: %d\n",cur_v);

	    // Calculate stride 
	    int temp_stride = cur_v - pre_v;
	    if(abs(temp_stride) > MAX_STRIDE && temp_stride > 0)
	        stride = cur_v - 65535 - pre_v;
	    else if (abs(temp_stride) > MAX_STRIDE && temp_stride < 0)
	        stride = cur_v + 65535 - pre_v;
	    else
	        stride = temp_stride;
	    // Update position
	    int temp = get_encoder_position();
	    temp += stride;
	    update_encoder_position(temp);
	    //fprintf(stderr, "INF: actual position: %.10d\n",temp);

	    if(get_max_left_position() >= temp) {
		param temp_x = get_g_x();
		temp_x.cmd = GPST_CANCEL;
		update_g_x(temp_x);
	    }
	    if(get_max_right_position() <= temp) {
		param temp_x = get_g_x();
		temp_x.cmd = GPST_CANCEL;
		update_g_x(temp_x);
	    }

	    // update pre_v
	    pre_v = cur_v;
	}
	usleep(1000); // uint: 1us
    } 
 
    return;
}

// Listening UART signal
int listening_uart(const char* device, int baud, char parity, int data_bit, int stop_bit)
{
    pthread_t receiveid;

    fd = open(device, O_RDWR);
    if (fd < 0){
	log_e("listening_uart: open device failed.");
	return -1;
    }
    set_speed(fd,baud);
    set_Parity(fd,data_bit,stop_bit,parity);

    pthread_create(&receiveid,NULL,(void*)receivethread,NULL);
    pthread_detach(receiveid);
    return 0;
}
void close_uart()
{
    close(fd);
}
