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

#include    "am335x_setting.h"


#define FALSE 0
#define TRUE 1


// 用于判定是否收到编码器的响应
volatile int encoder_is_enable_ = FALSE;

// 编码器的运动
volatile Direction encoder_movement = NOTMOVE;
// 编码器上一时刻的位置,用于计算运动
volatile int prev_encoder_position;
// 编码器的当前位置
volatile int encoder_position;

// 用于编码器返回值保存
unsigned short  cur_v;
// 用于变量赋值的互斥
pthread_mutex_t mutex_encoder = PTHREAD_MUTEX_INITIALIZER;


// 更新编码器的位置
void update_encoder_position(int position)
{
    pthread_mutex_lock(&mutex_encoder);
    encoder_position = position;
    pthread_mutex_unlock(&mutex_encoder);
}
// 更新编码器的运动状态
void update_encoder_movement()
{
    pthread_mutex_lock(&mutex_encoder);
    int stride = encoder_position - prev_encoder_position;
    if(stride > 0)
	encoder_movement = RIGHTMOVE;
    else if(stride < 0)
	encoder_movement = LEFTMOVE;
    else encoder_movement = NOTMOVE;
    pthread_mutex_unlock(&mutex_encoder);
}
// 更新编码器的响应标志
void encoder_is_enable()
{
    pthread_mutex_lock(&mutex_encoder);
    encoder_is_enable_ = TRUE;
    pthread_mutex_unlock(&mutex_encoder);
}

// 获取当前编码器的角度
double get_encoder_angle()
{
    //pthread_mutex_lock(&mutex_encoder);
    int position = encoder_position;
    //pthread_mutex_unlock(&mutex_encoder);
    double angle = (double)position * 360.0 / (double)E_PULSE_PER_CIRCLE;
    return angle;
}
// 获取当前编码器的运动
Direction get_encoder_movement()
{
    pthread_mutex_lock(&mutex_encoder);
    Direction direct = encoder_movement;
    pthread_mutex_unlock(&mutex_encoder);
    return direct;
}
// 判断编码器是否已经工作
int is_encoder_enable()
{
    return encoder_is_enable_;
}


unsigned char CRC_check(unsigned char *buf, int n)
{
    unsigned char sum = 0x80;
    for(int i = 0; i < n; i++)
    {
	sum += buf[i];
    }
    return sum;
}

// 用于对编码器的返回字符串解析，返回编码器的值
// Check and parse the encoder data frame
// 返回值 -1 表示没有解析到编码器的值
int recv_pst_data(char *str,int len )
{

    int startpos=-1;

    for(int i=0;i<len-3;i++ )
    {
	if( *(str) == 0xff &&
	    *(str+1) == 0x81 &&
	    *(str+2) == 0x00 )
	{
	    startpos=i;

	    break;
	}
    
    }
    if(-1 == startpos) return -1;

    if( startpos + 7 <= len )
    {
	unsigned char act_chk;
	memcpy(&act_chk,(unsigned char*)str+startpos+5,1);
	unsigned char chk = CRC_check((unsigned char*)str+startpos+3,2);
	if(chk == act_chk) {
	    char sorted_buff[2];
	    unsigned short ret;
	    memcpy(&sorted_buff[1],str+startpos+3,1);
	    memcpy(&sorted_buff[0],str+startpos+4,1);
	    memcpy(&ret,sorted_buff,2);
	    return ret;
	}
    }

    //char *ptr = strstr(str,"\xff\x81\x00");
    ////char *ptr_end = strchr(ptr+3,'\0');
    //if(NULL != ptr && NULL != ptr_end && (ptr_end - ptr >= 6)) {
    //    unsigned char act_chk;
    //    memcpy(&act_chk,ptr+5,1);
    //    unsigned char chk = CRC_check((unsigned char*)ptr+3,2);
    //    if(chk == act_chk) {
    //        char sorted_buff[2];
    //        unsigned short ret;
    //        memcpy(&sorted_buff[1],ptr+3,1);
    //        memcpy(&sorted_buff[0],ptr+4,1);
    //        memcpy(&ret,sorted_buff,2);
    //        return ret;
    //    }
    //}

    return -1;
}

//*************************************************************************************************************************************
int fd=-1;
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

// 编码器监听线程
//*************************************************************************************************************************************
void receivethread(void)
{
    int nread;
    char buff[128];

    printf("aa....\n");

    // 监听
    while(1) 
    {
	usleep(10);

	//printf("bb...\n");

	//if((nread = read(fd,buff,120))>3) 
	if((nread = read(fd,buff,120))>0) 
	{
	    buff[nread]='\0';

	    //printf("[RECEIVE LEN] is %d, content is:\n",nread);
	    //for(int i = 0; i < nread; i++){
	    //	fprintf(stderr,"%.2x ",buff[i]);
	    //}

	    int val = recv_pst_data(buff,nread);
	    if(-1 == val) continue;
	    cur_v = val;

	    //printf("cc...\n");

	    //printf("am335x: current position %d\n", cur_v);
	    //double test_angle = get_encoder_angle();
	    //printf("am35x: get_encoder_angle %f\n", test_angle);

	    // Calculate position
	    int position;
	    int temp_stride = cur_v - ENCODER_ZERO_POSITION;
	    if(abs(temp_stride) > MAX_STRIDE && temp_stride > 0)
	        position = cur_v - 65535 - ENCODER_ZERO_POSITION;
	    else if (abs(temp_stride) > MAX_STRIDE && temp_stride < 0)
	        position = cur_v + 65535 - ENCODER_ZERO_POSITION;
	    else
	        position = temp_stride;
	    // 使其和电机的运动方向一致
	    position *= -1;

	    // 判断编码器是否已经工作
	    if(!is_encoder_enable()) {
		update_encoder_position(position);
		prev_encoder_position = position;
		update_encoder_movement();
		// 设定启动点角度
		double angle = get_encoder_angle();
		set_g_start_angle(angle);
		encoder_is_enable();
	    }

	    // Update position
	    update_encoder_position(position);
	    update_encoder_movement();
	    prev_encoder_position = position;


	    ////printf("\nhex: %.4x\n",cur_v);
	    //printf("am335x: dec %d\n",cur_v);
	    //int temp = get_encoder_position();
	    //printf("am335x: actual position: %.10d\n",temp);
	    //fflush(stdout);

	    // TODO(wangzhiheng): 限位条件判断
	    double angle = get_encoder_angle();
	    Direction movement = get_encoder_movement();

	    if( ((angle > get_g_right_angle()) || (angle > 90)) && (RIGHTMOVE == movement) ) {
		//param temp_x;
		//temp_x.cmd = GPST_CANCEL;
		//update_g_x(temp_x);
	    }else if( ((angle < get_g_left_angle()) || (angle < 90)) && (LEFTMOVE == movement) ) {
		//param temp_x;
		//temp_x.cmd = GPST_CANCEL;
		//update_g_x(temp_x);
	    }
	}
	
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

    if(0 != pthread_create(&receiveid,NULL,(void*)receivethread,NULL)) return -1;
    if(0 != pthread_detach(receiveid)) return -1;
    return 0;
}
void close_uart()
{
    close(fd);
}
