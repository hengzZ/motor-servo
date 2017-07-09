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
#include     <math.h>
#include     <time.h>

#include    "elog.h"
#include    "am335x_setting.h"
#include    "alpha_motion_control.h"


// 用于判定是否收到编码器的响应
volatile int encoder_is_enable_ = FALSE;

// 编码器的速度，脉冲/毫秒
volatile double encoder_speed=0;
// 编码器上一时刻的位置,用于计算运动
volatile int prev_encoder_position;
volatile clock_t t1;
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
// 更新编码器的速度
void update_encoder_speed(double speed)
{
    pthread_mutex_lock(&mutex_encoder);
    encoder_speed = speed;
    pthread_mutex_unlock(&mutex_encoder);
}
// 更新编码器的响应标志
void encoder_is_enable()
{
    pthread_mutex_lock(&mutex_encoder);
    encoder_is_enable_ = TRUE;
    pthread_mutex_unlock(&mutex_encoder);
}

// 判断编码器是否已经工作
int is_encoder_enable()
{
    return encoder_is_enable_;
}
// 获取当前编码器的角度
double get_encoder_angle()
{
    pthread_mutex_lock(&mutex_encoder);
    int position = encoder_position;
    pthread_mutex_unlock(&mutex_encoder);
    double angle = (double)position * 360.0 / (double)E_PULSE_PER_CIRCLE;
    return angle;
}
// 获取当前编码器的速度
double get_encoder_speed()
{
    pthread_mutex_lock(&mutex_encoder);
    double speed = encoder_speed;
    pthread_mutex_unlock(&mutex_encoder);
    return speed;
}
// 获取当前编码器的运动
MovementStatus get_encoder_movement()
{
    pthread_mutex_lock(&mutex_encoder);
    double speed = encoder_speed;
    pthread_mutex_unlock(&mutex_encoder);
    MovementStatus direct;
    if(speed > 0) direct = RIGHTMOVE;
    else if(speed < 0) direct = LEFTMOVE;
    else direct = NOTMOVE;
    return direct;
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

    //printf("aa....\n");

    // 监听
    while(1) 
    {
	usleep(1000);

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
		t1 = clock(); 
		prev_encoder_position = position;
		// 设定启动点角度
		double angle = get_encoder_angle();
		set_g_start_angle(angle);
		encoder_is_enable();
	    }

	    // Update position
	    update_encoder_position(position);
	    // 测速，200ms测一次速
	    clock_t t2 = clock();
	    double duration = 1000.0*(t2-t1)/CLOCKS_PER_SEC;
	    if(duration > 200)
	    {
		double speed = (position - prev_encoder_position)/duration;
		update_encoder_speed(speed);
		t1 = t2;
		prev_encoder_position = position;
	    }

	}

	// TODO(wangzhiheng): 限位条件判断,用于校对电机与编码器是否同步一致
	// 编码器速度很小的时候，控制器的零速度信号是否有输出
	// 电机50r/min,零速度信号就响应, 对应编码器为0.27个脉冲/ms
	double speed = get_encoder_speed();
	if(fabs(speed) < 0.15){
	    // 零速度信号判断
	    if(1 != get_motor_zero_speed())
	    {
		//停止
		serve_off();
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
