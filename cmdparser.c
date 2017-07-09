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
#include "global_setting.h"
#include "alpha_setting.h"
#include "high_level_control.h"


// 记录执行的命令
volatile GFLAGS last_flags = 0;
// 用于赋值时的互斥
pthread_mutex_t mutex_cmd = PTHREAD_MUTEX_INITIALIZER;

// Socket
int server_port, queue_size;
int s, b, l, sa;
int on = 1;
// Socket收发
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
// 信息发送
int message_send(const char* msg)
{
    char sendbuf[64];
    memset(sendbuf,0,64);

    sprintf(sendbuf, msg);
    return m_socket_write(sendbuf,strlen(sendbuf));
}


// 套接字监听
//***************************************************************
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
	int connect_loop;
	
	// 监听: 信号解析与执行
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
			//// TODO 指令解析 ////
			param temp_x; //控制参数对象
			temp_x.cmd = 0;
			if(buf == strstr(buf,"cancel"))
			{
			    //printf("cmdparse: cancel");
			    temp_x.cmd = GPST_CANCEL;
			}
			else if(buf == strstr(buf,"stop"))
			{
			    //printf("cmdparse: stop");
			    temp_x.cmd = GEMG;
			}
			else if(buf == strstr(buf, "point"))
			{
			    double position;
			    sscanf(buf,"point %lf",&position);

			    //char tmp_buf[1024];
			    //sprintf(tmp_buf, buf);
			    //log_e(tmp_buf);
			    //printf("cmdparse: %s\n",tmp_buf);
			    //sprintf(tmp_buf, "cmdparse: point %lf",position);
			    //log_e(tmp_buf);
			    //printf("cmdparse: %s\n",tmp_buf);

			    if(get_anticlockwise()) temp_x.v[0] = (-position);
			    else temp_x.v[0] = position;
			    temp_x.cmd = GPOINT;
			}
			else if(buf == strstr(buf,"runleft"))
			{
			    if(get_anticlockwise()) temp_x.cmd = GRIGHT;
			    else temp_x.cmd = GLEFT;
			}
			else if(buf == strstr(buf,"runright"))
			{
			    if(get_anticlockwise()) temp_x.cmd = GLEFT;
			    else temp_x.cmd = GRIGHT;
			}
			else if(buf == strstr(buf,"speed"))
			{
			    double speed;
			    sscanf(buf,"speed %lf",&speed);
			    temp_x.v[0] = speed;
			    temp_x.cmd = GSPEED;
			}
			else if(buf == strstr(buf,"acce"))
			{
			    double acce;
			    sscanf(buf, "acce %lf",&acce);
			    temp_x.v[0] = acce;
			    temp_x.cmd = GACCE_TIME;
			}
			else if(buf == strstr(buf,"dece"))
			{
			    double dece;
			    sscanf(buf, "dece %lf",&dece);
			    temp_x.v[0] = dece;
			    temp_x.cmd = GDECE_TIME;
			}
			else if(buf == strstr(buf,"maxpoint"))
			{
			    double max_left, max_right;
			    sscanf(buf, "maxpoint %lf %lf",&max_left, &max_right);
			    if(get_anticlockwise()) max_left *= -1, max_right *= -1;
			    if(max_left > max_right){
				double temp = max_right;
				max_right = max_left;
				max_left = temp;
			    }
			    temp_x.v[0] = max_left;
			    temp_x.v[1] = max_right;
			    temp_x.cmd = GMAX_POINT;
			}
			else if(buf == strstr(buf,"check"))
			{
			    temp_x.cmd = GCHECK;
			}
			else if(buf == strstr(buf,"errormsg"))
			{
			    temp_x.cmd = GERROR_MSG;
			}
			else if(buf == strstr(buf,"alarmrst"))
			{
			    temp_x.cmd = GALARM_RST;
			}

			//// TODO 指令执行 ////
			int ret = 5; // ret=5标志无指令
			if ( temp_x.cmd & GPST_CANCEL )    // 运动取消
			{
			    ret = task_cancel();
			}
			else if ( temp_x.cmd & GEMG )      // 停止，并释放伺服
			{
			    //printf("signal_handler: stop\n");
			    ret = force_stop();
			    // 释放伺服，退出程序
			    set_stop(true);
			}
			else if ( temp_x.cmd & GPOINT )    // run to point 
			{
			    //printf("signal_handler: point %f\n", temp.v[0]);
			    ret = goto_point(temp_x.v[0]);
			}
			else if ( temp_x.cmd & GLEFT )     // run to left
			{
			    //printf("signal_handler: left\n");
			    ret = goto_left();
			}
			else if ( temp_x.cmd & GRIGHT )    // run to right 
			{
			    //printf("signal_handler: right\n");
			    ret = goto_right();
			}
			else if ( temp_x.cmd & GSPEED ) 
			{
			    //printf("signal_handler: speed\n");
			    // degree/s
			    ret = set_speed_value(temp_x.v[0]);
			}
			else if ( temp_x.cmd & GACCE_TIME ) 
			{
			    //printf("signal_handler: accetime\n");
			    // 1 means 0.1 ms
			    ret = set_acce_value(temp_x.v[0]);
			}
			else if ( temp_x.cmd & GDECE_TIME ) 
			{
			    //printf("signal_handler: decetime\n");
			    // 1 means 0.1 ms
			    ret = set_dece_value(temp_x.v[0]);
			}
			else if ( temp_x.cmd & GMAX_POINT )
			{
			    // degree
			    double left_angle = temp_x.v[0];
			    double right_angle = temp_x.v[0];
			    set_g_left_angle(left_angle);
			    set_g_right_angle(right_angle);
			    ret = 0;
			}
			else if ( temp_x.cmd & GCHECK )
			{
			    //printf("signal_handler: check\n");
			    ret = check_motion();
			}
			else if ( temp_x.cmd & GERROR_MSG )
			{
			    ret = send_error_msg();
			}
			else if ( temp_x.cmd & GALARM_RST )
			{
			    ret = alarm_reset();
			}

			// 指令执行情况发送
			if(5==ret){
			    ; //无指令
			}else{
			    if(-1==ret) message_send("$D\r\n"); // 未执行
			    else {
				// 保存执行的指令
				last_flags = temp_x.cmd;
				message_send("$C\r\n");	// 指令被执行
			    }
			}

			// FINISH状态判断
			int is_inp = is_INP();
			if( (GPOINT == last_flags) && is_inp )
			{
			    message_send("$B\r\n"); // position到位
			    update_g_ctrl_status(FINISH);
			}
			else if( (GPOINT != last_flags) && is_inp )
			{
			    update_g_ctrl_status(FREE);
			}
			else if( (GPOINT == last_flags) && (!is_inp) )
			{
			    update_g_ctrl_status(LOCATING);
			}
			else if( (GPOINT != last_flags) && (!is_inp) )
			{
			    update_g_ctrl_status(LEFTORRIGHT);
			}

			// 报警检测
			if( 0 == get_out_status(ALRM_DETC_B_ad) )
			{
			    update_g_ctrl_status(ERROOR);
			}

			// 伺服电机零速度判断
			if( 1 == get_out_status(ZRO_SPEED_ad) )
			{
			    set_motor_zero_speed();
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
