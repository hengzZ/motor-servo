#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
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
#include "am335x_setting.h"


// 记录执行的命令
//volatile GFLAGS last_flags = 0;
// 用于赋值时的互斥
pthread_mutex_t mutex_cmd = PTHREAD_MUTEX_INITIALIZER;
// Socket
int server_port, queue_size;

static int sock = -1;
static int conneted = 0;

struct sockaddr_in clientAddr;

// Socket收发
int m_socket_read(void *buf, size_t count)
{
    pthread_mutex_lock(&mutex_cmd);
    
    int len = sizeof(clientAddr);
    
    int bytes = recvfrom(sock, buf, count, 0, (struct sockaddr*)&clientAddr, &len);
    if(bytes>0) conneted = 1;
    
    
    pthread_mutex_unlock(&mutex_cmd);
    
    return bytes;
}
int m_socket_write(const void *buf, size_t count)
{
    int bytes;
    pthread_mutex_lock(&mutex_cmd);
    
    if(conneted){
	    bytes = sendto(sock, buf, count, 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
    }else{
	    bytes = 0;
    }
    pthread_mutex_unlock(&mutex_cmd);
    
    return bytes;
}
// 信息发送
int message_send(const char* msg)
{
    //char sendbuf[64];
    //memset(sendbuf,0,64);
    //sprintf(sendbuf, msg);
    return m_socket_write(msg,strlen(msg));
}


static void run_cmd(param *temp_x,  CtrlStatus  curstatus )
{
    int ret = 5;
    double left_angle;
    double right_angle;
    double cur_angle;
    
    switch( temp_x->cmd ){
    
        case GPST_CANCEL:
            ret = task_cancel();
            update_g_ctrl_status(STOPING);
            break;                    
        case GEMG :
            ret = force_stop();			   
            update_g_ctrl_status(STOPING);
            set_stop(true); // 释放伺服，退出程序
            break;
        case GSPEED:
            ret = set_speed_value(temp_x->v[0]);
            break;
        case GACCE_TIME :
            ret = set_acce_value(temp_x->v[0]);
            break;
        case GDECE_TIME  :
            ret = set_dece_value(temp_x->v[0]);
            break;                    
       case GMAX_POINT:               		
            // degreea			        
            //??????????? same value
            left_angle = temp_x->v[0];
            right_angle = temp_x->v[1];
            set_g_left_angle(left_angle);
            set_g_right_angle(right_angle);
            ret = 0;
            break;
        case   GCHECK:			
            //printf("signal_handler: check\n");
            ret = check_motion();
            break;
        case   GERROR_MSG:			
            ret = send_error_msg();
            break;
        case   GALARM_RST:
            ret = alarm_reset();
            break;

        case    GPOINT:			
            //printf("signal_handler: point %f\n", temp.v[0]);
            if(  FREE != curstatus ){						
                message_send("$D\r\n"); // 未执行
            }
            else{
                
                 cur_angle = get_encoder_angle();
                
                 if( fabs(cur_angle - temp_x->v[0] ) > 0.005) {
                    ret = goto_point(temp_x->v[0]);
                    update_g_ctrl_status(LOCATING);
                    //message_send("$C\r\n");	
                 }else{
                    message_send("$D\r\n"); // 未执行
                 }		                
                
            }
            
            break;
        case GLEFT:			
            //printf("signal_handler: left\n");
            if(  FREE != curstatus ){						
                message_send("$D\r\n"); // 未执行
            }
            else{
                ret = goto_left();
                update_g_ctrl_status(LEFTORRIGHT);
                //message_send("$C\r\n");	
            }
            break;
        case  GRIGHT: 
	        //printf("signal_handler: right\n");
	        if(  FREE != curstatus ){						
                message_send("$D\r\n"); // 未执行
            }
            else{
                ret = goto_right();
                update_g_ctrl_status(LEFTORRIGHT);
                //message_send("$C\r\n");	
            }
            break;
        default:
            break;
                               
    
    }
    
    


}

// 套接字监听
//***************************************************************
void parsesocket(void)
{

    if( -1 == sock ){
    
	    // udp addr
	    struct sockaddr_in addr;    // holds UDP IP address
	    memset(&addr, 0, sizeof(addr));
	    addr.sin_family = AF_INET;
	    addr.sin_addr.s_addr = htonl(INADDR_ANY);
	    addr.sin_port = htons(server_port);

	    // socket
	    sock = socket(AF_INET, SOCK_DGRAM, 0);
	    if(sock < 0){
	        log_e("socket");
	        exit(-1);
	    }

	    int flags = fcntl(sock, F_GETFL, 0); // 获取当前状态
	    fcntl(sock, F_SETFL, flags|O_NONBLOCK); // 非阻塞设置

	    // bind
	    if(bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0){
	        perror("bind");
	        exit(-1);
	    }
    }
	// Init Done
	int bytes = 0;
	char buf[128];

	CtrlStatus  curstatus;
	param temp_x; //控制参数对象
	// 监听: 信号解析与执行
	while(1)
	{
		//connect_loop = 1;
		//while(connect_loop)
		//{
		//connect_loop = 0;
	    bytes = m_socket_read(buf,127);	
	    if(bytes > 0) {  
	    
	        //important
	        buf[bytes]=0;
	        
	        //// TODO 指令解析 ////	       
	        temp_x.cmd = 0;
	        if(buf == strstr(buf,"cancel")){
	            //printf("cmdparse: cancel");
	            temp_x.cmd = GPST_CANCEL;
	        }
	        else if(buf == strstr(buf,"stop")){
	            //printf("cmdparse: stop");
	            temp_x.cmd = GEMG;
	        }
	        else if(buf == strstr(buf, "point")){
	            double position;
	            sscanf(buf,"point %lf",&position);

	            // Debug语句
	            char tmp_buf[1024];
	            sprintf(tmp_buf, buf);
	            log_e(tmp_buf);
	            printf("cmdparse: %s\n",tmp_buf);
	            sprintf(tmp_buf, "cmdparse: point %lf",position);
	            log_e(tmp_buf);
	            printf("cmdparse: %s\n",tmp_buf);

	            if(get_anticlockwise()) 
	                 temp_x.v[0] = (-position);
	            else temp_x.v[0] = position;
	            temp_x.cmd = GPOINT;
	        }
	        else if(buf == strstr(buf,"runleft")){
	
	            if(get_anticlockwise()) temp_x.cmd = GRIGHT;
	            else temp_x.cmd = GLEFT;
	        }
	        else if(buf == strstr(buf,"runright")){
	
	            if(get_anticlockwise()) temp_x.cmd = GLEFT;
	            else temp_x.cmd = GRIGHT;
	        }
	        else if(buf == strstr(buf,"speed")){
	            double speed;
	            sscanf(buf,"speed %lf",&speed);
	            temp_x.v[0] = speed;
	            temp_x.cmd = GSPEED;
	        }
	        else if(buf == strstr(buf,"acce")){
	
	            double acce;
	            sscanf(buf, "acce %lf",&acce);
	            temp_x.v[0] = acce;
	            temp_x.cmd = GACCE_TIME;
	        }
	        else if(buf == strstr(buf,"dece")){
	
	            double dece;
	            sscanf(buf, "dece %lf",&dece);
	            temp_x.v[0] = dece;
	            temp_x.cmd = GDECE_TIME;
	        }
	        else if(buf == strstr(buf,"maxpoint")){
	
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
	        else if(buf == strstr(buf,"check")){
	
	            temp_x.cmd = GCHECK;
	        }
	        else if(buf == strstr(buf,"errormsg")){
	
	            temp_x.cmd = GERROR_MSG;
	        }
	        else if(buf == strstr(buf,"alarmrst")){
	
	            temp_x.cmd = GALARM_RST;
	        }
	        else
	        {
	           temp_x.cmd = 0;
	        }
	        curstatus = get_g_ctrl_status();
	        //if initiliazing then return;
	        if(  INITLIZING == curstatus || NONE == curstatus ){
	
	            message_send("$D\r\n"); // 未执行
	            //continue;		
	        }	
		else
		{
		        run_cmd(&temp_x,curstatus);	
		}
	    } //END IF(BYTES>0)
		
		//delay for cmd running
		usleep(100000);	    // 100ms
		
		curstatus = get_g_ctrl_status();
		if( is_INP() ){
			
		    // Debug语句
		    //printf("main: is inp\n");

		    if( LOCATING == curstatus ){							
		        message_send("$B\r\n"); // position到位			    
		    }else if ( INITLIZING == curstatus ){	
		    	    
		        log_i("Init Success.");		    
		    }
		     
		    update_g_ctrl_status(FREE);
		}
		
    		
		if( LOCATING == curstatus ){
		     //printf("bb... in main\n");
		     double curangle = get_encoder_angle();
		     double diff_angle = curangle - get_destination_angle(); 
		     if (fabs(diff_angle) <= 0.005){                          
			 int r = task_cancel();
			 //if(-1 == r) update_g_ctrl_status(ERROOR);
		     }
		 }  

		// 报警检测
		if( 0 == get_out_status(ALRM_DETC_B_ad) ){			
		    update_g_ctrl_status(ERROOR);
		}
		// 伺服电机零速度判断
		if( 1 == get_out_status(ZRO_SPEED_ad) ){			
		    set_motor_zero_speed();
		}			
		
		//delay for modbus reading
		usleep(100000);	    // 100ms
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
