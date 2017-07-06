#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <math.h>

#include "elog.h"
#include "iniparser.h"

#include "global_setting.h"

#include "alpha_setting.h"
#include "am335x_setting.h"
#include "cmdparser.h"
#include "high_level_control.h"


// 状态发送开/关
volatile bool g_status = true;
// 释放伺服开/关
volatile bool g_stop = false;

// 变量赋值时的互斥
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


// 设置状态发送开关
void set_status(bool mode)
{
    pthread_mutex_lock(&mutex);
    g_status = mode;
    pthread_mutex_unlock(&mutex);
}
bool get_status()
{
    pthread_mutex_lock(&mutex);
    bool mode = g_status;
    pthread_mutex_unlock(&mutex);
    return mode;
}
// 设置释放伺服开关
void set_stop(bool mode)
{
    pthread_mutex_lock(&mutex);
    g_stop = mode;
    pthread_mutex_unlock(&mutex);
}
bool get_stop()
{
    pthread_mutex_lock(&mutex);
    bool mode = g_stop;
    pthread_mutex_unlock(&mutex);
    return mode;
}

// 保存Socket参数的对象
am335x_socket_t g_am335x_socket;
// 保存modbus参数的对象
rtu_master_t g_rtu_master;
// 保存串口参数的对象
uart_t g_am335x_uart;

// 配置表创建与解析函数
void create_example_ini_file(void);
int  parse_ini_file(char* ini_name);

// 对全局控制信号进行监听的函数
void signal_handler(void);


int main(int argc, char** argv)
{
    int ret;

    // EasyLogger Log配置
    elog_init();
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);
    // Start EasyLogger
    elog_start();

    // 读配置表
    ret = parse_ini_file("configure.ini");
    if(-1 == ret)
    {
        create_example_ini_file();
        ret = parse_ini_file("configure.ini");
        if(-1 == ret)
        {
            log_e("Init from configure.ini failed.");
            return -1;
        }
    }
    log_i("Init from configure.ini success.");

    // 初始化modbus通信使用的内存缓冲区
    ret = init_buffers_for_modbus();
    if(-1 == ret){
        log_e("Init buffers for modbus communication failed.");
        return -1;
    }
    // 打开modbus控制终端("/dev/ttyO1", 38400, 'E', 8, 1, 1);
    ret = open_modbus_rtu_master(g_rtu_master.device, g_rtu_master.baud, \
            g_rtu_master.parity, g_rtu_master.data_bit, g_rtu_master.stop_bit, g_rtu_master.slave);
    if(-1 == ret){
    	log_e("Open modbus_rtu_master failed.");
    	free_buffers_for_modbus();
    	return -1;
    }

    // 初始化伺服电机控制器的寄存器配置，请慎重！(Warn: parameter setting.)
    if(1 == g_rtu_master.reset_parameter){
        init_parameters();
        log_w("Warn: parameter setting. Dangerous!!!");
    }
    // 检验核实伺服电机控制器的寄存器配置
    if(-1 == check_parameters()){
        log_e("check parameters failed.");
        close_modbus_rtu_master();
        free_buffers_for_modbus();
        return -1;
    }

    // 串口监听，监听编码器信息("/dev/ttyO2", 9600, 'N', 8, 1);
    ret = listening_uart(g_am335x_uart.device, g_am335x_uart.baud, \
            g_am335x_uart.parity, g_am335x_uart.data_bit, g_am335x_uart.stop_bit);
    if(-1 == ret){
    	log_e("Open am335x uart failed.");
    	close_modbus_rtu_master();
    	free_buffers_for_modbus();
    	return -1;
    }
    // 监听套接字，解析终端的控制命令
    ret = listening_socket(g_am335x_socket.server_port, g_am335x_socket.queue_size);
    if(-1 == ret){
        log_e("Open am335x ethernet port failed.");
        close_modbus_rtu_master();
        free_buffers_for_modbus();
        close_uart();
        return -1;
    }

    // 电机进入伺服
    ret = serve_on();
    if(-1 == ret){
        log_e("servo on failed.");
    	close_modbus_rtu_master();
    	free_buffers_for_modbus();
        close_uart();
    	return -1;
    }
    // 确认进入就绪态
    ret = is_ready();
    if(ret != 1){
    	log_e("servo_on is ON, but is_ready OFF.");
    	serve_off();
    	close_modbus_rtu_master();
    	free_buffers_for_modbus();
        close_uart();
    	return -1;
    }
    /// 发送运行速度、加速时间、减速时间到控制器
    ret = send_cruise_speed();
    if(-1 == ret) set_stop(true);
    ret = send_imme_acceleration_time();
    if(-1 == ret) set_stop(true);
    ret = send_imme_deceleration_time();
    if(-1 == ret) set_stop(true);

    //// TODO(wangzhiheng): 为了获得启动时的实际角度，进行零点矫正
    /// 等待编码器响应状态为enable,即编码器有数据
    int waittime = 300; // 5分钟
    for(int i=0; i < waittime; ++i)
    {
        if(!is_encoder_enable()){
            if(i == waittime-1){
                set_stop(true);
                break;
            }
            usleep(1000000); // 1s
            continue;
        }else
            break;
    }

    /// 添加全局控制信号检测函数到线程中去
    pthread_t signalthreadid;
    if(pthread_create(&signalthreadid,NULL,(void*)signal_handler,NULL) != 0) {
        log_e("create signal handle thread failed.");
        set_stop(true);
    }
    if(pthread_detach(signalthreadid) != 0) {
        log_e("detach signal handle thread failed.");
        set_stop(true);
    }
    // 初始化完成！
    log_i("Init Success.");

    //// 初始化完成后的自检操作
    if(!get_stop())
    {
        param temp;
        temp.cmd = GCHECK;
        update_g_x(temp);
        update_g_ctrl_status(LEFTORRIGHT);
    }

    //// 循环获取编码器的位置，并发送到控制终端
    while(!get_stop()) {
        
        CtrlStatus ctrl_status = get_g_ctrl_status();
        double angle = get_encoder_angle();
        double diff_angle = angle - get_destination_angle();

        //printf("aa...in mian: diff angle %f, status %d, fabs %f\n", diff_angle, ctrl_status, fabs(diff_angle));

        if( (LOCATING == ctrl_status) &&
            (fabs(diff_angle) <= 0.005) )
        {
            //printf("bb... in main\n");

            int ret = task_cancel();
            if(-1 == ret) update_g_ctrl_status(ERROOR);
        }

        // 获取角度信息
        if(get_anticlockwise()) angle *= -1;
        // 获取控制状态
        CtrlStatus status = get_g_ctrl_status();
        // 生成发送字符串
        char buf[64];
        memset(buf,0,64);
        sprintf(buf,"$A%.3f,%1d\r\n",angle,status);
        m_socket_write(buf,strlen(buf));


        usleep(1000); // 1ms
    }

    // 释放伺服，并退出程序
    serve_off();
    close_modbus_rtu_master();
    free_buffers_for_modbus();
    close_uart();

    log_i("Exit Success.");
    elog_close();
    return 0;
}


void signal_handler(void)
{
    int ret;

    while(1){

        // 获取当前指令并清空
        param temp = get_g_x();
        param temp_update;
        temp_update.cmd = 0;
        update_g_x(temp_update);
        
        if ( temp.cmd & GPST_CANCEL )    // 运动取消
        {
            //printf("signal_handler: cancel\n");

            ret = task_cancel();
            if(-1 == ret) update_g_ctrl_status(ERROOR);
        }
        else if ( temp.cmd & GEMG )      // 停止，并释放伺服
        {
            //printf("signal_handler: stop\n");

            ret = force_stop();
            if(-1 == ret) update_g_ctrl_status(ERROOR);

            set_stop(true);
        }
        else if ( temp.cmd & GPOINT )    // run to point 
        {
            //printf("signal_handler: point %f\n", temp.v[0]);
            
            ret = goto_point(temp.v[0]);
            if(-1 == ret) update_g_ctrl_status(ERROOR);
        }
        else if ( temp.cmd & GLEFT )     // run to left
        {
            //printf("signal_handler: left\n");

            ret = goto_left();
            if(-1 == ret) update_g_ctrl_status(ERROOR);
        }
        else if ( temp.cmd & GRIGHT )    // run to right 
        {
            //printf("signal_handler: right\n");

            ret = goto_right();
            if(-1 == ret) update_g_ctrl_status(ERROOR);
        }
        else if ( temp.cmd & GSPEED ) 
        {
            //printf("signal_handler: speed\n");

            // degree/s
            ret = set_speed_value(temp.v[0]);
            if (-1 == ret) update_g_ctrl_status(ERROOR);
        }
        else if ( temp.cmd & GACCE_TIME ) 
        {
            //printf("signal_handler: accetime\n");

            // 1 means 0.1 ms
            ret = set_acce_value(temp.v[0]);
            if (-1 == ret) update_g_ctrl_status(ERROOR);
        }
        else if ( temp.cmd & GDECE_TIME ) 
        {
            //printf("signal_handler: decetime\n");

            // 1 means 0.1 ms
            ret = set_dece_value(temp.v[0]);
            if (-1 == ret) update_g_ctrl_status(ERROOR);
        }
        else if ( temp.cmd & GMAX_POINT )
        {
            // degree
            double left_angle = temp.v[0];
            double right_angle = temp.v[0];
            set_g_left_angle(left_angle);
            set_g_right_angle(right_angle);
        }
        else if ( temp.cmd & GSTATUS )
        {
            bool mode = ((int)temp.v[0]) ? true : false;
            set_status(mode);
        }
        else if ( temp.cmd & GCHECK )
        {
            //printf("signal_handler: check\n");
            
            ret = check_motion();
            if(-1 == ret) update_g_ctrl_status(ERROOR);
        }


        // 更新控制状态
        CtrlStatus status = get_g_ctrl_status();
        if(!is_INP()){
            if(GPOINT & temp.cmd) 
                update_g_ctrl_status(LOCATING);
            else 
                update_g_ctrl_status(LEFTORRIGHT);
            if(ERROOR == status){
                update_g_ctrl_status(ERROOR);
                //取消任务
                task_cancel();
            }
        }
        else if(ERROOR == status)
        {
            update_g_ctrl_status(ERROOR);
        }
        else if((LOCATING == status)||(LEFTORRIGHT == status))
        {
            update_g_ctrl_status(FINISH);
        }
        else{
            update_g_ctrl_status(FREE);
        }

        if(get_status()||(FINISH == status)) 
        {
            // 若为finish,发送字符串到控制终端
            char buf[64];
            memset(buf,0,64);
            sprintf(buf,"$B\r\n");
            m_socket_write(buf,strlen(buf));
        }

        usleep(200000);     // 200ms
    }
}

void create_example_ini_file(void)
{
    FILE* ini;
    if ((ini=fopen("configure.ini", "w"))==NULL) {
        log_e("Create *.ini failed.");
        return ;
    }

    // Write configure into *.ini
    fprintf(ini,

    "[IP]"                                  "\n"
    "RemoteIP = 192.168.0.15"               "\n"
    "RemoteName = root"                     "\n\n"

    "[Motion Control]"                      "\n"
    "anticlockwise = 0"                     "\n"
    "max_left_position = -90"               "\n"    // degree
    "max_right_position = 90"               "\n"    // degree
    "speed = 4"                             "\n"    // degree/s
    "imme_acceleration_time = 10000"        "\n"    // 0.1ms    : 10000 means 1s
    "imme_deceleration_time = 10000"        "\n"    // 0.1ms
    "check_speed = 4"                       "\n"    // degree/s 
    "check_acce_time = 20000"               "\n"    // 0.1ms    : 20000 means 2s
    "check_dece_time = 20000"               "\n\n"  // 0.1ms

    "[RTU MASTER]"                          "\n"
    "device = /dev/ttyO1"                   "\n"
    "baud = 38400"                          "\n"
    "parity = E"                            "\n"
    "data_bit = 8"                          "\n"
    "stop_bit = 1"                          "\n"
    "slave = 1"                             "\n"
    "RESET = 0"                             "\n\n"

    "[UART]"                                "\n"
    "device = /dev/ttyO2"                   "\n"
    "baud = 9600"                           "\n"
    "parity = N"                            "\n"
    "data_bit = 8"                          "\n"
    "stop_bit = 1"                          "\n\n"

    "[SOCKET]"                              "\n"
    "server_port = 12345"                   "\n"
    "queue_size = 1"                        "\n"
    "mode = TCP"                            "\n\n"
    
    );

    fclose(ini);
    log_i("Create configure.ini success.");
}

int parse_ini_file(char * ini_name)
{
    dictionary  *   ini ;

    // Temporary variables
    double max_left_position;
    double max_right_position;
    uint32_t speed;
    uint32_t imme_acceleration_time;
    uint32_t imme_deceleration_time;
    uint32_t check_speed;
    uint32_t check_acce_time;
    uint32_t check_dece_time;

    ini = iniparser_load(ini_name);
    if (ini==NULL) {
        log_e("Load configure.ini failed.");
        return -1 ;
    }
    iniparser_dump(ini, stderr);

    // Get configure
    int anticlock = iniparser_getint(ini, "Motion Control:anticlockwise", 0);
    set_anticlockwise(anticlock);

    double temp_angle = iniparser_getdouble(ini, "Motion Control:max_left_position", 0);
    max_left_position = temp_angle;
    temp_angle = iniparser_getdouble(ini, "Motion Control:max_right_position", 0);
    max_right_position = temp_angle;

    double temp_speed = iniparser_getdouble(ini, "Motion Control:speed", 0);
    speed = temp_speed*60*100/360*TRANSMISSION_RATIO;
    temp_speed = iniparser_getdouble(ini, "Motion Control:check_speed", 0);
    check_speed = temp_speed*60*100/360*TRANSMISSION_RATIO;

    imme_acceleration_time = iniparser_getint(ini, "Motion Control:imme_acceleration_time", 0);
    imme_deceleration_time = iniparser_getint(ini, "Motion Control:imme_deceleration_time", 0);
    check_acce_time = iniparser_getint(ini, "Motion Control:check_acce_time", 0);
    check_dece_time = iniparser_getint(ini, "Motion Control:check_dece_time", 0);

    // Port 
    const char* ptr;
    ptr = iniparser_getstring(ini, "RTU MASTER:device", "/dev/ttyO1");
    memcpy(g_rtu_master.device, ptr, strlen(ptr));
    g_rtu_master.device[strlen(ptr)] = '\0';

    ptr = iniparser_getstring(ini, "RTU MASTER:parity", "E");
    memcpy(&g_rtu_master.parity, ptr, 1);
    g_rtu_master.baud = iniparser_getint(ini, "RTU MASTER:baud", 38400);
    g_rtu_master.data_bit = iniparser_getint(ini, "RTU MASTER:data_bit", 8);
    g_rtu_master.stop_bit = iniparser_getint(ini, "RTU MASTER:stop_bit", 1);
    g_rtu_master.slave = iniparser_getint(ini, "RTU MASTER:slave", 1);
    g_rtu_master.reset_parameter = iniparser_getint(ini, "RTU MASTER:RESET", 0);

    ptr = iniparser_getstring(ini, "UART:device", "/dev/ttyO2");
    memcpy(g_am335x_uart.device, ptr, strlen(ptr));
    g_am335x_uart.device[strlen(ptr)] = '\0';

    ptr = iniparser_getstring(ini, "UART:parity", "N");
    memcpy(&g_am335x_uart.parity, ptr, 1);
    g_am335x_uart.baud = iniparser_getint(ini, "UART:baud", 9600);
    g_am335x_uart.data_bit = iniparser_getint(ini, "UART:data_bit", 8);
    g_am335x_uart.stop_bit = iniparser_getint(ini, "UART:stop_bit", 1);

    g_am335x_socket.server_port = iniparser_getint(ini, "SOCKET:server_port", 12345);
    g_am335x_socket.queue_size = iniparser_getint(ini, "SOCKET:queue_size", 1);

    // Set control parameter
    set_g_left_angle(max_left_position);
    set_g_right_angle(max_right_position);

    set_cruise_speed(speed);
    set_imme_acceleration_time(imme_acceleration_time);
    set_imme_deceleration_time(imme_deceleration_time);
    set_check_speed(check_speed);
    set_check_acce_time(check_acce_time);
    set_check_dece_time(check_dece_time);
   
    iniparser_freedict(ini);
    return 0 ;
}
