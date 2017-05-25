#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>

#include "elog.h"
#include "iniparser.h"

#include "alpha_setting.h"
#include "alpha_motion_control.h"
#include "am335x_setting.h"
#include "cmdparser.h"


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
    GCHECK=1024
} GFLAGS;

typedef struct {
    GFLAGS cmd;
    int32_t v[2];
}param;

// DataObject for control and data transmission
volatile bool g_status = false;
volatile bool g_stop = false;
volatile param g_x;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void update_g_x(param x)
{
    pthread_mutex_lock(&mutex);
    g_x = x;
    pthread_mutex_unlock(&mutex);
}
param get_g_x()
{
    pthread_mutex_lock(&mutex);
    param temp_x = g_x;
    pthread_mutex_unlock(&mutex);
    return temp_x;
}
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

// DataObject for SOCKET listening
am335x_socket_t g_am335x_socket;
// DataObject for am335x's modbus-TRU listening
rtu_master_t g_rtu_master;
// DataObject for am335x's UART listening
uart_t g_am335x_uart;

// *.ini file related
void create_example_ini_file(void);
int  parse_ini_file(char* ini_name);

// Deal with control signal
void signal_handler(void);


int main(int argc, char** argv)
{
    int ret;

    // Init EasyLogger
    elog_init();
    // Set log format
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);
    // Start EasyLogger
    elog_start();

    // Read configure for *.ini file
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

    // Init buffers for modbus communication
    ret = init_buffers_for_modbus();
    if(-1 == ret){
        log_e("Init buffers for modbus communication failed.");
        return -1;
    }
    // open_modbus_rtu_master("/dev/ttyO1", 38400, 'E', 8, 1, 1);
    ret = open_modbus_rtu_master(g_rtu_master.device, g_rtu_master.baud, \
            g_rtu_master.parity, g_rtu_master.data_bit, g_rtu_master.stop_bit, g_rtu_master.slave);
    if(-1 == ret){
    	log_e("Open modbus_rtu_master failed.");
    	free_buffers_for_modbus();
    	return -1;
    }

    // Init Parameter (Warn: parameter setting.)
    if(1 == g_rtu_master.reset_parameter){
        init_parameters();
        log_w("Warn: parameter setting. Dangerous!!!");
    }
    // Check parameters
    if(-1 == check_parameters()){
        log_e("check parameters failed.");
        close_modbus_rtu_master();
        free_buffers_for_modbus();
        return -1;
    }

    // listening_uart("/dev/ttyO2", 9600, 'N', 8, 1);
    ret = listening_uart(g_am335x_uart.device, g_am335x_uart.baud, \
            g_am335x_uart.parity, g_am335x_uart.data_bit, g_am335x_uart.stop_bit);
    if(-1 == ret){
    	log_e("Open am335x uart failed.");
    	close_modbus_rtu_master();
    	free_buffers_for_modbus();
    	return -1;
    }
    // listening socket for control command
    ret = listening_socket(g_am335x_socket.server_port, g_am335x_socket.queue_size);
    if(-1 == ret){
        log_e("Open am335x ethernet port failed.");
        close_modbus_rtu_master();
        free_buffers_for_modbus();
        close_uart();
        return -1;
    }

    // Servo on
    ret = serve_on();
    if(-1 == ret){
        log_e("servo on failed.");
    	close_modbus_rtu_master();
    	free_buffers_for_modbus();
        close_uart();
    	return -1;
    }
    ret = is_ready();
    if(ret != 1){
    	log_e("servo_on is ON, but is_ready OFF.");
    	serve_off();
    	close_modbus_rtu_master();
    	free_buffers_for_modbus();
        close_uart();
    	return -1;
    }
    /// Set default Speed, Acceleration time, Deceleration time
    ret = send_cruise_speed();
    if(-1 == ret) set_stop(true);
    ret = send_imme_acceleration_time();
    if(-1 == ret) set_stop(true);
    ret = send_imme_deceleration_time();
    if(-1 == ret) set_stop(true);

    // Create thread for control signal dealing
    pthread_t signalthreadid;
    if(pthread_create(&signalthreadid,NULL,(void*)signal_handler,NULL) != 0) {
        log_e("create signal handle thread failed.");
        set_stop(true);
    }
    if(pthread_detach(signalthreadid) != 0) {
        log_e("detach signal handle thread failed.");
        set_stop(true);
    }
    // Init Done
    log_i("Init Success.");

    // Check motion
    param temp = get_g_x();
    temp.cmd = GCHECK;
    update_g_x(temp);

    // Loop for getting degree
    int anticlock = get_anticlockwise();
    while(!get_stop()) {
            int temp = get_encoder_position();
            if(get_anticlockwise()) temp *= -1;
            char buf[64];
            memset(buf,0,64);
            sprintf(buf,"$%.3f\r",360.0*temp/65535);
	    //printf("main loop: actual position: %.10d\n",temp);
            //printf("main loop: actual degree %s\n",buf);
            //fflush(stdout);
            m_socket_write(buf,strlen(buf));

            usleep(1000); // 1ms
    }

    // close
    serve_off();
    close_modbus_rtu_master();
    free_buffers_for_modbus();
    close_uart();
    elog_close();

    return 0;
}


void signal_handler(void)
{
    int ret;

    while(1){

        param temp = get_g_x();
        if ( temp.cmd & GPST_CANCEL )    // cancel
        {
            ret = positioning_cancel_on();
            if(-1 == ret) m_socket_write("#EOPE\r",strlen("#EOPE\r"));
            ret = positioning_cancel_off();
            if(-1 == ret) m_socket_write("#EOPE\r",strlen("#EOPE\r"));
            temp.cmd = 0;
            update_g_x(temp);
        }
        else if ( temp.cmd & GEMG )      // stop
        {
            ret = forced_stop_on();
            if(-1 == ret) m_socket_write("#EOPE\r",strlen("#EOPE\r"));
            ret = forced_stop_off();
            if(-1 == ret) m_socket_write("#EOPE\r",strlen("#EOPE\r"));
            set_stop(true);
            temp.cmd = 0;
            update_g_x(temp);
        }
        else if ( temp.cmd & GPOINT )    // run to point 
        {
            // 360,000 means 360 degree
            int32_t m_position = (double)temp.v[0] / 360000 * M_PULSE_PER_CIRCLE * TRANSMISSION_RATIO;
            //printf("run to position %d\n",m_position);
            //fflush(stdout);
            set_point_position(m_position);
            ret = run_to_point();
            if(-1 == ret) m_socket_write("#EOPE\r",strlen("#EOPE\r"));
            else if(1 == ret) {
                temp.cmd &= ~GPOINT;
                update_g_x(temp);
            }
        }
        else if ( temp.cmd & GLEFT )     // run to left
        {
            ret = left_direction_run();
            if(-1 == ret) m_socket_write("#EOPE\r",strlen("#EOPE\r"));
            else if(1 == ret) {
                temp.cmd &= ~GLEFT;
                update_g_x(temp);
            }
        }
        else if ( temp.cmd & GRIGHT )    // run to right 
        {
            ret = right_direction_run();
            if(-1 == ret) m_socket_write("#EOPE\r",strlen("#EOPE\r"));
            else if(1 == ret) {
                temp.cmd &= ~GRIGHT;
                update_g_x(temp);
            }
        }
        else if ( temp.cmd & GSPEED ) 
        {
            // 360,000 means 360 degree/s
            uint32_t speed = (double)temp.v[0] / 360000 * 60 * 100;
            //printf("set speed %d\n", speed);
            set_cruise_speed(speed);
            if(is_INP()){
                ret = send_cruise_speed();
                if (-1 == ret) m_socket_write("#EOPE\r",strlen("#EOPE\r"));
                temp.cmd &= ~GSPEED;
                update_g_x(temp);
            }
        }
        else if ( temp.cmd & GACCE_TIME ) 
        {
            set_imme_acceleration_time(temp.v[0]);      // 1 means 0.1 ms
            if(is_INP()){
                ret = send_imme_acceleration_time();
                if (-1 == ret) m_socket_write("#EOPE\r",strlen("#EOPE\r"));
                temp.cmd &= ~GACCE_TIME;
                update_g_x(temp);
            }
        }
        else if ( temp.cmd & GDECE_TIME ) 
        {
            set_imme_deceleration_time(temp.v[0]);      // 1 means 0.1 ms
            if(is_INP()){
                ret = send_imme_deceleration_time();
                if (-1 == ret) m_socket_write("#EOPE\r",strlen("#EOPE\r"));
                temp.cmd &= ~GDECE_TIME;
                update_g_x(temp);
            }
        }
        else if ( temp.cmd & GMAX_POINT )
        {
            // 360,000 means 360 degree
            int32_t m_position_left = (double)temp.v[0] / 360000 * M_PULSE_PER_CIRCLE * TRANSMISSION_RATIO;
            int32_t m_position_right = (double)temp.v[1] / 360000 * M_PULSE_PER_CIRCLE * TRANSMISSION_RATIO;
            int32_t e_position_left = (double)temp.v[0] / 360000 * E_PULSE_PER_CIRCLE;
            int32_t e_position_right = (double)temp.v[1] / 360000 * E_PULSE_PER_CIRCLE;
            //printf("set motor maxpoint %.10d  %.10d\n",m_position_left,m_position_right);
            //printf("set encoder maxpoint %.10d  %.10d\n",e_position_left,e_position_right);
            set_limit_left_position(m_position_left);
            set_limit_right_position(m_position_right);
            set_max_left_position(e_position_left);
            set_max_right_position(e_position_right);
            temp.cmd &= ~GMAX_POINT;
            update_g_x(temp);
        }
        else if ( temp.cmd & GSTATUS )
        {
            bool mode = temp.v[0] ? true : false;
            set_status(mode);
            temp.cmd &= ~GSTATUS;
            update_g_x(temp);
        }
        else if ( temp.cmd & GCHECK )
        {
            temp.cmd &= ~GCHECK;
            update_g_x(temp);
            ret = check_motion();
            if(-1 == ret) m_socket_write("#EOPE\r",strlen("#EOPE\r"));
        }

        // Get status
        if(get_status()) 
        {
            char buf[64];
            int position = get_encoder_position();
            //setbuf(stdout,NULL);
            //printf("get status: actual encoder position: %.10d\n", position);
            //printf("%.3f\n",360.0*position/65535);
            //fflush(stdout);
            int max_left = get_max_left_position() + E_PULSE_OFFSET;
            int max_right = get_max_right_position() - E_PULSE_OFFSET;
            if(is_INP()) 
            {
                if(position <= max_left) {
                    const char *ptr = get_anticlockwise() ? "#MAXR\r" : "#MAXL\r";
                    memcpy(buf,ptr,strlen(ptr)+1);
                }
                else if(position >= max_right) {
                    const char *ptr = get_anticlockwise() ? "#MAXL\r" : "#MAXR\r";
                    memcpy(buf,ptr,strlen(ptr)+1);
                }else {
                    const char *ptr = "#INPO\r";
                    memcpy(buf,ptr,strlen(ptr)+1);
                }
                //printf("status: %s\n",buf);
                m_socket_write(buf,strlen(buf));
            }
            // Check error
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
    "cruise_left_position = -90"            "\n"    // degree
    "cruise_right_position = 90"            "\n"    // degree
    "direct_left_position = -90"            "\n"    // degree
    "direct_right_position = 90"            "\n"    // degree
    "max_left_position = -90"               "\n"    // degree
    "max_right_position = 90"               "\n"    // degree
    "speed = 600"                           "\n"    // degree/s : 3600 means 600r/min
    "imme_acceleration_time = 10000"        "\n"    // 0.1ms    : 10000 means 1s
    "imme_deceleration_time = 10000"        "\n"    // 0.1ms
    "check_speed = 600"                     "\n"    // degree/s : 12000 means 2000r/min
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
    int32_t cruise_left_position;
    int32_t cruise_right_position;
    int32_t direct_left_position;
    int32_t direct_right_position;
    int32_t max_left_position;
    int32_t max_right_position;
    int32_t limit_left_position;
    int32_t limit_right_position;
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

    double temp_position = iniparser_getdouble(ini, "Motion Control:cruise_left_position", 0);
    cruise_left_position = temp_position / 360 * M_PULSE_PER_CIRCLE * TRANSMISSION_RATIO;
    temp_position = iniparser_getdouble(ini, "Motion Control:cruise_right_position", 0);
    cruise_right_position = temp_position / 360 * M_PULSE_PER_CIRCLE * TRANSMISSION_RATIO;
    temp_position = iniparser_getdouble(ini, "Motion Control:direct_left_position", 0);
    direct_left_position = temp_position / 360 * M_PULSE_PER_CIRCLE * TRANSMISSION_RATIO;
    temp_position = iniparser_getdouble(ini, "Motion Control:direct_right_position", 0);
    direct_right_position = temp_position / 360 * M_PULSE_PER_CIRCLE * TRANSMISSION_RATIO;

    temp_position = iniparser_getdouble(ini, "Motion Control:max_left_position", 0);
    limit_left_position = temp_position / 360 * M_PULSE_PER_CIRCLE * TRANSMISSION_RATIO;
    max_left_position = temp_position / 360 * E_PULSE_PER_CIRCLE;

    temp_position = iniparser_getdouble(ini, "Motion Control:max_right_position", 0);
    limit_right_position = temp_position / 360 * M_PULSE_PER_CIRCLE * TRANSMISSION_RATIO;
    max_right_position = temp_position / 360 * E_PULSE_PER_CIRCLE;

    double temp_speed = iniparser_getdouble(ini, "Motion Control:speed", 0);
    speed = temp_speed / 360 * 60 * 100;
    temp_speed = iniparser_getdouble(ini, "Motion Control:check_speed", 0);
    check_speed = temp_speed / 360 * 60 * 100;

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
    set_limit_left_position(limit_left_position);
    set_limit_right_position(limit_right_position);
    set_max_left_position(max_left_position);
    set_max_right_position(max_right_position);
    set_cruise_left_position(cruise_left_position);
    set_cruise_right_position(cruise_right_position);
    set_direct_left_position(direct_left_position);
    set_direct_right_position(direct_right_position);
    set_cruise_speed(speed);
    set_imme_acceleration_time(imme_acceleration_time);
    set_imme_deceleration_time(imme_deceleration_time);
    set_check_speed(check_speed);
    set_check_acce_time(check_acce_time);
    set_check_dece_time(check_dece_time);
    set_point_position(0);
   
    //// Inf - zhihengw
    //printf("limit_left_position %d\n",limit_left_position);
    //printf("limit_right_position %d\n",limit_right_position);
    //printf("max_left_position %d\n",max_left_position);
    //printf("max_right_position %d\n",max_right_position);
    //printf("speed %d\n",speed);
    //printf("check_acce_time %d\n",check_acce_time);
    //printf("check_dece_time %d\n",check_dece_time);
    //printf("check_speed %d\n",check_speed);
    //printf("imme_acceleration_time %d\n",imme_acceleration_time);
    //printf("imme_deceleration_time %d\n",imme_deceleration_time);


    iniparser_freedict(ini);
    return 0 ;
}
