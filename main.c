#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>

#include "elog.h"
#include "iniparser.h"

#include "cmdparser.h"
#include "am335x_setting.h"
#include "alpha_setting.h"
#include "alpha_motion_control.h"


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

// DataObject for control and data transmission
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

// Linux Timer for control query
static struct itimerval oldtv;
void set_timer();
void signal_handler(int m);

// DataObject for SOCKET listening
am335x_socket_t g_am335x_socket;
// DataObject for am335x's modbus-TRU listening
rtu_master_t g_rtu_master;
// DataObject for am335x's UART listening
uart_t g_am335x_uart;

// *.ini file related
void create_example_ini_file(void);
int  parse_ini_file(char* ini_name);

int stop_flag;

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
    ret = open_modbus_rtu_master(g_rtu_master.device, g_rtu_master.baud, g_rtu_master.parity, g_rtu_master.data_bit, g_rtu_master.stop_bit, g_rtu_master.slave);
    if(-1 == ret){
    	log_e("Open modbus_rtu_master failed.");
    	free_buffers_for_modbus();
    	return -1;
    }
    // Init Parameter (Warn: parameter setting.)
    if(1 == g_rtu_master.reset_parameter){
        init_parameters();
        log_i("Warn: parameter setting. Dangerous!!!");
    }
    // listening_uart("/dev/ttyO2", 9600, 'N', 8, 1);
    ret = listening_uart(g_am335x_uart.device, g_am335x_uart.baud, g_am335x_uart.parity, g_am335x_uart.data_bit, g_am335x_uart.stop_bit);
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
    // Init Done
    log_i("Init Success.");

    ///
    send_cruise_speed();
    send_imme_acceleration_time();
    send_imme_deceleration_time();
    //set_point_position(10100000);
    //run_to_point();

    //// Init timer for control query
    //signal(SIGALRM, signal_handler);
    //set_timer();

    // // Debug - zhihengw
    // modbus_read_registers(ctx, PA1_05_ad,  2,  tab_rp_registers);
    // fprintf(stderr,"DEB: PA1_05 %.10d\n",tab_rp_registers[1]);
    // modbus_read_registers(ctx, PA1_06_ad,  2,  tab_rp_registers);
    // fprintf(stderr,"DEB: PA1_06 %.10d\n",tab_rp_registers[1]);
    // modbus_read_registers(ctx, PA1_07_ad,  2,  tab_rp_registers);
    // fprintf(stderr,"DEB: PA1_07 %.10d\n",tab_rp_registers[1]);

    // TODO loop for getting degree
    stop_flag = 0;
    while(!stop_flag) {
            //int temp = get_encoder_position();
	    //fprintf(stderr, "INF: actual position: %.10d\n",temp);
            signal_handler(0);
            usleep(100000);
    }

    // close
    serve_off();
    close_modbus_rtu_master();
    free_buffers_for_modbus();
    close_uart();
    elog_close();

    return 0;
}


void set_timer()
{
    struct itimerval itv;
    itv.it_value.tv_sec = 0;
    itv.it_value.tv_usec = 500000;        // 500ms start time
    itv.it_interval.tv_sec = 0;
    itv.it_interval.tv_usec = 200000;     // 200ms inter time
    setitimer(ITIMER_REAL, &itv, &oldtv);
}
void signal_handler(int m)
{
        int ret;

        param temp = get_g_x();
        ret = temp.cmd;
        printf("signal handler:");
        printf("    cmd: %d\n",ret);

        if ( temp.cmd & GPST_CANCEL )    // cancel
        {
            ret = positioning_cancel_on();
            if(-1 == ret) exit(-1);
            ret = positioning_cancel_off();
            if(-1 == ret) exit(-1);
            temp.cmd = 0;
            update_g_x(temp);
        }
        else if ( temp.cmd & GEMG )      // stop
        {
            ret = forced_stop_on();
            if(-1 == ret) exit(-1);
            ret = forced_stop_off();
            if(-1 == ret) exit(-1);
            temp.cmd = 0;
            update_g_x(temp);
            stop_flag = 1;
        }
        else if ( temp.cmd & GPOINT )    // run to point 
        {
            printf("run to point %d\n",temp.v[0]);
            set_point_position(temp.v[0]);
            ret = run_to_point();
            if(-1 == ret) exit(-1);
            else if(1 == ret){
                temp.cmd &= ~GPOINT;
                update_g_x(temp);
	    }
        }
        else if ( temp.cmd & GLEFT )     // run to left
        {
            ret = left_direction_run();
            if(-1 == ret) exit(-1);
            else if(1 == ret){
                temp.cmd &= ~GLEFT;
                update_g_x(temp);
            }
        }
        else if ( temp.cmd & GRIGHT )    // run to right 
        {
            ret = right_direction_run();
            if(-1 == ret) exit(-1);
            else if(1 == ret){
                temp.cmd &= ~GRIGHT;
                update_g_x(temp);
            }
        }
        else if ( temp.cmd & GSPEED ) 
        {
            set_cruise_speed(temp.v[0]);
            if(is_INP()){
                ret = send_cruise_speed();
                if (-1 == ret) exit(-1);
                temp.cmd &= ~GSPEED;
                update_g_x(temp);
            }
        }
        else if ( temp.cmd & GACCE_TIME ) 
        {
            set_imme_acceleration_time(temp.v[0]);
            if(is_INP()){
                ret = send_imme_acceleration_time();
                if (-1 == ret) exit(-1);
                temp.cmd &= ~GACCE_TIME;
                update_g_x(temp);
	    }
        }
        else if ( temp.cmd & GDECE_TIME ) 
        {
            set_imme_deceleration_time(temp.v[0]);
            if(is_INP()){
                ret = send_imme_deceleration_time();
                if (-1 == ret) exit(-1);
                temp.cmd &= ~GDECE_TIME;
                update_g_x(temp);
            }
        }
        else if ( temp.cmd & GMAX_POINT )
        {
            set_max_left_position(temp.v[0]);
            set_max_right_position(temp.v[1]);
            temp.cmd &= ~GMAX_POINT;
            update_g_x(temp);
        }
        else if ( temp.cmd & GSTATUS )
        {

            temp.cmd &= ~GSTATUS;
            update_g_x(temp);
        }

        //
        int position = get_encoder_position();
        printf("actual encoder position: %.10d\n", position);

        // Get constrol status
        if(0) {

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
    "cruise_speed = 50000"                  "\n"
    "cruise_left_position = -5050000"       "\n"
    "cruise_right_position = 5050000"       "\n"
    "direct_left_position = -10100000"      "\n"
    "direct_right_position = 10100000"      "\n"
    "imme_acceleration_time = 10000"        "\n"
    "imme_deceleration_time = 10000"        "\n"
    "max_left_position = -10100000"         "\n"
    "max_right_position = 10100000"         "\n\n"

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
    "queue_size = 10"                       "\n"
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
    uint32_t cruise_speed;
    uint32_t imme_acceleration_time;
    uint32_t imme_deceleration_time;

    ini = iniparser_load(ini_name);
    if (ini==NULL) {
        log_e("Load configure.ini failed.");
        return -1 ;
    }
    iniparser_dump(ini, stderr);

    // Get configure
    cruise_left_position = iniparser_getint(ini, "Motion Control:cruise_left_position", 0);
    cruise_right_position = iniparser_getint(ini, "Motion Control:cruise_right_position", 0);
    direct_left_position = iniparser_getint(ini, "Motion Control:direct_left_position", 0);
    direct_right_position = iniparser_getint(ini, "Motion Control:direct_right_position", 0);
    max_left_position = iniparser_getint(ini, "Motion Control:max_left_position", 0);
    max_right_position = iniparser_getint(ini, "Motion Control:max_right_position", 0);
    cruise_speed = iniparser_getint(ini, "Motion Control:cruise_speed", 0);
    imme_acceleration_time = iniparser_getint(ini, "Motion Control:imme_acceleration_time", 0);
    imme_deceleration_time = iniparser_getint(ini, "Motion Control:imme_deceleration_time", 0);

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

    // set control parameter
    set_cruise_left_position(cruise_left_position);
    set_cruise_right_position(cruise_right_position);
    set_direct_left_position(direct_left_position);
    set_direct_right_position(direct_right_position);
    set_max_left_position(max_left_position);
    set_max_right_position(max_right_position);
    set_cruise_speed(cruise_speed);
    set_imme_acceleration_time(imme_acceleration_time);
    set_imme_deceleration_time(imme_deceleration_time);
    set_point_position(0);


    iniparser_freedict(ini);
    return 0 ;
}
