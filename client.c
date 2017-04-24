#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>

#include "elog.h"
#include "iniparser.h"

#include "cmdparser.h"
#include "am335x_setting.h"
#include "alpha_setting.h"
#include "alpha_motion_control.h"


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
volatile uint32_t g_flags;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
void write_gflags(uint32_t flags)
{
    pthread_mutex_lock(&mutex);
    g_flags = flags;
    pthread_mutex_unlock(&mutex);
}
uint32_t read_gflags()
{
    uint32_t ret;
    pthread_mutex_lock(&mutex);
    ret = g_flags;
    pthread_mutex_unlock(&mutex);
    return ret;
}

// Socket attributes
am335x_socket_t g_am335x_socket;
//  RTU master attributes
rtu_master_t g_rtu_master;
// UART attributes
uart_t g_am335x_uart;

void create_example_ini_file(void);
int  parse_ini_file(char* ini_name);
void save_ini_file(char* ini_name);


int main(int argc, char** argv)
{
    int ret;

    // initialize EasyLogger
    elog_init();
    // set EasyLogger log format
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);
    // start EasyLogger
    elog_start();

    // initialize iniparser
    ret = parse_ini_file("configure.ini");
    if(-1 == ret)
    {
        create_example_ini_file();
        ret = parse_ini_file("configure.ini");
        if(-1 == ret)
        {
            log_e("Configure failed.");
            return -1;
        }
    }
    log_i("Configure Success.");

    // init buffers for modbus communication
    ret = init_buffers_for_modbus();
    if(-1 == ret){
        log_e("init buffers for modbus failed.");
        return -1;
    }
    //ret = open_modbus_rtu_master("/dev/ttyO1", 38400, 'E', 8, 1, 1);
    ret = open_modbus_rtu_master(g_rtu_master.device, g_rtu_master.baud, g_rtu_master.parity, g_rtu_master.data_bit, g_rtu_master.stop_bit, g_rtu_master.slave);
    if(-1 == ret){
    	log_e("open modbus_rtu_master failed.");
    	free_buffers_for_modbus();
    	return -1;
    }
    // Init Parameter (Warning: remerber just do it one time.)
    if(1 == g_rtu_master.reset_parameter){
        init_parameters();
        log_i("WARNNING: Init Parameter Done (Reset Parameters Setting).");
    }

    // TODO listening am335x UART
    //ret = listening_uart("/dev/ttyO2", 9600, 'N', 8, 1);
    ret = listening_uart(g_am335x_uart.device, g_am335x_uart.baud, g_am335x_uart.parity, g_am335x_uart.data_bit, g_am335x_uart.stop_bit);
    if(-1 == ret){
    	log_e("open am335x uart failed.");
    	close_modbus_rtu_master();
    	free_buffers_for_modbus();
    	return -1;
    }
    // TODO listening console
    //ret = listening_console();
    ret = listening_socket(g_am335x_socket.server_port, g_am335x_socket.queue_size);
    if(-1 == ret){
        log_e("open am335x ethernet port failed.");
        close_modbus_rtu_master();
        free_buffers_for_modbus();
        close_uart();
        return -1;
    }

    // serve on
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
    log_i("Init Success.");


    // TODO loop check
    while(TRUE)
    {
        // Stop
        if ( g_flags & GEMG ) 
        {
            ret = forced_stop_on();
            if(-1 == ret) break;
            ret = forced_stop_off();
            if(-1 == ret) break;
            break;
        }
        if ( g_flags & GFREE_ON ) 
        {
            ret = free_run_on();
            if(-1 == ret) break;
            ret = free_run_off();
            if(-1 == ret) break;
            uint32_t temp_gflags = 0;
            write_gflags(temp_gflags);
        }
        if ( g_flags & GPST_CANCEL ) 
        {
            ret = positioning_cancel_on();
            if(-1 == ret) break;
            ret = positioning_cancel_off();
            if(-1 == ret) break;
            uint32_t temp_gflags = 0;
            write_gflags(temp_gflags);
        }
        // Pause
        if ( g_flags & GPAUSE ) 
        {
            ret = pause_on();
            if (-1 == ret) break;
            uint32_t temp_gflags = read_gflags();
            temp_gflags &= (~GPAUSE);
            write_gflags(temp_gflags);

        }else if( g_flags & GPAUSE_OFF ) {
            ret = pause_off();
            if(-1 == ret) break;
            uint32_t temp_gflags = read_gflags();
            temp_gflags &= (~GPAUSE_OFF);
            write_gflags(temp_gflags);
        }
        // Speed setting
        if ( g_flags & GSPEED ) 
        {
            ret = send_cruise_speed();
            if (-1 == ret) break;
            uint32_t temp_gflags = read_gflags();
            temp_gflags &= (~GSPEED);
            write_gflags(temp_gflags);
        }
        if ( g_flags & GACCE_TIME ) 
        {
            ret = send_imme_acceleration_time();
            if (-1 == ret) break;
            uint32_t temp_gflags = read_gflags();
            temp_gflags &= (~GACCE_TIME);
            write_gflags(temp_gflags);
        }
        if ( g_flags & GDECE_TIME ) 
        {
            ret = send_imme_deceleration_time();
            if (-1 == ret) break;
            uint32_t temp_gflags = read_gflags();
            temp_gflags &= (~GDECE_TIME);
            write_gflags(temp_gflags);
        }
        // Motion Control
        if ( ( g_flags & GCRUISE ) && ( g_flags & GRIGHT ) ) 
        {
            ret = right_cruise();
            if (1 == ret) {
                uint32_t temp = read_gflags();
                temp = temp & (~GRIGHT) | GLEFT;
                write_gflags(temp);
            }
            else if(-1 == ret) break;

        }else if ( ( g_flags & GCRUISE ) && ( g_flags & GLEFT) ) {
            ret = left_cruise();
            if (1 == ret) {
                uint32_t temp = read_gflags();
                temp = temp & (~GLEFT) | GRIGHT;
                write_gflags(temp);
            }
            else if(-1 == ret) break;

        }else if ( g_flags & GRIGHT ) {
            ret = right_direction_run();
            if(-1 == ret) break;

        }else if ( g_flags & GLEFT ) {
            ret = left_direction_run();
            if(-1 == ret) break;
        }
    }

    // // test alpha
    // fprintf(stderr,"INF:test immediate value data control.\n");
    // immediate_value_data_op_test();
    // getchar();

    // close
    serve_off();
    close_modbus_rtu_master();
    free_buffers_for_modbus();
    close_uart();
    elog_close();

    return 0;
}


void create_example_ini_file(void)
{
    FILE* ini;
    if ((ini=fopen("configure.ini", "w"))==NULL) {
        log_e("Create ini file failed.");
        return ;
    }

    // congfigure parameter
    fprintf(ini,
    "[IP]\n"
    "RemoteIP = 192.168.0.15\n"
    "RemoteName = root\n"
    "\n"

    "[Motion Control]\n"
    "cruise_speed = 5000\n"
    "cruise_left_position = -200000\n"
    "cruise_right_position = 200000\n"
    "direct_left_position = -1000000\n"
    "direct_right_position = 1000000\n"
    "imme_acceleration_time = 100000\n"
    "imme_deceleration_time = 100000\n"
    "max_left_position = -1000000\n"
    "max_right_position = 1000000\n"
    "\n"

    "[RTU MASTER]\n"
    "device = /dev/ttyO1\n"
    "baud = 38400\n"
    "parity = E\n"
    "data_bit = 8\n"
    "stop_bit = 1\n"
    "slave = 1\n"
    "RESET = 0\n"
    "\n"

    "[UART]\n"
    "device = /dev/ttyO2\n"
    "baud = 9600\n"
    "parity = N\n"
    "data_bit = 8\n"
    "stop_bit = 1\n"
    "\n"

    "[SOCKET]\n"
    "server_port = 12345\n"
    "queue_size = 10\n"
    "mode = TCP\n"
    "\n"
    
    );

    fclose(ini);
    log_i("Create ini file success.[configure.ini]");
}

int parse_ini_file(char * ini_name)
{
    dictionary  *   ini ;

    // temporary variables
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
        log_e("Load ini file failed.");
        return -1 ;
    }
    iniparser_dump(ini, stderr);

    // parse
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
    g_am335x_socket.queue_size = iniparser_getint(ini, "SOCKET:queue_size", 10);

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

    uint32_t temp_flags = read_gflags();
    temp_flags |= GSPEED | GACCE_TIME | GDECE_TIME;
    write_gflags(temp_flags);

    //printf("%.1d\n",cruise_speed);
    //printf("%.1d\n",cruise_left_position);
    //printf("%.1d\n",cruise_right_position);
    //printf("%.1d\n",imme_acceleration_time);
    //printf("%.1d\n",imme_deceleration_time);
    //printf("%.1d\n",max_left_position);
    //printf("%.1d\n",max_right_position);
    //printf("%s\n",rtu_device);
    //printf("%c\n",rtu_parity);
    //printf("baud:%d\n",rtu_baud);
    //printf("data_bit:%d\n",rtu_data_bit);
    //printf("stop_bit:%d\n",rtu_stop_bit);
    //printf("slave:%d\n",rtu_slave);
    //printf("reset:%d\n",rtu_reset);
    //printf("%s\n",uart_device);
    //printf("strlen(uart_device): %d\n", strlen(uart_device));
    //printf("%c\n",uart_parity);

    iniparser_freedict(ini);
    return 0 ;
}
