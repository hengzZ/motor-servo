#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include "elog.h"
#include "iniparser.h"

#include "am335x_setting.h"
#include "alpha_setting.h"
#include "alpha_motion_control.h"

extern int max_left_position;
extern int max_right_position;
extern int listening_console(); 

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
    GLEFT_PST=2048
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
	
    // init buffers for modbus communication
    ret = init_buffers_for_modbus();
    if(-1 == ret){
        log_e("init buffers for modbus failed.");
        return -1;
    }
    ret = open_modbus_rtu_master("/dev/ttyO1",38400,'E',8,1,1);
    if(-1 == ret){
    	log_e("open modbus_rtu_master failed.");
    	free_buffers_for_modbus();
    	return -1;
    }
    // Init Parameter (Warning: remerber just do it one time.)
    // init_parameters();
    // log_i("Init Parameter Done.");
    
    // initialize iniparser
    ret = parse_ini_file("configure.ini");
    if(-1 == ret)
    {
        create_example_ini_file();
        ret = parse_ini_file("configure.ini");
        if(-1 == ret)
        {
            log_e("initialize iniparser failed.");
            free_buffers_for_modbus();
            close_modbus_rtu_master();
            return -1;
        }
    }
    log_i("Init Success.");
    // listening am335x UART
    ret = listening_uart("/dev/ttyO2",9600,'N',8,1);
    if(-1 == ret){
    	log_e("open am335x uart failed.");
    	free_buffers_for_modbus();
    	close_modbus_rtu_master();
    	return -1;
    }
    // serve on
    ret = serve_on();
    if(-1 == ret){
        log_e("servo on failed.");
    	free_buffers_for_modbus();
    	close_modbus_rtu_master();
    	return -1;
    }
    ret = is_ready();
    if(ret != 1){
    	log_e("servo_on is ON, but is_ready OFF.");
    	serve_off();
    	free_buffers_for_modbus();
    	close_modbus_rtu_master();
    	return -1;
    }

    g_flags = GCRUISE | GRIGHT;
    char str[1024];
    sprintf(str,"%.8x",g_flags);
    log_e(str);
    // TODO
    while(TRUE)
    {
        if ( g_flags & GEMG ) 
        {
            log_e("stop.");
            break;
        }else if ( g_flags & GFREE_ON ) 
        {

        }else if ( g_flags & GPST_CANCEL ) 
        {
            log_e("positiion cancel.");
            ret = positioning_cancel_on();
            if(-1 == ret) break;
            ret = positioning_cancel_off();
            if(-1 == ret) break;

        }else if ( g_flags & GPAUSE ) 
        {
            ret = pause_on();
            if (-1 == ret) break;

        }else if ( 0 == g_flags & GPAUSE)
        {
            ret = pause_off();
            if (-1 == ret) break;
        }
        else if ( g_flags & GSPEED ) 
        {
            ret = send_cruise_speed();
            if (-1 == ret) break;

        }else if ( g_flags & GACCE_TIME ) 
        {
            ret = send_imme_acceleration_time();
            if (-1 == ret) break;

        }else if ( g_flags & GDECE_TIME ) 
        {
            ret = send_imme_deceleration_time();
            if (-1 == ret) break;

        }else if ( ( g_flags & GCRUISE ) && ( g_flags & GRIGHT ) ) 
        {
            ret = set_abs_control_mode();
            if(-1 == ret) break;
            ret = right_cruise();
            if (1 == ret) {
                uint32_t temp = read_gflags();
                temp = (temp & (~GRIGHT) ) | GLEFT;

                char str[1024];
                sprintf(str,"%.8x",temp);
                log_e(str);
                
                write_gflags(temp);
            }
            else if(-1 == ret) break;

        }else if ( ( g_flags & GCRUISE ) && ( g_flags & GLEFT) ) 
        {
            ret = set_abs_control_mode();
            if(-1 == ret) break;
            ret = left_cruise();
            if (1 == ret) {
                uint32_t temp = read_gflags();
                temp = (temp & (~GLEFT) ) | GRIGHT;

                char str[1024];
                sprintf(str,"%.8x",temp);
                log_e(str);

                write_gflags(temp);
            }
            else if(-1 == ret) break;

        }else if ( g_flags & GRIGHT ) 
        {
            ret = set_inc_control_mode();
            if(-1 == ret) break;
            ret = right_direction_run();
            if(-1 == ret) break;

        }else if ( g_flags & GLEFT ) 
        {
            ret = set_inc_control_mode();
            if(-1 == ret) break;
            ret = left_direction_run();
            if(-1 == ret) break;

        }else {
            ret = pause_off();
            if(-1 == ret) break;
        }
    }

    // // // test alpha
    // // fprintf(stderr,"INF:test immediate value data control.\n");
    // // immediate_value_data_op_test();
    // getchar();

    // close
    serve_off();
    close_uart();
    free_buffers_for_modbus();
    close_modbus_rtu_master();
    elog_close();

    return 0;
}


void create_example_ini_file(void)
{
    FILE* ini;
    if ((ini=fopen("configure.ini", "w"))==NULL) {
        log_e("iniparser: cannot create configure.ini");
        return ;
    }

    // congfigure parameter
    fprintf(ini,
    "[IP]\n"
    "RemoteIP = 192.168.0.15\n"
    "RemoteName = root\n"
    "ControlIP = \n"
    "\n"
    "[Motion Control]\n"
    "cruise_speed = 200\n"
    "cruise_left_position = -2000\n"
    "cruise_right_position = 2000\n"
    "imme_acceleration_time = 5000\n"
    "imme_deceleration_time = 5000\n"
    "\n"
    "max_left_position = -65536\n"
    "max_right_position = 65536\n"
    "\n");

    fclose(ini);
    log_i("create default configure.ini");
}

int parse_ini_file(char * ini_name)
{
    dictionary  *   ini ;

    int ret;
    // temporary variables
    int32_t cruise_left_position;
    int32_t cruise_right_position;
    int32_t max_left_position;
    int32_t max_right_position;
    uint32_t cruise_speed;
    uint32_t imme_acceleration_time;
    uint32_t imme_deceleration_time;

    ini = iniparser_load(ini_name);
    if (ini==NULL) {
        log_e("cannot parse ini file.");
        return -1 ;
    }
    iniparser_dump(ini, stderr);

    // parse
    cruise_left_position = iniparser_getint(ini, "Motion Control:cruise_left_position", 0);
    cruise_right_position = iniparser_getint(ini, "Motion Control:cruise_right_position", 0);
    max_left_position = iniparser_getint(ini, "Motion Control:max_left_position", 0);
    max_right_position = iniparser_getint(ini, "Motion Control:max_right_position", 0);
    cruise_speed = iniparser_getint(ini, "Motion Control:cruise_speed", 0);
    imme_acceleration_time = iniparser_getint(ini, "Motion Control:imme_acceleration_time", 0);
    imme_deceleration_time = iniparser_getint(ini, "Motion Control:imme_deceleration_time", 0);

    // set control parameter
    set_cruise_left_position(cruise_left_position);
    set_cruise_right_position(cruise_right_position);
    set_cruise_speed(cruise_speed);
    set_imme_acceleration_time(imme_acceleration_time);
    set_imme_deceleration_time(imme_deceleration_time);
    // send setting to motor
    ret = send_cruise_speed();
    if (-1 == ret) return -1;
    ret = send_imme_acceleration_time();
    if (-1 == ret) return -1;
    ret = send_imme_deceleration_time();
    if (-1 == ret) return -1;

    //printf("%.1d\n",cruise_speed);
    //printf("%.1d\n",cruise_left_position);
    //printf("%.1d\n",cruise_right_position);
    //printf("%.1d\n",imme_acceleration_time);
    //printf("%.1d\n",imme_deceleration_time);
    //printf("%.1d\n",max_left_position);
    //printf("%.1d\n",max_right_position);

    iniparser_freedict(ini);
    return 0 ;
}
