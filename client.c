#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "elog.h"
#include "iniparser.h"

#include "am335x_setting.h"
#include "alpha_setting.h"
#include "alpha_motion_control.h"


void create_example_ini_file(void);
int  parse_ini_file(char * ini_name);

int main(int argc, char** argv)
{
    // test log
    /* close printf buffer */
    setbuf(stdout, NULL);
    /* initialize EasyLogger */
    elog_init();
    /* set EasyLogger log format */
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);

    /* start EasyLogger */
    elog_start();

    while(true) {
    /* test log output for all level */
    log_a("Hello EasyLogger!");
    log_e("Hello EasyLogger!");
    log_w("Hello EasyLogger!");
    log_i("Hello EasyLogger!");
    log_d("Hello EasyLogger!");
    log_v("Hello EasyLogger!");
    // elog_raw("Hello EasyLogger!");
    sleep(5);
    }
	
    // // test iniparser
    // int status ;

    // if (argc<2) {
    //     create_example_ini_file();
    //     status = parse_ini_file("configure.ini");
    // } else {
    //     status = parse_ini_file(argv[1]);
    // }
    // return status ;


    // int ret;

    // // init
    // init_buffers_for_modbus();
    // ret = open_modbus_rtu_master("/dev/ttyO1",38400,'E',8,1,1);
    // if(ret != 1){
    // 	fprintf(stderr,"ERR:open modbus_rtu_master failed.\n");
    // 	free_buffers_for_modbus();
    // 	return -1;
    // }
    // // Init Parameter (Warning: remerber just do it one time.)
    // init_parameters();
    // fprintf(stderr,"Init Parameter Done.\n");
    // getchar();
    
    // // serve on
    // if(serve_on()) ret = is_ready();
    // if(ret != 1){
    // 	fprintf(stderr,"ERR:serve_on is ON, but is_ready OFF\n");
    // 	serve_off();
    // 	free_buffers_for_modbus();
    // 	close_modbus_rtu_master();
    // 	return -1;
    // }
    // // listening am335x UART
    // ret = listening_uart("/dev/ttyO2",9600,'N',8,1);
    // if(ret != 1){
    // 	fprintf(stderr,"ERR:open am335x uart failed.\n");
    // 	serve_off();
    // 	free_buffers_for_modbus();
    // 	close_modbus_rtu_master();
    // 	return -1;
    // }


    // // set_cruise_left_position(10000);
    // // set_cruise_right_position(-10000);
    // // set_cruise_speed(10000);
    // while(0)
    // {

    // }

    // // // test alpha
    // // fprintf(stderr,"INF:test immediate value data control.\n");
    // // immediate_value_data_op_test();
    // getchar();


    // // close
    // serve_off();
    // close_uart();
    // free_buffers_for_modbus();
    // close_modbus_rtu_master();
    return 0;
}


void create_example_ini_file(void)
{
    FILE    *   ini ;

    if ((ini=fopen("configure.ini", "w"))==NULL) {
        fprintf(stderr, "iniparser: cannot create example.ini\n");
        return ;
    }

    fprintf(ini,
    "#\n"
    "# This is an example of ini file\n"
    "#\n"
    "\n"
    "[Pizza]\n"
    "\n"
    "Ham       = yes ;\n"
    "Mushrooms = TRUE ;\n"
    "Capres    = 0 ;\n"
    "Cheese    = Non ;\n"
    "\n"
    "\n"
    "[Wine]\n"
    "\n"
    "Grape     = Cabernet Sauvignon ;\n"
    "Year      = 1989 ;\n"
    "Country   = Spain ;\n"
    "Alcohol   = 12.5  ;\n"
    "\n");
    fclose(ini);
}


int parse_ini_file(char * ini_name)
{
    dictionary  *   ini ;

    /* Some temporary variables to hold query results */
    int             b ;
    int             i ;
    double          d ;
    const char  *   s ;

    ini = iniparser_load(ini_name);
    if (ini==NULL) {
        fprintf(stderr, "cannot parse file: %s\n", ini_name);
        return -1 ;
    }
    iniparser_dump(ini, stderr);

    /* Get pizza attributes */
    printf("Pizza:\n");

    b = iniparser_getboolean(ini, "pizza:ham", -1);
    printf("Ham:       [%d]\n", b);
    b = iniparser_getboolean(ini, "pizza:mushrooms", -1);
    printf("Mushrooms: [%d]\n", b);
    b = iniparser_getboolean(ini, "pizza:capres", -1);
    printf("Capres:    [%d]\n", b);
    b = iniparser_getboolean(ini, "pizza:cheese", -1);
    printf("Cheese:    [%d]\n", b);

    /* Get wine attributes */
    printf("Wine:\n");
    s = iniparser_getstring(ini, "wine:grape", NULL);
    printf("Grape:     [%s]\n", s ? s : "UNDEF");

    i = iniparser_getint(ini, "wine:year", -1);
    printf("Year:      [%d]\n", i);

    s = iniparser_getstring(ini, "wine:country", NULL);
    printf("Country:   [%s]\n", s ? s : "UNDEF");

    d = iniparser_getdouble(ini, "wine:alcohol", -1.0);
    printf("Alcohol:   [%g]\n", d);

    iniparser_freedict(ini);
    return 0 ;
}
