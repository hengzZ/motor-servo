#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "am335x_setting.h"
#include "alpha_setting.h"
#include "alpha_motion_control.h"


int main(int argc, char** argv)
{
	int ret;

	// init
	init_buffers_for_modbus();
	ret = open_modbus_rtu_master("/dev/ttyO1",38400,'E',8,1,1);
	if(ret != 1){
		fprintf(stderr,"ERR:open modbus_rtu_master failed.\n");
		free_buffers_for_modbus();
		return -1;
	}
	// // Init Parameter (Warning: remerber just do it one time.)
	// init_parameters();
	// fprintf(stderr,"Init Parameter Done.\n");
	// getchar();
	
	// serve on
	if(serve_on()) ret = is_ready();
	if(ret != 1){
		fprintf(stderr,"ERR:serve_on is ON, but is_ready OFF\n");
		serve_off();
		free_buffers_for_modbus();
		close_modbus_rtu_master();
		return -1;
	}
	// listening am335x UART
	// ret = listening_uart("/dev/ttyO2",9600,'N',8,1);
	// if(ret != 1){
	// 	fprintf(stderr,"ERR:open am335x uart failed.\n");
	// 	serve_off();
	// 	free_buffers_for_modbus();
	// 	close_modbus_rtu_master();
	// 	return -1;
	// }
	// Control thread Init

	// // Motor Control
	// set_cruise_left_position(10000);
	// set_cruise_right_position(-10000);
	// set_cruise_speed(10000);
	// //cruise();
	// left_direction_run();
	


	// test alpha
	//fprintf(stderr,"INF:test immediate value data control.\n");
	//immediate_value_data_op_test();

	// pause
	fprintf(stderr,"INF:press Enter to continue..\n");
	getchar();


	// close
	serve_off();
	close_uart();
	free_buffers_for_modbus();
	close_modbus_rtu_master();
	return 0;
}
