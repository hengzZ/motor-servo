//*************************************************************************************************************************************
//Notes:    Priority among Input Signals
//Section            Description                                    Application signal(Function No.)
//  01               (Operation) Highest Priority                   54,  1
//  02               (Operation) Priority                           10, 34
//  03               Controlling the torque                         19, 20 
//  04               Stopping the motor                              7,  8, 26, 31, 32, 50
//  05               turning the motor                               2,  3,  4,  5
//  06               determing the home position                     6,  7,  8,  49,16
//  07               Signal irrelativant to motor operation          11,55
//*************************************************************************************************************************************
#ifndef ALPHA_MOTION_CONTROL_H
#define ALPHA_MOTION_CONTROL_H

#include "alpha_setting.h"

#define ABS_POSITION_MODE   0
#define INC_POSITION_MODE   1


// control
int left_direction_run();
int right_direction_run();
int run_to_point();
int left_cruise();
int right_cruise();
void cruise();

// Motion control setting
int set_abs_control_mode();
int set_inc_control_mode();
int immediate_value_operation_run();
int set_cruise_left_position(int32_t position);
int set_cruise_right_position(int32_t position);
int set_point_position(int32_t position);
int set_direct_left_position(int32_t position);
int set_direct_right_position(int32_t position);
int set_limit_left_position(int32_t position);
int set_limit_right_position(int32_t position);
int32_t get_limit_left_position();
int32_t get_limit_right_position();
int set_cruise_speed(uint32_t speed);
int send_cruise_speed();
int set_imme_acceleration_time(uint32_t time);
int send_imme_acceleration_time();
int set_imme_deceleration_time(uint32_t time);
int send_imme_deceleration_time();
int is_INP();

// Servo control
int serve_on();
int serve_off();
int is_ready();

// Interrupting/Stopping Operation
int forced_stop_on();
int forced_stop_off();
int pause_on();
int pause_off();
int positioning_cancel_on();
int positioning_cancel_off();
int free_run_on();
int free_run_off();

// Check motion
int check_motion();

// functions for test use, implementing in file alpha_test.c
// **NoTE: PA1_01 must be set 1
void forward_command_test();
// **NoTE: PA1_01 must be set 7
void immediate_value_data_op_test();

#endif	//ALPHA_MOTION_CONTROL_H
