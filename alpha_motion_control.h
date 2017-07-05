//*************************************************************************************************************************************
//Notes:    Priority among Input Signals(请参照以下的输入信号优先级)
//Section            Description                                    Application signal(信号功能码)
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


// 控制指令
int left_direction_run();
int right_direction_run();
int run_to_point();
int left_cruise();
int right_cruise();
void cruise();

// 低层控制函数
int set_abs_control_mode(); //绝对位置模式
int set_inc_control_mode();
int immediate_value_operation_run(); //运行，run
// 目标位置参数设置
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

int set_check_speed(uint32_t speed);
int set_check_acce_time(uint32_t time);
int set_check_dece_time(uint32_t time);

// 立即数位置到达
int is_INP();
// 伺服
int serve_on();
int serve_off();
int is_ready();

// 暂停/停止
int forced_stop_on();
int forced_stop_off();
int pause_on();
int pause_off();
int positioning_cancel_on();
int positioning_cancel_off();
int free_run_on();
int free_run_off();

// 自检，执行极限之间的一次巡航
int check_motion();

// functions for test use, implementing in file alpha_test.c (旧的测试，没有使用)
// **NoTE: PA1_01 must be set 1
void forward_command_test();
// **NoTE: PA1_01 must be set 7
void immediate_value_data_op_test();

#endif	//ALPHA_MOTION_CONTROL_H
