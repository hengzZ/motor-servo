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
#ifndef ALPHA_MOTION_CONTROL_H
#define ALPHA_MOTION_CONTROL_H


int serve_on();
int serve_off();
int is_ready();

// function for test use, implementing in file alpha_test.c
void forward_command_test();
void immediate_value_data_op_test();

#endif	//ALPHA_MOTION_CONTROL_H
