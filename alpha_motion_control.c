#include <stdio.h>
#include "modbus.h"
#include "alpha_setting.h"
#include "alpha_motion_control.h"

// mode: 0-ABS	1-INC
static uint16_t cruise_speed[2];
static uint16_t cruise_left_position[2];
static uint16_t cruise_right_position[2];
static uint16_t imme_acceleration_time[2];
static uint16_t imme_deceleration_time[2];


// immediate value control run
void immediate_value_operation_run()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, START_ad, 1); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, START_ad, 0); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}

// check inpositon
static inline int is_INP()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_read_bits(ctx, INP_ad,1,tab_rp_bits); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
		return -1;
	}
	if(1==tab_rp_bits[0]) return 1;
	return 0;
}
// wait until in position
void wait_until_inposition_on()
{
	while(1!=is_INP()){ }
}

// Immediate value data setting: 
//		The immediate value status of immediate data is configured as follows:
//		Data||4 bytes|| Immediate value status			1byte	(ntoe: bit0 configure the command method,0:ABS 1:INC)
//					 || Immediate value value M code	1byte
//					 || Not used						2bytes
// Unidirectional movement
void left_direction_run()
{
	tab_rq_registers[0] = INC_POSITION_MODE << 8;
	tab_rq_registers[1] = 0x0000;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_STATUS_ad, 2, tab_rq_registers); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
	while(TRUE){
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_left_position); i++){
			if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
		}
		immediate_value_operation_run();
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_left_position); i++){
			if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
		}
		wait_until_inposition_on();
		immediate_value_operation_run();
		wait_until_inposition_on();
	}
}
void right_direction_run()
{
	tab_rq_registers[0] = INC_POSITION_MODE << 8;
	tab_rq_registers[1] = 0x0000;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_STATUS_ad, 2, tab_rq_registers); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
	while(TRUE){
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_right_position); i++){
			if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
		}
		immediate_value_operation_run();
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_right_position); i++){
			if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
		}
		wait_until_inposition_on();
		immediate_value_operation_run();
		wait_until_inposition_on();
	}
}

// Cruise Control (巡航控制)
void cruise()
{
	tab_rq_registers[0] = ABS_POSITION_MODE << 8;
	tab_rq_registers[1] = 0x0000;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_STATUS_ad, 2, tab_rq_registers); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
	while(TRUE){
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_left_position); i++){
			if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
		}
		immediate_value_operation_run();
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_right_position); i++){
			if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
		}
		wait_until_inposition_on();
		immediate_value_operation_run();
		wait_until_inposition_on();
	 }
}
// position: +/- 2,000,000,000
void set_cruise_left_position(const int32_t position)
{
	cruise_left_position[1] = position;
	cruise_left_position[0] = position >> 16;
}
void set_cruise_right_position(const int32_t position)
{
	cruise_right_position[1] = position;
	cruise_right_position[0] = position >> 16;
}
// speed: 0.01r/min
void set_cruise_speed(const uint32_t speed)
{
	cruise_speed[1] = speed;
	cruise_speed[0] = speed >> 16;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_SPEED_ad, 2, cruise_speed); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}
// acc/dec time: 0.1ms
void set_imme_acceleration_time(const uint32_t time)
{
	imme_acceleration_time[1] = time;
	imme_acceleration_time[0] = time >> 16;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_ACC_TIM_ad, 2, imme_acceleration_time); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}
void set_imme_deceleration_time(const uint32_t time)
{
	imme_deceleration_time[1] = time;
	imme_deceleration_time[0] = time >> 16;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_DEC_TIM_ad, 2, imme_deceleration_time); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}


// Servo control
int serve_on()
{
	int rc;

	for(int i = 0; i < OPLOOPS && 1 != modbus_read_bits(ctx,S_RDY_ad,1,tab_rp_bits); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connetion is not stable.\n");
		return -1;
	}
	if(1 != tab_rp_bits[0]){
		fprintf(stderr,"ERR:[S-RDY] not ON");
		return -1;
	}
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx,SERVO_ON_ad,1); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
		return -1;
	}
	return 1;
}

int serve_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx,SERVO_ON_ad,0); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
		return -1;
	}
	return 1;
}

int is_ready()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_read_bits(ctx,RDY_ad,1,tab_rp_bits); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
		return -1;
	}
	if(1==tab_rp_bits[0]) return 1;
	return 0;
}

// Interrupting/Stopping Operation
// [EMG]: forced stop.
void forced_stop_on()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, EMG_ad, 1); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}
void forced_stop_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, EMG_ad, 0); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}

void pause_on()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, PAUSE_ad, 1); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}
void pause_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, PAUSE_ad, 0); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}

void positioning_cancel_on()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, PST_CANCEL_ad, 1); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}
void positioning_cancel_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, PST_CANCEL_ad, 0); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}

// if free-run is turned on, operation is stoped and the motor keeps rotating due to the inertia of the load.
void free_run_on()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, FREE_RUN_ad, 1); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}
void free_run_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, FREE_RUN_ad, 0); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
	}
}
