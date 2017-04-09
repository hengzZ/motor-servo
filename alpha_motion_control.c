#include <stdio.h>
#include "alpha_motion_control.h"

#include "elog.h"

// mode: 0-ABS	1-INC
// flags: | motion  flags | 1. left 2. right 3. if cruise 4. 
//		  | control flags | 1. direct left 2. direct right 3. cruise 4. EMG 5. Pause 6. Positioning cancel 7. Free run
//		  |
uint16_t cruise_speed[2];
uint16_t cruise_left_position[2];
uint16_t cruise_right_position[2];
uint16_t imme_acceleration_time[2];
uint16_t imme_deceleration_time[2];


// left direction movement
int left_direction_run()
{
	if(is_INP())
	{
		int ret;
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_left_position); i++){
			if(i==OPLOOPS-1)
			{
				log_e("communication failed.");
				return -1;
			}
		}
		ret = immediate_value_operation_run();
		return ret;
	}
	return 0;
}
// right direction movement
int right_direction_run()
{
	if(is_INP())
	{
		int ret;
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_right_position); i++){
			if(i==OPLOOPS-1)
			{
				log_e("communication failed.");
				return -1;
			}
		}
		ret = immediate_value_operation_run();
		return ret;
	}
	return 0;
}

// left_cruise
int	left_cruise()
{
	if(is_INP())
	{
		int ret;
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_left_position); i++){
			if(i==OPLOOPS-1)
			{
				log_e("communication failed.");
				return -1;
			}
		}
		ret = immediate_value_operation_run();
		return ret;
	}
	return 0;
}
// right_cruise
int right_cruise()
{
	if(is_INP())
	{
		int ret;
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_right_position); i++){
			if(i==OPLOOPS-1)
			{
				log_e("communication failed.");
				return -1;
			}
		}
		ret = immediate_value_operation_run();
		return ret;
	}
	return 0;
}
// Cruise Control (巡航控制)
void cruise()
{
	while(TRUE)
	{
		int ret;
		ret = left_cruise();
		ret = right_cruise();
	}
}


//***********************************************************************************************************************
// Immediate value data setting: 
//		The immediate value status of immediate data is configured as follows:
//		Data||4 bytes|| Immediate value status			1byte	(ntoe: bit0 configure the command method,0:ABS 1:INC)
//					 || Immediate value value M code	1byte
//					 || Not used						2bytes
//***********************************************************************************************************************
// tab_rq_registers[0] = INC_POSITION_MODE << 8;
// tab_rq_registers[1] = 0x0000;
// for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_STATUS_ad, 2, tab_rq_registers); i++){
// 	if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
// }
int set_abs_control_mode()
{
	tab_rq_registers[0] = ABS_POSITION_MODE << 8;
	tab_rq_registers[1] = 0x0000;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_STATUS_ad, 2, tab_rq_registers); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}		
	return 0;
}
int set_inc_control_mode()
{
	tab_rq_registers[0] = INC_POSITION_MODE << 8;
	tab_rq_registers[1] = 0x0000;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_STATUS_ad, 2, tab_rq_registers); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}

// immediate value control run
int immediate_value_operation_run()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, START_ad, 1); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, START_ad, 0); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}

// position: +/- 2,000,000,000
int set_cruise_left_position(const int32_t position)
{
	cruise_left_position[1] = position;
	cruise_left_position[0] = position >> 16;
	return 0;
}
int set_cruise_right_position(const int32_t position)
{
	cruise_right_position[1] = position;
	cruise_right_position[0] = position >> 16;
	return 0;
}

// speed: 0.01r/min
int set_cruise_speed(const uint32_t speed)
{
	cruise_speed[1] = speed;
	cruise_speed[0] = speed >> 16;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_SPEED_ad, 2, cruise_speed); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}

// acc/dec time: 0.1ms
int set_imme_acceleration_time(const uint32_t time)
{
	imme_acceleration_time[1] = time;
	imme_acceleration_time[0] = time >> 16;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_ACC_TIM_ad, 2, imme_acceleration_time); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}
int set_imme_deceleration_time(const uint32_t time)
{
	imme_deceleration_time[1] = time;
	imme_deceleration_time[0] = time >> 16;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_DEC_TIM_ad, 2, imme_deceleration_time); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}

// check inpositon
int is_INP()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_read_bits(ctx, INP_ad,1,tab_rp_bits); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return 0;
		} 
	}
	if(1==tab_rp_bits[0]) return 1;
	return 0;
}

// servo on
int serve_on()
{
	int rc;

	for(int i = 0; i < OPLOOPS && 1 != modbus_read_bits(ctx,S_RDY_ad,1,tab_rp_bits); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1; 
		}
	}
	if(1 != tab_rp_bits[0]) return -1;
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx,SERVO_ON_ad,1); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}
// servo off
int serve_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx,SERVO_ON_ad,0); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}
// check ready
int is_ready()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_read_bits(ctx,RDY_ad,1,tab_rp_bits); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return 0;
		}
	}
	if(1==tab_rp_bits[0]) return 1;
	return 0;
}

// Interrupting/Stopping Operation
// [EMG]: forced stop.
int forced_stop_on()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, EMG_ad, 1); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}
int forced_stop_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, EMG_ad, 0); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}
// pause
int pause_on()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, PAUSE_ad, 1); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}
int pause_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, PAUSE_ad, 0); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}
// positoning cancel
int positioning_cancel_on()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, PST_CANCEL_ad, 1); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}
int positioning_cancel_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, PST_CANCEL_ad, 0); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}
// free run
// if free-run is turned on, operation is stoped and the motor keeps rotating due to the inertia of the load.
int free_run_on()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, FREE_RUN_ad, 1); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}
int free_run_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, FREE_RUN_ad, 0); i++){
		if(i==OPLOOPS-1)
		{
			log_e("communication failed.");
			return -1;
		}
	}
	return 0;
}
