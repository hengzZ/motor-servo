#include <unistd.h>
#include <stdio.h>
#include <pthread.h>

#include "elog.h"
#include "alpha_motion_control.h"


#define MAX_LEFT_POSITION	-10100000
#define MAX_RIGHT_POSITION	 10100000

// mode: 0-ABS	1-INC
// flags: | motion  flags | 1. left 2. right 3. point 
//		  | control flags | 1. direct left 2. direct right 3. cruise 4. EMG 5. Pause 6. Positioning cancel 7. Free run
//		  |
uint16_t direct_left_position[2];
uint16_t direct_right_position[2];
uint16_t cruise_left_position[2];
uint16_t cruise_right_position[2];
uint16_t cruise_speed[2];
uint16_t imme_acceleration_time[2];
uint16_t imme_deceleration_time[2];
uint16_t point_position[2];

int32_t limit_left_position;
int32_t limit_right_position;
pthread_mutex_t mutex_motion_ctrl = PTHREAD_MUTEX_INITIALIZER;


// left direction movement
int left_direction_run()
{
	if(is_INP())
	{
		int ret;
		ret = set_abs_control_mode();
		if(-1 == ret) {
			log_e("left_direction_run: set abs control mode failed.");
			return -1;
		}
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, direct_left_position); i++){
			if(OPLOOPS-1==i)
			{
				log_e("left_direction_run: communication failed.");
				return -1;
			}
		}
		ret = immediate_value_operation_run();
		if(0 == ret) return 1;
		else return -1;
	}
	return 0;
}
// right direction movement
int right_direction_run()
{
	if(is_INP())
	{
		int ret;
		ret = set_abs_control_mode();
		if(-1 == ret) {
			log_e("right_direction_run: set abs control mode failed.");
			return -1;
		}
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, direct_right_position); i++){
			if(OPLOOPS-1==i)
			{
				log_e("right_direction_run: communication failed.");
				return -1;
			}
		}
		ret = immediate_value_operation_run();
		if(0 == ret) return 1;
		else return -1;
	}
	return 0;
}

// run to point 
int run_to_point()
{
	if(is_INP())
	{
		int ret;
		ret = set_abs_control_mode();
		if(-1 == ret) {
			log_e("run_to_point: set abs control mode failed.");
			return -1;
		}
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, point_position); i++){
			if(OPLOOPS-1==i)
			{
				log_e("run_to_point: communication failed.");
				return -1;
			}
		}
		ret = immediate_value_operation_run();
		if(0 == ret) return 1;
		else return -1;
	}
	return 0;
}

// left_cruise
int	left_cruise()
{
	if(is_INP())
	{
		int ret;
		ret = set_abs_control_mode();
		if(-1 == ret) {
			log_e("left_cruise: set abs control mode failed.");
			return -1;
		}
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_left_position); i++){
			if(OPLOOPS-1==i)
			{
				log_e("left_cruise: communication failed.");
				return -1;
			}
		}
		ret = immediate_value_operation_run();
		if(0 == ret) return 1;
		else return -1;
	}
	return 0;
}
// right_cruise
int right_cruise()
{
	if(is_INP())
	{
		int ret;
		ret = set_abs_control_mode();
		if(-1 == ret) {
			log_e("right_cruise: set abs control mode failed.");
			return -1;
		}
		for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, cruise_right_position); i++){
			if(OPLOOPS-1==i)
			{
				log_e("right_cruise: communication failed.");
				return -1;
			}
		}
		ret = immediate_value_operation_run();
		if(0 == ret) return 1;
		else return -1;
	}
	return 0;
}
// Cruise Control (无限巡航)
void cruise()
{
	while(TRUE)
	{
		int ret, drct = 0;
		if(0 == drct) {
			ret = left_cruise();
			if(-1 == ret) break;
			else if(1 == ret) drct = 1;
		} else if(1 == drct) {
			ret = right_cruise();
			if(-1 == ret) break;
			else if(1 == ret) drct = 0;
		}
		usleep(200000);		// 2ms
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
//***********************************************************************************************************************
int set_abs_control_mode()
{
	tab_rq_registers[0] = ABS_POSITION_MODE << 8;
	tab_rq_registers[1] = 0x0000;
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_STATUS_ad, 2, tab_rq_registers); i++){
		if(OPLOOPS-1==i)
		{
			log_e("set_abs_control_mode: communication failed.");
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
		if(OPLOOPS-1==i)
		{
			log_e("set_inc_control_mode: communication failed.");
			return -1;
		}
	}
	return 0;
}

// immediate value control run
int immediate_value_operation_run()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, START_ad, 1); i++){
		if(OPLOOPS-1==i)
		{
			log_e("set start_on: communication failed.");
			return -1;
		}
	}
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, START_ad, 0); i++){
		if(OPLOOPS-1==i)
		{
			log_e("set start_off: communication failed.");
			return -1;
		}
	}
	return 0;
}

// position: +/- 2,000,000,000
int set_cruise_left_position(int32_t position)
{
	pthread_mutex_lock(&mutex_motion_ctrl);
	if(position < limit_left_position) position = limit_left_position;
	if(position > limit_right_position) position = limit_right_position;
	cruise_left_position[1] = position;
	cruise_left_position[0] = position >> 16;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return 0;
}
int set_cruise_right_position(int32_t position)
{
	pthread_mutex_lock(&mutex_motion_ctrl);
	if(position < limit_left_position) position = limit_left_position;
	if(position > limit_right_position) position = limit_right_position;
	cruise_right_position[1] = position;
	cruise_right_position[0] = position >> 16;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return 0;
}
int set_point_position(int32_t position)
{
	pthread_mutex_lock(&mutex_motion_ctrl);
	if(position < limit_left_position) position = limit_left_position;
	if(position > limit_right_position) position = limit_right_position;
	point_position[1] = position;
	point_position[0] = position >> 16;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return 0;
}
int set_direct_left_position(int32_t position)
{
	pthread_mutex_lock(&mutex_motion_ctrl);
	if(position < limit_left_position) position = limit_left_position;
	if(position > limit_right_position) position = limit_right_position;
	direct_left_position[1] = position;
	direct_left_position[0] = position >> 16;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return 0;
}
int set_direct_right_position(int32_t position)
{
	pthread_mutex_lock(&mutex_motion_ctrl);
	if(position < limit_left_position) position = limit_left_position;
	if(position > limit_right_position) position = limit_right_position;
	direct_right_position[1] = position;
	direct_right_position[0] = position >> 16;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return 0;
}
int set_limit_left_position(int32_t position)
{
	pthread_mutex_lock(&mutex_motion_ctrl);
	if(position < MAX_LEFT_POSITION) position = MAX_LEFT_POSITION;
	if(position > MAX_RIGHT_POSITION) position = MAX_RIGHT_POSITION;
	limit_left_position = position;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return 0;
}
int set_limit_right_position(int32_t position)
{
	pthread_mutex_lock(&mutex_motion_ctrl);
	if(position < MAX_LEFT_POSITION) position = MAX_LEFT_POSITION;
	if(position > MAX_RIGHT_POSITION) position = MAX_RIGHT_POSITION;
	limit_right_position = position;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return 0;
}
int32_t get_limit_left_position()
{
	int32_t position;
	pthread_mutex_lock(&mutex_motion_ctrl);
	position = limit_left_position;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return position;
}
int32_t get_limit_right_position()
{
	int32_t position;
	pthread_mutex_lock(&mutex_motion_ctrl);
	position = limit_right_position;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return position;
}

// speed: 0.01r/min
int set_cruise_speed(uint32_t speed)
{
	pthread_mutex_lock(&mutex_motion_ctrl);
	if(speed <= 0) speed = 0;
	if(speed >= 200000) speed = 200000;	// 2000r/min
	cruise_speed[1] = speed;
	cruise_speed[0] = speed >> 16;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return 0;
}
int send_cruise_speed()
{
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_SPEED_ad, 2, cruise_speed); i++){
		if(OPLOOPS-1==i)
		{
			log_e("send_cruise_speed: communication failed.");
			return -1;
		}
	}		
	return 0;
}
// acce time: 0.1ms
int set_imme_acceleration_time(uint32_t time)
{
	pthread_mutex_lock(&mutex_motion_ctrl);
	if(time <= 0) time = 0;
	if(time >= 99999) time = 99999;	// 9.9999s
	imme_acceleration_time[1] = time;
	imme_acceleration_time[0] = time >> 16;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return 0;
}
int send_imme_acceleration_time()
{
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_ACC_TIM_ad, 2, imme_acceleration_time); i++){
		if(OPLOOPS-1==i)
		{
			log_e("send_imme_acceleration_time: communication failed.");
			return -1;
		}
	}		
	return 0;
}
// dece time: 0.1ms
int set_imme_deceleration_time(uint32_t time)
{
	pthread_mutex_lock(&mutex_motion_ctrl);
	if(time <= 0) time = 0;
	if(time >= 99999) time = 99999;	// 9.9999s
	imme_deceleration_time[1] = time;
	imme_deceleration_time[0] = time >> 16;
	pthread_mutex_unlock(&mutex_motion_ctrl);
	return 0;
}
int send_imme_deceleration_time()
{
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_DEC_TIM_ad, 2, imme_deceleration_time); i++){
		if(OPLOOPS-1==i)
		{
			log_e("send_imme_deceleration_time: communication failed.");
			return -1;
		}
	}		
	return 0;
}

// check inpositon
int is_INP()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_read_bits(ctx, INP_ad,1,tab_rp_bits); i++){
		if(OPLOOPS-1==i)
		{
			log_e("is_INP: communication failed.");
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
		if(OPLOOPS-1==i)
		{
			log_e("serve_on: require S_RDY, communication failed.");
			return -1; 
		}
	}
	if(1 != tab_rp_bits[0]) { 
		log_e("serve_on: S_RDY is not ON.");
		return -1;
	}
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx,SERVO_ON_ad,1); i++){
		if(OPLOOPS-1==i)
		{
			log_e("serve_on: communication failed.");
			return -1;
		}
	}
	return 0;
}
// servo off
int serve_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx,SERVO_ON_ad,0); i++){
		if(OPLOOPS-1==i)
		{
			log_e("serve_off: communication failed.");
			return -1;
		}
	}
	return 0;
}
// check ready
int is_ready()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_read_bits(ctx,RDY_ad,1,tab_rp_bits); i++){
		if(OPLOOPS-1==i)
		{
			log_e("is_ready: communication failed.");
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
			log_e("forced_stop_on: communication failed.");
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
			log_e("forced_stop_off: communication failed.");
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
			log_e("pause_on: communication failed.");
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
			log_e("pause_off: communication failed.");
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
			log_e("positioning_cancel_on: communication failed.");
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
			log_e("positioning_cancel_off: communication failed.");
			return -1;
		}
	}
	return 0;
}
// free run
// warn: if free-run is turned on, operation is stoped and the motor keeps rotating due to the inertia of the load.
int free_run_on()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx, FREE_RUN_ad, 1); i++){
		if(i==OPLOOPS-1)
		{
			log_e("free_run_on: communication failed.");
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
			log_e("free_run_off: communication failed.");
			return -1;
		}
	}
	return 0;
}

// Check motion
int check_motion()
{
	uint16_t speed[2];
	uint16_t acce_time[2];
	uint16_t dece_time[2];
	uint16_t home_position[2];
	uint16_t left_position[2];
	uint16_t right_position[2];

	int ret;
	// Cancel old task
	if(!is_INP()) {
		ret = positioning_cancel_on();
		if(-1 == ret) return -1;
		ret = positioning_cancel_off();
		if(-1 == ret) return -1;
	}

	speed[0] = 250000 >> 16;	// 2500r/min
	speed[1] = 250000&0xFFFF;
	acce_time[0] = 10000 >> 16;	// 1s
	acce_time[1] = 10000;
	dece_time[0] = 10000 >> 16;	// 1s
	dece_time[1] = 10000;
	home_position[0] = 0;
	home_position[1] = 0;
	left_position[0] = limit_left_position >> 16;
	left_position[1] = limit_left_position;
	right_position[0] = limit_right_position >> 16;
	right_position[1] = limit_right_position;

	// Set ckeck_motion speed, acce time, dece time
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_SPEED_ad, 2, speed); i++){
		if(OPLOOPS-1==i)
		{
			log_e("check_motion: communication failed.");
			return -1;
		}
	}		
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_ACC_TIM_ad, 2, acce_time); i++){
		if(OPLOOPS-1==i)
		{
			log_e("check_motion: communication failed.");
			return -1;
		}
	}
	for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_DEC_TIM_ad, 2, dece_time); i++){
		if(OPLOOPS-1==i)
		{
			log_e("check_motion: communication failed.");
			return -1;
		}
	}		
	// Point home
	while(1){
		if(is_INP())
		{
			ret = set_abs_control_mode();
			if(-1 == ret) {
				log_e("check_motion: set abs control mode failed.");
				return -1;
			}
			for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, home_position); i++){
				if(OPLOOPS-1==i)
				{
					log_e("check_motion: communication failed.");
					return -1;
				}
			}
			ret = immediate_value_operation_run();
			if(0 == ret) break;
			else return -1;
		}
		usleep(200000);		// 200ms
	}
	// Point limit_left_position
	while(1){
		if(is_INP())
		{
			ret = set_abs_control_mode();
			if(-1 == ret) {
				log_e("check_motion: set abs control mode failed.");
				return -1;
			}
			for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, left_position); i++){
				if(OPLOOPS-1==i)
				{
					log_e("check_motion: communication failed.");
					return -1;
				}
			}
			ret = immediate_value_operation_run();
			if(0 == ret) break;
			else return -1;
		}
		usleep(200000);		// 200ms
	}
	// Point limit_right_position
	while(1){
		if(is_INP())
		{
			ret = set_abs_control_mode();
			if(-1 == ret) {
				log_e("check_motion: set abs control mode failed.");
				return -1;
			}
			for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, right_position); i++){
				if(OPLOOPS-1==i)
				{
					log_e("check_motion: communication failed.");
					return -1;
				}
			}
			ret = immediate_value_operation_run();
			if(0 == ret) break;
			else return -1;
		}
		usleep(200000);		// 200ms
	}
	// Point home
	while(1){
		if(is_INP())
		{
			ret = set_abs_control_mode();
			if(-1 == ret) {
				log_e("check_motion: set abs control mode failed.");
				return -1;
			}
			for(int i = 0; i < OPLOOPS && 2 != modbus_write_registers(ctx, IMME_VLU_POSITION_ad, 2, home_position); i++){
				if(OPLOOPS-1==i)
				{
					log_e("check_motion: communication failed.");
					return -1;
				}
			}
			ret = immediate_value_operation_run();
			if(0 == ret) break;
			else return -1;
		}
		usleep(200000);		// 200ms
	}

	// Set default speed, acce time, dece time
	ret = send_cruise_speed();
	if(-1 == ret) return -1;
	ret = send_imme_acceleration_time();
	if(-1 == ret) return -1;
	ret = send_imme_deceleration_time();
	if(-1 == ret) return -1;

	return 0;
}
