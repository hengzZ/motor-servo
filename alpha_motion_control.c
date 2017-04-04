#include <stdio.h>
#include "modbus.h"
#include "alpha_setting.h"
#include "alpha_motion_control.h"

void positive_positioning()
{

}
void negative_positioning()
{

}
void cruise()
{

}

void forward_run()
{

}
void backward_run()
{

}


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
	return -1;
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
