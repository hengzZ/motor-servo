#include <stdio.h>
#include "modbus.h"
#include "alpha_setting.h"
#include "alpha_motion_control.h"

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
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx,S_ON_ad,1); i++){
		if(i==OPLOOPS-1) fprintf(stderr,"ERR:connection is not stable.\n");
		return -1;
	}
	return 1;
}

int serve_off()
{
	for(int i = 0; i < OPLOOPS && 1 != modbus_write_bit(ctx,S_ON_ad,0); i++){
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
// Interrupting/Stopping Operation
void forced_stop()
{

}

void pause()
{

}

void positioning_cancel()
{

}

void free_run()
{

}
