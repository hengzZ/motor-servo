#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "alpha_setting.h"

modbus_t *ctx = NULL;

uint16_t *tab_rq_registers;
uint8_t  *tab_rq_bits;
uint16_t *tab_rp_registers; 
uint8_t	 *tab_rp_bits;


void io_signals_mapping();
void positioning_data_operation_485_setting();
void immediate_value_date_operation_485_setting();


int open_modbus_rtu_master(const char *device, int baud, char parity, int data_bit, int stop_bit, int slave)
{
	int rc;
	ctx = modbus_new_rtu(device , baud, parity, data_bit, stop_bit);
	if(ctx == NULL){
		fprintf(stderr,"ERROR: unable to create the limodbus context.\n");
		return -1;
	}
	// SERVER_ID
	rc = modbus_set_slave(ctx,slave);
	if(-1 == rc){
		fprintf(stderr,"ERROR: set slave failed.\n");
		modbus_free(ctx);
		return -1;
	}
	// DEBUD mode
	modbus_set_debug(ctx, TRUE);
	if (modbus_connect(ctx) == -1) {
		fprintf(stderr,"ERROR: connection failed.\n");
		modbus_free(ctx);
		return -1;
	}
	return 1;
}
void  close_modbus_rtu_master()
{
	modbus_close(ctx);
	modbus_free(ctx);
}

void init_buffers_for_modbus()
{
	tab_rq_registers = (uint16_t *) malloc(REGISTERS_BUFFER_SIZE * sizeof(uint16_t));
	memset(tab_rq_registers, 0, REGISTERS_BUFFER_SIZE * sizeof(uint16_t));
	tab_rp_registers = (uint16_t *) malloc(REGISTERS_BUFFER_SIZE * sizeof(uint16_t));
	memset(tab_rp_registers, 0, REGISTERS_BUFFER_SIZE  * sizeof(uint16_t));
	tab_rq_bits = (uint8_t *)malloc(BITS_BUFFER_SIZE * sizeof(uint8_t));
	memset(tab_rq_bits, 0, BITS_BUFFER_SIZE * sizeof(uint8_t));
	tab_rp_bits = (uint8_t *)malloc(BITS_BUFFER_SIZE * sizeof(uint8_t));
	memset(tab_rp_bits, 0, BITS_BUFFER_SIZE * sizeof(uint8_t));
}
void free_buffers_for_modbus()
{
	free(tab_rq_registers);
	free(tab_rp_registers);
	free(tab_rq_bits);
	free(tab_rp_bits);
}

void init_parameters()
{
	// positioning_data_operation_485_setting();
	immediate_value_date_operation_485_setting();
	io_signals_mapping();
}

void io_signals_mapping()
{
	// [CONT] assign
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = SERVO_ON_fc;
	modbus_write_registers(ctx, PA3_9_ad,  2,  tab_rq_registers);
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = FWD_fc;
	modbus_write_registers(ctx, PA3_10_ad, 2, tab_rq_registers);
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = REV_fc;
	modbus_write_registers(ctx, PA3_11_ad, 2, tab_rq_registers);
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = START_fc;
	modbus_write_registers(ctx, PA3_12_ad, 2, tab_rq_registers);

	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = EMG_fc;
	modbus_write_registers(ctx, PA3_13_ad, 2, tab_rq_registers);
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = PAUSE_fc;
	modbus_write_registers(ctx, PA3_14_ad, 2, tab_rq_registers);
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = PST_CANCEL_fc;
	modbus_write_registers(ctx, PA3_15_ad, 2, tab_rq_registers);
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = FREE_RUN_fc;
	modbus_write_registers(ctx, PA3_16_ad, 2, tab_rq_registers);


	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = CTRL_MOD_SLCT_fc;
	modbus_write_registers(ctx, PA3_21_ad, 2, tab_rq_registers);

	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = X1_fc;
	modbus_write_registers(ctx, PA3_22_ad, 2, tab_rq_registers);
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = X2_fc;
	modbus_write_registers(ctx, PA3_23_ad, 2, tab_rq_registers);
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = X3_fc;
	modbus_write_registers(ctx, PA3_24_ad, 2, tab_rq_registers);

	// [OUT] assign
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = S_RDY_fc;
	modbus_write_registers(ctx, PA3_56_ad, 2, tab_rq_registers);
}

//NOTES;
// The W type servo amplifier is capable of :
//		1. speed control and torque control with analog voltages.
//		2. position control with pulse.
//		3. positioning data operation with Di/Do signals or RS-485.(Selected)
//		4. immediate value data operation with RS-485.(Selected)
void positioning_data_operation_485_setting()
{
	// Positioning control with speed control, torque control, or pulse is used.
	// Enter "1" to "5" to PA1_01 for control mode selection.
	//	[Setting value]									[Control mode]
	//	    ×									    [Set OFF]	  |	   [Set ON]
	//		0											Position control
	//		1											   Speed control
	//		2											  Torque control
	//		3									Position control  |   Speed control
	//		4									Position control  |	 Torque control
	//		5									   Speed control  |	 Torque control
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = 0x0001;
	modbus_write_registers(ctx, PA1_01_ad,  2,  tab_rq_registers);
}

void immediate_value_date_operation_485_setting()
{
	// if Positioning operation, Enter 7 to PA1_01.
	// if use immediate value positioning operation, Enter 0 to PA2_40, means unusing internal positioning data.
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = 0x0007;
	modbus_write_registers(ctx, PA1_01_ad,  2,  tab_rq_registers);
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = 0x0000;
	modbus_write_registers(ctx, PA2_40_ad,  2,  tab_rq_registers);
}

