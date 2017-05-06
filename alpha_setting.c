#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "elog.h"
#include "alpha_setting.h"


modbus_t *ctx = NULL;

// Buffers for modbus-RTU reply/query
uint16_t *tab_rq_registers = NULL;
uint8_t  *tab_rq_bits = NULL;
uint16_t *tab_rp_registers = NULL; 
uint8_t	 *tab_rp_bits = NULL;

// Parameter setting for io mapping
void io_signals_mapping();
// Set PA01_01=1	(speed mode with RS485 for speed selection)
void positioning_data_operation_485_setting();
// Set PA01_01=7, PA02_40=0 (immediate value operation mode)
void immediate_value_date_operation_485_setting();

// Init/Free bufffers for modbus-TRU
int init_buffers_for_modbus()
{
	tab_rq_registers = (uint16_t *) malloc(REGISTERS_BUFFER_SIZE * sizeof(uint16_t));
	memset(tab_rq_registers, 0, REGISTERS_BUFFER_SIZE * sizeof(uint16_t));
	tab_rp_registers = (uint16_t *) malloc(REGISTERS_BUFFER_SIZE * sizeof(uint16_t));
	memset(tab_rp_registers, 0, REGISTERS_BUFFER_SIZE  * sizeof(uint16_t));
	tab_rq_bits = (uint8_t *)malloc(BITS_BUFFER_SIZE * sizeof(uint8_t));
	memset(tab_rq_bits, 0, BITS_BUFFER_SIZE * sizeof(uint8_t));
	tab_rp_bits = (uint8_t *)malloc(BITS_BUFFER_SIZE * sizeof(uint8_t));
	memset(tab_rp_bits, 0, BITS_BUFFER_SIZE * sizeof(uint8_t));
	if(tab_rq_registers == NULL || tab_rp_registers == NULL || tab_rq_bits == NULL || tab_rp_bits == NULL) return -1;
	return 0;
}
void free_buffers_for_modbus()
{
	free(tab_rq_registers);
	free(tab_rp_registers);
	free(tab_rq_bits);
	free(tab_rp_bits);
	tab_rq_registers = NULL;
	tab_rp_registers = NULL;
	tab_rq_bits = NULL;
	tab_rp_bits = NULL;
}

// Open/Close modbus-RTU master
int open_modbus_rtu_master(const char *device, int baud, char parity, int data_bit, int stop_bit, int slave)
{
	int rc;
	ctx = modbus_new_rtu(device , baud, parity, data_bit, stop_bit);
	if(ctx == NULL){
		return -1;
	}
	// SLAVE_ID
	rc = modbus_set_slave(ctx,slave);
	if(-1 == rc){
		modbus_free(ctx);
		return -1;
	}
	// Debug mode
	//modbus_set_debug(ctx, TRUE);
	if (modbus_connect(ctx) == -1) {
		modbus_free(ctx);
		return -1;
	}
	return 0;
}
void close_modbus_rtu_master()
{
	modbus_close(ctx);
	modbus_free(ctx);
}

// Parameter setting for io mapping
void io_signals_mapping()
{
	// [CONT] mapping
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

	// [OUT] mapping
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = S_RDY_fc;
	modbus_write_registers(ctx, PA3_56_ad, 2, tab_rq_registers);
}
//*********************************************************************
// NOTES:
// The W type servo amplifier is capable of :
//		1. speed control and torque control with analog voltages.
//		2. position control with pulse.
//		3. positioning data operation with Di/Do signals or RS-485.
//		4. immediate value data operation with RS-485.(Selected)
//*********************************************************************
// Set PA01_01=1	(speed mode with RS485 for speed selection)
void positioning_data_operation_485_setting()
{
	//  Positioning control with speed control, torque control, or pulse is used.
	//  Enter "1" to "5" to PA1_01 for control mode selection.
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
// Set PA01_01=7, PA02_40=0 (immediate value operation mode)
void immediate_value_date_operation_485_setting()
{
	//  Positioning operation, Enter 7 to PA1_01.
	//  immediate value positioning operation, Enter 0 to PA2_40, means unusing internal positioning data.
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = 0x0007;
	modbus_write_registers(ctx, PA1_01_ad,  2,  tab_rq_registers);
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = 0x0000;
	modbus_write_registers(ctx, PA2_40_ad,  2,  tab_rq_registers);
	// Pulse size per circle
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = M_PULSE_PER_CIRCLE & 0xFFFF;
	modbus_write_registers(ctx, PA1_05_ad,  2,  tab_rq_registers);
	// PA1_06,07 default value 16,1
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = 16;
	modbus_write_registers(ctx, PA1_06_ad,  2,  tab_rq_registers);
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = 1;
	modbus_write_registers(ctx, PA1_07_ad,  2,  tab_rq_registers);
	// OT check
	const int32_t plus_position = M_PULSE_PER_CIRCLE * TRANSMISSION_RATIO;	// actual transmission ratio 252.5, for Pulse size 40000
	const int32_t minus_position = -M_PULSE_PER_CIRCLE * TRANSMISSION_RATIO;
	// PA1_01=7, PA2_25=0 for PTP mode, PA2_25=1 for INC mode and OT invalid
	tab_rq_registers[1] = 0x0000;
	tab_rq_registers[0] = 0x0000;											// PTP mode	
	modbus_write_registers(ctx, PA2_25_ad,  2,  tab_rq_registers);
	// +OT position > -OT position
	tab_rq_registers[1] = plus_position;
	tab_rq_registers[0] = plus_position >> 16;
	modbus_write_registers(ctx, PA2_26_ad,  2,  tab_rq_registers);
	tab_rq_registers[1] = minus_position;
	tab_rq_registers[0] = minus_position >> 16;
	modbus_write_registers(ctx, PA2_27_ad,  2,  tab_rq_registers);
	tab_rq_registers[1] = plus_position;
	tab_rq_registers[0] = plus_position >> 16;
	modbus_write_registers(ctx, PA2_28_ad,  2,  tab_rq_registers);
	tab_rq_registers[1] = minus_position;
	tab_rq_registers[0] = minus_position >> 16;
	modbus_write_registers(ctx, PA2_29_ad,  2,  tab_rq_registers);
}
void init_parameters()
{
	//positioning_data_operation_485_setting();
	immediate_value_date_operation_485_setting();
	io_signals_mapping();
}

int check_parameters()
{
	if(2 != modbus_read_registers(ctx, PA1_01_ad,  2,  tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != 7) return -1;
	
	if(2 != modbus_read_registers(ctx, PA2_40_ad,  2,  tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != 0) return -1;

	if(2 != modbus_read_registers(ctx, PA1_05_ad,  2,  tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != (M_PULSE_PER_CIRCLE&0xFFFF)) return -1;

	if(2 != modbus_read_registers(ctx, PA1_06_ad,  2,  tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != 16) return -1;

	if(2 != modbus_read_registers(ctx, PA1_07_ad,  2,  tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != 1) return -1;

	if(2 != modbus_read_registers(ctx, PA2_25_ad,  2,  tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != 0) return -1;
	
	// [CONT] mapping
	if(2 != modbus_read_registers(ctx, PA3_9_ad,  2,  tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != SERVO_ON_fc) return -1;

	if(2 != modbus_read_registers(ctx, PA3_10_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != FWD_fc) return -1;

	if(2 != modbus_read_registers(ctx, PA3_11_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != REV_fc) return -1;

	if(2 != modbus_read_registers(ctx, PA3_12_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != START_fc) return -1;

	if(2 != modbus_read_registers(ctx, PA3_13_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != EMG_fc) return -1;

	if(2 != modbus_read_registers(ctx, PA3_14_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != PAUSE_fc) return -1;

	if(2 != modbus_read_registers(ctx, PA3_15_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != PST_CANCEL_fc) return -1;

	if(2 != modbus_read_registers(ctx, PA3_16_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != FREE_RUN_fc) return -1;

	if(2 != modbus_read_registers(ctx, PA3_21_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != CTRL_MOD_SLCT_fc) return -1;

	if(2 != modbus_read_registers(ctx, PA3_22_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != X1_fc) return -1;

	if(2 != modbus_read_registers(ctx, PA3_23_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != X2_fc) return -1;

	if(2 != modbus_read_registers(ctx, PA3_24_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != X3_fc) return -1;

	// [OUT] mapping
	if(2 != modbus_read_registers(ctx, PA3_56_ad, 2, tab_rp_registers)) return -1;
	if(tab_rp_registers[0] != 0 || tab_rp_registers[1] != S_RDY_fc) return -1;

	return 0;
}
