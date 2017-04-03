#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "alpha_setting.h"

modbus_t *ctx = NULL;

uint16_t *tab_rq_registers;
uint8_t  *tab_rq_bits;
uint16_t *tab_rp_registers; 
uint8_t	 *tab_rp_bits;

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

void close_modbus_rtu_master()
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

int init_parameters()
{
	int ret = 1;

	// PARAMETERS SETTINGS
	// [CONT]
	//tab_rq_registers[0] = 0x0000;
	//tab_rq_registers[1] = S_ON_fc;
	//rc = modbus_write_registers(ctx, PA3_9_ad,  2,  tab_rq_registers);
	//tab_rq_registers[0] = 0x0000;
	//tab_rq_registers[1] = FWD_fc;
	//rc = modbus_write_registers(ctx, PA3_10_ad, 2, tab_rq_registers);
	//tab_rq_registers[0] = 0x0000;
	//tab_rq_registers[1] = REV_fc;
	//rc = modbus_write_registers(ctx, PA3_11_ad, 2, tab_rq_registers);
	//tab_rq_registers[0] = 0x0000;
	//tab_rq_registers[1] = START_fc;
	//rc = modbus_write_registers(ctx, PA3_12_ad, 2, tab_rq_registers);

	//tab_rq_registers[0] = 0x0000;
	//tab_rq_registers[1] = CTRL_MOD_SLCT_fc;
	//rc = modbus_write_registers(ctx, PA3_21_ad, 2, tab_rq_registers);

	//tab_rq_registers[0] = 0x0000;
	//tab_rq_registers[1] = X1_fc;
	//rc = modbus_write_registers(ctx, PA3_22_ad, 2, tab_rq_registers);
	//tab_rq_registers[0] = 0x0000;
	//tab_rq_registers[1] = X2_fc;
	//rc = modbus_write_registers(ctx, PA3_23_ad, 2, tab_rq_registers);
	//tab_rq_registers[0] = 0x0000;
	//tab_rq_registers[1] = X3_fc;
	//rc = modbus_write_registers(ctx, PA3_24_ad, 2, tab_rq_registers);

	// [OUT]
	//tab_rq_registers[0] = 0x0000;
	//tab_rq_registers[1] = S_RDY_fc;
	//rc = modbus_write_registers(ctx, PA3_56_ad, 2, tab_rq_registers);
	return ret;
	
}
