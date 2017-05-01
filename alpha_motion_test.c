#include <stdio.h>
#include "modbus.h"
#include "alpha_setting.h"
#include "alpha_motion_control.h"


void forward_command_test()
{
	// Control Mode selection ON
	// **NoTE: PA1_01 must be set 1
	while(1 != modbus_write_bit(ctx,CTRL_MOD_SLCT_ad,1)){ }
	// Set [FWD] mode through [X3] [X2] [X1]
	while(1 != modbus_write_bit(ctx,X3_ad,0)){ }
	while(1 != modbus_write_bit(ctx,X2_ad,0)){ }
	while(1 != modbus_write_bit(ctx,X1_ad,1)){ }


	//fprintf(stderr,"\n");
	//// Read OUTs
	//rc = modbus_read_bits(ctx,0x0500,1,tab_rp_bits);
	//rc = modbus_read_bits(ctx,0x0501,1,tab_rp_bits);
	//rc = modbus_read_bits(ctx,0x0502,1,tab_rp_bits);

	//fprintf(stderr,"\n");
	//rc = modbus_read_bits(ctx,0x0208,1,tab_rp_bits);
	//rc = modbus_read_bits(ctx,0x0209,1,tab_rp_bits);
	//rc = modbus_read_bits(ctx,0x020A,1,tab_rp_bits);
	//rc = modbus_read_bits(ctx,0x020B,1,tab_rp_bits);

	//fprintf(stderr,"\n");
	//rc = modbus_read_bits(ctx,0x0215,1,tab_rp_bits);
	//rc = modbus_read_bits(ctx,0x0216,1,tab_rp_bits);
	//rc = modbus_read_bits(ctx,0x0217,1,tab_rp_bits);


	// Set [FWD] ON to run 
	while(1 != modbus_write_bit(ctx,FWD_ad,1)){ }
	fprintf(stderr,"press Enter to stop.\n");
	getchar();
	// Set [FWD] OFF to stop
	while(1 != modbus_write_bit(ctx,FWD_ad,0)){ }
}


void immediate_value_data_op_test()
{
	// **NoTE: PA1_01 must be set 7
	// SETTING 1
	// Designation method = ABS. 
	// Immediate value position = 500,000 units.	Immediate value speed = 500.00 r/min
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = 0x0000;
	while(2 != modbus_write_registers(ctx,0x5100,2,tab_rq_registers)){ }
	tab_rq_registers[0] = 0x0007;
	tab_rq_registers[1] = 0xA120;
	while(2 != modbus_write_registers(ctx,0x5101,2,tab_rq_registers)){ }
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = 0xC350;
	while(2 != modbus_write_registers(ctx,0x5102,2,tab_rq_registers)){ }

	// set [START] ON
	while(1 != modbus_write_bit(ctx,0x020B,1)){ }
	// set [START] OFF
	while(1 != modbus_write_bit(ctx,0x020B,0)){ }

	// SETTING 2
	// Immediate value position = -100,000 units.	Immediate value speed = 200.00 r/min
	tab_rq_registers[0] = 0xFFFE;
	tab_rq_registers[1] = 0x7960;
	while(2 != modbus_write_registers(ctx,0x5101,2,tab_rq_registers)){ }
	tab_rq_registers[0] = 0x0000;
	tab_rq_registers[1] = 0x4E20;
	while(2 != modbus_write_registers(ctx,0x5102,2,tab_rq_registers)){ }

	// Check [INP]=1, means immediate value data operation is finished.
	tab_rp_bits[0] = 0;
	for(;;){
		while(1 != modbus_read_bits(ctx,0x0501,1,tab_rp_bits)){ }
		if(1 == tab_rp_bits[0]) break;
	}

	
	// set [START] ON
	while(1 != modbus_write_bit(ctx,0x020B,1)){ }
	// set [START] OFF
	while(1 != modbus_write_bit(ctx,0x020B,0)){ }

	// Check [INP]=1, means immediate value data operation is finished.
	tab_rp_bits[0] = 0;
	for(;;){
		while(1 != modbus_read_bits(ctx,0x0501,1,tab_rp_bits)){ }
		if(1 == tab_rp_bits[0]) break;
	}
}
