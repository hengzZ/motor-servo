
/// *************************************  Read Following Notes before Coding ******************************************************
///CONT/OUT PARAMETER SETTING (I/O)
// [Resource]:	[I/O Signals]
//				[input  signals] 1~78	factory default:	 1 [S-ON] assigned to PA03_01 CONT1; 11 [RST] assigned to PA03_02 CONT2;
//				[output signals] 1~95	factory default:	 1  [RDY] assigned to PA03_51  OUT1;  2 [INP] assigned to PA03_52  OUT2;
// 										factory default:	76 [alarm detection] assigned to PA03_53 OUT3;
// [Resource]:	[Parameter] 
//				Description: Parameters of the servo amplifier are divided into the following setting items according to the function
//				WARNING: 	 The write enable frequency of EEPROM is about 100,000 cycles:
//							 `Parameter editing
//							 `Position preset of absolute position system
//							 `Batch transfer of parameters
//				[Division]:	 1. [Basic Parameters]															[PA1_01 to 50]
//							 - Be sure to check or enter these parameters before STARATING OPERATION.
//							 2. [Control gain and filter setting parameter]									[PA1_51 to 99]
//							 - Use to adjust the gain manually.
//							 3. [Automatic operation setting parameter]										[PA2_01 to 50]
//							 - Use to enter or change the positioning operation speed and homing function.
//							 4. [Extended function setting parameter]										[PA2_51 to 99]
//							 - Use to enter or change the extended functions such as the torque limit.
//							 5. [Input terminal function setting parameter]									[PA3_01 to 50]
//							 - Use to enter or change input signals of the servo amplifier.
//							 6. [Output terminal function setting parameter]								[PA3_51 to 99]
// [Resource]: [Operation]
//			   [Priority among Input Signals]				* Please consult the manual.
//			   [Selection of Operation Procedure]			* Please consult the manual.
//			   [Operation Check]/[Check before Operation]	* Please consult the manual.
//			   WARNING:		 1. [Communications Timings]
//							 - set time T1 through PA2_94.
//							 - if time T1 specified longer than T0, actual response time is specified T1.
//							 2. [Communication Time Over]
//							 - communication time over is detected if any time other than 0.00s is set on PA2_95.
//							 - **Note: if a communication time over has occured, \
//							 - ** all the communication CONT signals(CONT9-24) operated by the Modbus communications are set off.
//			   [Operation]	 1. [First Test Operation at Keypad]
//							 2. [Position Control (Pulse)]
//							 3. [Speed Control]
//							 4. [Torque Control]
//							 5. [Mode Selection]
//							 6. [Extension Mode]
//							 7. [Homing]
//							 8. [Interrupt Positioning]
//							 9. [Torque Limit]
//							 10.[Positioning Data Operation]
//							 11.[Immediate Value Data Operation]
//							 12.[Interrupting/Stopping Operation]
/// *****************************************  Read Above Notes before Coding ******************************************************
/// *****************************************    [ I/O Signals Assignment ]   ******************************************************
//  note: [Communication CONT/OUT signals (register)]		CONT - 0x0000	OUT - 0x0100
//  	  [Parameter (register)]							PA1_1-99 - 0x4000-4062	PA2_1-99 - 0x4100-4162	PA3_1-99 - 0x4200-4262
//		  [CONT/OUT signals (coil)]							CONT9-24 - 0x0208-0217	OUT6-21  - 0x0305-0314	
//															CONT1-5  - 0x0400-0404	OUT1-3   - 0x0500-0502
//		  ** PA3_51-71 to set OUT1-21 's signal function
//		  ** PA3_01-24 to set CONT1-24 's signal function
//	[OUT]:
//			`Use factory default:	PA3_51  OUT1	1	[RDY]	|	PA3_52	OUT2	2	[INP]	|	PA3_53  OUT3   76	[Alarm Detection]
//			`Setting:				PA3_56	OUT6	28	[S-RDY]	|	
//	[CONT]:
//			`Not use factory default.
//			`Setting:				PA3_09	CONT9	1	[S-ON]	|	PA3_10	CONT10	2	[FWD]	|	PA3_11	CONT11	3	[REV]
//									PA3_12	CONT12	4	[START]	|	
//									PA3_22	CONT22	51	[X1]	|	PA3_23	CONT23	52	[X2]	|	PA3_24	CONT24	53	[X3]
/// ********************************************************************************************************************************
