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
#ifndef ALPHA_SETTING_H
#define ALPHA_SETTING_H

#include "modbus.h"

// resent numbers when failes.
#define OPLOOPS	2

// REGISTER ADDRESS:[for parameter setting]
#define     PA1_01_ad                       0x4000
#define     PA2_40_ad                       0x4127

#define     PA3_9_ad                        0x4208
#define     PA3_10_ad                       0x4209
#define     PA3_11_ad                       0x420A
#define     PA3_12_ad                       0x420B
#define     PA3_13_ad                       0x420C
#define     PA3_14_ad                       0x420D
#define     PA3_15_ad                       0x420E
#define     PA3_16_ad                       0x420F
#define     PA3_17_ad                       0x4210
#define     PA3_18_ad                       0x4211
#define     PA3_19_ad                       0x4212
#define     PA3_20_ad                       0x4213
#define     PA3_21_ad                       0x4214
#define     PA3_22_ad                       0x4215
#define     PA3_23_ad                       0x4216
#define     PA3_24_ad                       0x4217

#define     PA3_56_ad                       0x4237
#define     PA3_57_ad                       0x4238
#define     PA3_58_ad                       0x4239
#define     PA3_59_ad                       0x423A
#define     PA3_60_ad                       0x423B
#define     PA3_61_ad                       0x423C
#define     PA3_62_ad                       0x423D
#define     PA3_63_ad                       0x423E
#define     PA3_64_ad                       0x423F
#define     PA3_65_ad                       0x4240
#define     PA3_66_ad                       0x4241
#define     PA3_67_ad                       0x4242
#define     PA3_68_ad                       0x4243
#define     PA3_69_ad                       0x4244
#define     PA3_70_ad                       0x4245
#define     PA3_71_ad                       0x4246

#define     IMME_VLU_STATUS_ad              0x5100
#define     IMME_VLU_POSITION_ad            0x5101
#define     IMME_VLU_SPEED_ad               0x5102
#define     IMME_VLU_ACC_TIM_ad             0x5103
#define     IMME_VLU_DEC_TIM_ad             0x5104

// COIL ADDRESS:[for controlling]                                       [Setting Register]
#define     CONT1_ad                        0x0400                      //PA3_01
#define     CONT2_ad                        0x0401                      //PA3_02
#define     CONT3_ad                        0x0402                      //PA3_03
#define     CONT4_ad                        0x0403                      //PA3_04
#define     CONT5_ad                        0x0404                      //PA3_05

#define     CONT9_ad                        0x0208                      //PA3_09
#define     CONT10_ad                       0x0209                      //PA3_10
#define     CONT11_ad                       0x020A                      //PA3_11
#define     CONT12_ad                       0x020B                      //PA3_12
#define     CONT13_ad                       0x020C                      //PA3_13
#define     CONT14_ad                       0x020D                      //PA3_14
#define     CONT15_ad                       0x020E                      //PA3_15
#define     CONT16_ad                       0x020F                      //PA3_16
#define     CONT17_ad                       0x0210                      //PA3_17
#define     CONT18_ad                       0x0211                      //PA3_18
#define     CONT19_ad                       0x0212                      //PA3_19
#define     CONT20_ad                       0x0213                      //PA3_20
#define     CONT21_ad                       0x0214                      //PA3_21
#define     CONT22_ad                       0x0215                      //PA3_22
#define     CONT23_ad                       0x0216                      //PA3_23
#define     CONT24_ad                       0x0217                      //PA3_24

#define     OUT1_ad                         0x0500                      //PA3_51
#define     OUT2_ad                         0x0501                      //PA3_52
#define     OUT3_ad                         0x0502                      //PA3_53

#define     OUT6_ad                         0x0305                      //PA3_56
#define     OUT7_ad                         0x0306                      //PA3_57
#define     OUT8_ad                         0x0307                      //PA3_58
#define     OUT9_ad                         0x0308                      //PA3_59
#define     OUT10_ad                        0x0309                      //PA3_60
#define     OUT11_ad                        0x030A                      //PA3_61
#define     OUT12_ad                        0x030B                      //PA3_62
#define     OUT13_ad                        0x030C                      //PA3_63
#define     OUT14_ad                        0x030D                      //PA3_64
#define     OUT15_ad                        0x030E                      //PA3_65
#define     OUT16_ad                        0x030F                      //PA3_66
#define     OUT17_ad                        0x0310                      //PA3_67
#define     OUT18_ad                        0x0311                      //PA3_68
#define     OUT19_ad                        0x0312                      //PA3_69
#define     OUT20_ad                        0x0313                      //PA3_70
#define     OUT21_ad                        0x0314                      //PA3_71


// Function Code:
// OUT Signals:
#define     SERVO_ON_fc                     1
#define     FWD_fc                          2
#define     REV_fc                          3
#define     START_fc                        4
#define     ORG_fc                          5
#define     LS_fc                           6
#define     OT_PLUS_fc                      7
#define     OT_MINUS_fc                     8
#define     EMG_fc                          10
#define     RST_fc                          11
#define     ACCO_fc                         14
#define     PST_PRESET_fc                   16
#define     GAIN_SWITCH_fc                  17
#define     TRQ_LMT0_fc                     19
#define     TRQ_LMT1_fc                     20
#define     IMME_VLU_CTINU_fc               22
#define     IMME_VLU_CHG_fc                 23
#define     ELCT_GEAR_NUM_SEL0_fc           24
#define     ELCT_GEAR_NUM_SEL1_fc           25
#define     CMD_PULSE_INHIBIT_fc            26
#define     CMD_PULSE_RTO1_fc               27
#define     COM_PULSE_RTO2_fc               28
#define     PROPORTIONAL_CTRL_fc            29
#define     PAUSE_fc                        31
#define     PST_CANCEL_fc                   32
#define     EXTR_REGN_RESIST_OH_fc          34
#define     TEACHING_fc                     35
#define     CTRL_MOD_SLCT_fc                36
#define     PST_CTRL_fc                     37
#define     TRQ_CTRL_fc                     38
#define     OVERRIDE_EN_fc                  43
#define     OVERRIDE1_fc                    44
#define     OVERRIDE2_fc                    45
#define     OVERRIDE4_fc                    46
#define     OVERRIDE8_fc                    47
#define     INTRUPT_IN_EN_fc                48
#define     INTRUPT_IN_fc                   49
#define     DEVIATION_CLR_fc                50
#define     X1_fc                           51
#define     X2_fc                           52
#define     X3_fc                           53
#define     FREE_RUN_fc                     54
#define     EDT_PERMISSION_fc               55
#define     ANTI_FREQ_SLCT0_fc              57
#define     ANTI_FREQ_SLCT1_fc              58
#define     AD0_fc                          60
#define     AD1_fc                          61
#define     AD2_fc                          62
#define     AD3_fc                          63
#define     PST_DATA_SLCT_fc                77
#define     BRDCST_CANCEL_fc                78

// IN Signals:
#define     RDY_fc                          1
#define     INP_fc                          2
#define     SPD_LMT_DETC_fc                 11
#define     OVER_WR_CMPL_fc                 13
#define     BRK_TIMING_fc                   14
#define     ALRM_DECT_fc                    16
#define     POINT_DETC_AREA1_fc             17
#define     POINT_DETC_AREA2_fc             18
#define     LMTER_DETC_fc                   19
#define     OT_DETC_fc                      20
#define     CYC_END_DETC_fc                 21
#define     HMING_CMPL_fc                   22
#define     ZRO_DEVI_fc                     23
#define     ZRO_SPEED_fc                    24
#define     SPD_COINCD_fc                   25
#define     TRQ_LMT_DETC_fc                 26
#define     ORLD_WRNING_fc                  27
#define     S_RDY_fc                        28
#define     ED_PRMSION_RSP_fc               29
#define     DATA_ERROR_fc                   30
#define     ADDR_ERROR_fc                   31
#define     ALRM_CODE0_fc                   32
#define     ALRM_CODE1_fc                   33
#define     ALRM_CODE2_fc                   34
#define     ALRM_CODE3_fc                   35
#define     ALRM_CODE4_fc                   36
#define     OT_PLUS_DETC_fc                 38
#define     OT_MINUS_DETC_fc                39
#define     HM_PST_LS_DETC_fc               40
#define     FRC_STP_DETC_fc                 41
#define     BATRY_WRNING_fc                 45
#define     LIFE_WRNING_fc                  46
#define     MD0_fc                          60
#define     MD1_fc                          61
#define     MD2_fc                          62
#define     MD3_fc                          63
#define     MD4_fc                          64
#define     MD5_fc                          65
#define     MD6_fc                          66
#define     MD7_fc                          67
#define     PST_PRST_CMPL_fc                75
#define     ALRM_DETC_fc                    76
#define     IMME_VLU_CNTIN_PERMS_fc         79
#define     IMME_VLU_CNTIN_CMPL_fc          80
#define     IMME_VLU_CHG_CMPL_fc            81
#define     CMD_PST_CMPL_fc                 82
#define     RANGE1_OF_PST_fc                83
#define     RANGE2_OF_PST_fc                84
#define     INTRUPT_PST_DECT_fc             85
#define     CONT_A_THROUGH_fc               91
#define     CONT_B_THROUGH_fc               92
#define     CONT_C_THROUGH_fc               93
#define     CONT_D_THROUGH_fc               94
#define     CONT_E_THROUGH_fc               95

// Signal Control Address:
// IN Signals:
#define     SERVO_ON_ad                     CONT9_ad
#define     FWD_ad                          CONT10_ad
#define     REV_ad                          CONT11_ad
#define     START_ad                        CONT12_ad
#define     ORG_ad                          
#define     LS_ad                           
#define     OT_PLUS_ad                      
#define     OT_MINUS_ad                     
#define     EMG_ad                          CONT13_ad
#define     RST_ad                          
#define     ACCO_ad                         
#define     PST_PRESET_ad                   
#define     GAIN_SWITCH_ad                  
#define     TRQ_LMT0_ad                     
#define     TRQ_LMT1_ad                     
#define     IMME_VLU_CTINU_ad               
#define     IMME_VLU_CHG_ad                 
#define     ELCT_GEAR_NUM_SEL0_ad           
#define     ELCT_GEAR_NUM_SEL1_ad           
#define     CMD_PULSE_INHIBIT_ad            
#define     CMD_PULSE_RTO1_ad               
#define     COM_PULSE_RTO2_ad               
#define     PROPORTIONAL_CTRL_ad            
#define     PAUSE_ad                        CONT14_ad
#define     PST_CANCEL_ad                   CONT15_ad
#define     EXTR_REGN_RESIST_OH_ad          
#define     TEACHING_ad                     
#define     CTRL_MOD_SLCT_ad                CONT21_ad
#define     PST_CTRL_ad                     
#define     TRQ_CTRL_ad                     
#define     OVERRIDE_EN_ad                  
#define     OVERRIDE1_ad                    
#define     OVERRIDE2_ad                    
#define     OVERRIDE4_ad                    
#define     OVERRIDE8_ad                    
#define     INTRUPT_IN_EN_ad                
#define     INTRUPT_IN_ad                   
#define     DEVIATION_CLR_ad                
#define     X1_ad                           CONT22_ad
#define     X2_ad                           CONT23_ad
#define     X3_ad                           CONT24_ad
#define     FREE_RUN_ad                     CONT16_ad
#define     EDT_PERMISSION_ad               
#define     ANTI_FREQ_SLCT0_ad              
#define     ANTI_FREQ_SLCT1_ad              
#define     AD0_ad                          
#define     AD1_ad                          
#define     AD2_ad                          
#define     AD3_ad                          
#define     PST_DATA_SLCT_ad                
#define     BRDCST_CANCEL_ad                

// OUT Signals:
#define     RDY_ad                          OUT1_ad
#define     INP_ad                          OUT2_ad
#define     SPD_LMT_DETC_ad                 
#define     OVER_WR_CMPL_ad                 
#define     BRK_TIMING_ad                   
#define     ALRM_DECT_ad                    OUT3_ad
#define     POINT_DETC_AREA1_ad             
#define     POINT_DETC_AREA2_ad             
#define     LMTER_DETC_ad                   
#define     OT_DETC_ad                      
#define     CYC_END_DETC_ad                 
#define     HMING_CMPL_ad                   
#define     ZRO_DEVI_ad                     
#define     ZRO_SPEED_ad                    
#define     SPD_COINCD_ad                   
#define     TRQ_LMT_DETC_ad                 
#define     ORLD_WRNING_ad                  
#define     S_RDY_ad                        OUT6_ad
#define     ED_PRMSION_RSP_ad               
#define     DATA_ERROR_ad                   
#define     ADDR_ERROR_ad                   
#define     ALRM_CODE0_ad                   
#define     ALRM_CODE1_ad                   
#define     ALRM_CODE2_ad                   
#define     ALRM_CODE3_ad                   
#define     ALRM_CODE4_ad                   
#define     OT_PLUS_DETC_ad                 
#define     OT_MINUS_DETC_ad                
#define     HM_PST_LS_DETC_ad               
#define     FRC_STP_DETC_ad                 
#define     BATRY_WRNING_ad                 
#define     LIFE_WRNING_ad                  
#define     MD0_ad                          
#define     MD1_ad                          
#define     MD2_ad                          
#define     MD3_ad                          
#define     MD4_ad                          
#define     MD5_ad                          
#define     MD6_ad                          
#define     MD7_ad                          
#define     PST_PRST_CMPL_ad                
#define     ALRM_DETC_ad                    
#define     IMME_VLU_CNTIN_PERMS_ad         
#define     IMME_VLU_CNTIN_CMPL_ad          
#define     IMME_VLU_CHG_CMPL_ad            
#define     CMD_PST_CMPL_ad                 
#define     RANGE1_OF_PST_ad                
#define     RANGE2_OF_PST_ad                
#define     INTRUPT_PST_DECT_ad             
#define     CONT_A_THROUGH_ad               
#define     CONT_B_THROUGH_ad               
#define     CONT_C_THROUGH_ad               
#define     CONT_D_THROUGH_ad               
#define     CONT_E_THROUGH_ad               

#define		REGISTERS_BUFFER_SIZE	12
#define		BITS_BUFFER_SIZE		24

extern modbus_t *ctx;

extern uint16_t *tab_rq_registers;
extern uint8_t  *tab_rq_bits;
extern uint16_t *tab_rp_registers; 
extern uint8_t	 *tab_rp_bits;


// Init parameters when reset the system.
// note: it just need once.
void init_parameters();

// Init recv(reply)/sent(query) buffers for modbus communication
// note: return 0 if successful, -1 for fails.
int init_buffers_for_modbus();
// Free(anti-init) buffers initialed buffers
void free_buffers_for_modbus();

// Open modbus
// note: return 0 if successful, -1 for fails.
int open_modbus_rtu_master(const char *device, int baud, char parity, int data_bit, int stop_bit, int slave);
// Close modbus
void close_modbus_rtu_master();


#endif	//ALPHA_GLOBAL_SETTING_H
