/***********************************
 * @file	HULPx.cpp
 * @author	Gfy63 (mrgoofy@gmx.net)
 * 
 * GPLv3 Licence https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * @copyright 2024
 **********************************/

/****
 * ! test
 * * info
 * ? test
 * todo to do
 * //// delete
 * **/

/*----------------------------------
	INCLUDE
----------------------------------*/

#include "HULPx.h"

/*----------------------------------
	CONSTRUCTORS
----------------------------------*/

/*----------------------------------
	PRIVATE FUNCTIONS PROTOTYPE
----------------------------------*/

void hulpx_disassemblerPrint(int pos);

/*----------------------------------
	EXTERN PUBLIC VARIABELS
----------------------------------*/

/*----------------------------------
	PUBLIC VARIABELS
----------------------------------*/

/*----------------------------------
	PRIVATE VARIABELS
----------------------------------*/

// #define IS_DEBUG_DISASSEMBLER

#ifdef IS_DEBUG_DISASSEMBLER
	#define debugDisassmbler Serial.printf( "--- Code line: %4i ---\t", __LINE__ );
#else
	#define debugDisassmbler
#endif

const char* ulpRegStr[] = {
	"R0",
	"R1",
	"R2",
	"R3",
};

const char* ulpPeriphStr[] = {
	"RTC_CNTL",
	"RTC_IO",
	"SENS",
	"RTC_I2C"
};

const char* ulpSarStr[] = {
	"RTC_CNTL",
	"RTC_IO",
	"SENS",
	"RTC_I2C"
};

const char* ulpSignStr[] = {
	"+",
	"-",
};

const char* ulpCmpStr[] = {
	"LT",
	"GE",
	"LE"
};

const char* ulpCmpR0str[] = {
	"R0 < ",
	"R0 >= ",
	"R0 <="
};

const char* ulpCmpSstr[] = {
	"Scnt < ",
	"Scnt >= ",
	"Scnt <="
};

/*----------------------------------
	PUBLIC FUNCTION
----------------------------------*/

void hulpx_Dump( )
{
	hulpx_Dump( 0, 0, 0 );
}

void hulpx_Dump(int start )
{
	hulpx_Dump( start, 0, 0 );
}

void hulpx_Dump(int start, int end )
{
	hulpx_Dump( start, end, 0 );
}

void hulpx_Dump(int start, int end, int programAddr)
{
	Serial.printf("==   DISASSEMBLER DUMP   ==\n");

	if( end <= 0 ) end = 2047;

	for ( int i = start ; i <= end ; i++ ) {
		hulpx_disassemblerPrint(i);

		if( i < programAddr )
			continue;

		if( i < (2047-5) && RTC_SLOW_MEM[i+1] == 0 && RTC_SLOW_MEM[i+2] == 0 && RTC_SLOW_MEM[i+3] == 0 && RTC_SLOW_MEM[i+4] == 0 ) {
			break;
		}
	}
} // hulpx_Dump()

////////////////////////////////////

uint8_t HULPx_ms_to_ulp_tick_shift( uint32_t time_ms )
{
	uint16_t ms_ticks = hulp_ms_to_ulp_ticks( time_ms );
	uint16_t ms_tick_shift = hulp_ms_to_ulp_tick_shift( time_ms );

	if( ms_ticks > 32000 && ms_tick_shift < 32  )
		++ms_tick_shift;

	return ms_tick_shift;

} // HULPx_ms_to_ulp_tick_shift()

uint16_t HULPx_ms_to_ulp_ticks( uint32_t time_ms )
{
	uint16_t ms_ticks = hulp_ms_to_ulp_ticks( time_ms );
	uint16_t ms_tick_shift = hulp_ms_to_ulp_tick_shift( time_ms );

	if( ms_ticks > 32000 && ms_tick_shift < 32  )
		ms_ticks = ms_ticks / 2;

	return ms_ticks;

} // HULPx_ms_to_ulp_ticks()

////////////////////////////////////

/*----------------------------------
	PRIVATE FUNCTION
----------------------------------*/

void hulpx_disassemblerPrint(int pos) {
	ulp_insn_t *prog = (ulp_insn_t*)&RTC_SLOW_MEM[pos];

	if(prog->halt.opcode==0 && RTC_SLOW_MEM[pos] !=0){

		debugDisassmbler;
		Serial.printf("%04X : %08X DATA %5d\t\t\t\t// ST ADDR:0x%04X (0x%04X)\n",
			pos, 
			RTC_SLOW_MEM[pos], 
			(uint16_t)(RTC_SLOW_MEM[pos] & 0xFFFF), 
			RTC_SLOW_MEM[pos] >> 21,
			(uint16_t)(RTC_SLOW_MEM[pos] & 0xFFFF) ); 
		return;
	}

	Serial.printf("%04X : %08X PROG ", pos, RTC_SLOW_MEM[pos]);

	switch( prog->halt.opcode ){
		case 0:		// NOP
			debugDisassmbler;
			Serial.printf("NOP\t\t\t\t// NOP");
			break;

		case OPCODE_WR_REG:	// REG_WR
			debugDisassmbler;
			// Serial.printf("REG_WR 0x%04X, %3d, %3d, %3d\t// %s%0s_%02X[%d:%d] = %d", 
			Serial.printf("REG_WR 0x%04X, %3d, %3d, %3d\t// %s_0x%02X[%d:%d] = %d", 

				prog->wr_reg.addr, 
				prog->wr_reg.high, 
				prog->wr_reg.low, 
				prog->wr_reg.data, 
				ulpPeriphStr[prog->wr_reg.periph_sel], 
				prog->wr_reg.addr * sizeof(uint32_t), 
				prog->wr_reg.high, 
				prog->wr_reg.low, 
				prog->wr_reg.data );
			break;

		case OPCODE_RD_REG:		// REG_RD
			debugDisassmbler;
			Serial.printf("REG_RD 0x%04X, %3d, %3d\t\t// R0 = %s_0x%02X[%d:%d]", 
				prog->rd_reg.addr, 
				prog->rd_reg.high, 
				prog->rd_reg.low, 
				ulpPeriphStr[prog->rd_reg.periph_sel], 
				prog->rd_reg.addr * sizeof(uint32_t), 
				prog->rd_reg.high, 
				prog->rd_reg.low );
			break;
		
		case OPCODE_I2C:		// I2C
			if( prog->i2c.rw == 0 ){
				debugDisassmbler;
				Serial.printf("I2C_RD 0x%02X, %3d, %3d, 0x%02X", 
					prog->i2c.i2c_addr, 
					prog->i2c.high_bits, 
					prog->i2c.low_bits,
					prog->i2c.i2c_sel);
			} else {
				debugDisassmbler;
				Serial.printf("I2C_WR 0x%02X, 0x%02X, %3d, %3d, 0x%02X", 
					prog->i2c.i2c_addr, 
					prog->i2c.data, 
					prog->i2c.high_bits, 
					prog->i2c.low_bits, 
					prog->i2c.i2c_sel);
			}
			break;
		
		case OPCODE_DELAY:		// WAIT
			debugDisassmbler;
			Serial.printf("WAIT %3d\t\t\t\t// Delay %d", 
				prog->delay.cycles, 
				prog->delay.cycles );
			break;
		
		case OPCODE_ADC:		// ADC
			debugDisassmbler;
			Serial.printf("ADC %s, %s, %3d, %d", 
				ulpPeriphStr[prog->adc.dreg], 
				ulpSarStr[prog->adc.sar_sel], 
				prog->adc.mux, 
				prog->adc.cycles );
			break;
		
		case OPCODE_ST:			// ST
			debugDisassmbler;
			Serial.printf("ST %s, %s, %3d\t\t\t// MEM[%s+%d] = %s (0x%04X)", 
				ulpRegStr[prog->st.dreg], 
				ulpRegStr[prog->st.sreg], 
				prog->st.offset, 
				ulpRegStr[prog->st.sreg], 
				prog->st.offset, 
				ulpRegStr[prog->st.dreg],
				prog->st.offset );
			break;

		case OPCODE_ALU:	// ALU
			switch( prog->alu_reg.sel ) {
				case ALU_SEL_ADD:	// ADD
					switch( prog->alu_reg.sub_opcode ) {
						case SUB_OPCODE_ALU_REG:
							// reg
							debugDisassmbler;
							Serial.printf("ADD %s, %s, %s\t\t\t// %s = %s + %s", 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg], 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg]);
							break;
						
						case SUB_OPCODE_ALU_IMM:
							// imm
							debugDisassmbler;
							Serial.printf("ADD %s, %s, %3d\t\t\t// %s = %s + %d", 
								ulpRegStr[prog->alu_imm.dreg], 
								ulpRegStr[prog->alu_imm.sreg], 
								(int16_t)prog->alu_imm.imm, 
								ulpRegStr[prog->alu_imm.dreg], 
								ulpRegStr[prog->alu_imm.sreg], 
								(int16_t)prog->alu_imm.imm);
							break;

						case SUB_OPCODE_ALU_CNT:
							// imm
							debugDisassmbler;
							Serial.printf("STAGE_INC\t\t\t\t// Scnt = Scnt + 1" );
							break;

						default:				
							// unknown
							debugDisassmbler;
							Serial.printf("ADD unknown" );
							break;
					}
					break;

				case ALU_SEL_SUB:	// SUB
					switch( prog->alu_reg.sub_opcode ) {
						case SUB_OPCODE_ALU_REG:
							// reg
							debugDisassmbler;
							Serial.printf("SUB %s, %s, %s\t\t\t// %s = %s - %s",
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg], 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg]);
							break;

						case SUB_OPCODE_ALU_IMM:
							// imm
							debugDisassmbler;
							Serial.printf("SUB %s, %s, %3d\t\t\t// %s = %s - %d", 
								ulpRegStr[prog->alu_imm.dreg], 
								ulpRegStr[prog->alu_imm.sreg], 
								(int16_t)prog->alu_imm.imm, 
								ulpRegStr[prog->alu_imm.dreg], 
								ulpRegStr[prog->alu_imm.sreg], 
								(int16_t)prog->alu_imm.imm);
							break;

						case SUB_OPCODE_ALU_CNT:
							// imm
							debugDisassmbler;
							Serial.printf("STAGE_DEC\t\t\t\t// Scnt = Scnt - 1" );
							break;

						default:				
							// unknown
							debugDisassmbler;
							Serial.printf("ADD unknown" );
							break;
					}
					break;

				case ALU_SEL_AND:	// AND
					switch( prog->alu_reg.sub_opcode ) {
						case SUB_OPCODE_ALU_REG:
							// reg
							debugDisassmbler;
							Serial.printf("AND %s, %s, %s\t\t\t// %s = %s & %s", 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg], 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg]);
							break;

						case SUB_OPCODE_ALU_IMM:
							// imm
							debugDisassmbler;
							Serial.printf("AND %s, %s, %3d\t\t\t// %s = %s & %d", 
								ulpRegStr[prog->alu_imm.dreg], 
								ulpRegStr[prog->alu_imm.sreg], 
								(int16_t)prog->alu_imm.imm, 
								ulpRegStr[prog->alu_imm.dreg], 
								ulpRegStr[prog->alu_imm.sreg], 
								(int16_t)prog->alu_imm.imm);
							break;

						case SUB_OPCODE_ALU_CNT:
							// cnt
							debugDisassmbler;
							Serial.printf("STAGE_RST\t\t\t\t// Scnt = 0" );
							break;
						
						default:				
							// unknown
							debugDisassmbler;
							Serial.printf("SUB unknown" );
							break;
					}
					break;

				case ALU_SEL_OR:	// OR
					switch( prog->alu_reg.sub_opcode ) {
						case SUB_OPCODE_ALU_REG:
							// reg
							debugDisassmbler;
							Serial.printf("OR %s, %s, %s\t\t\t// %s = %s | %s", 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg], 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg]);
							break;
							
						case SUB_OPCODE_ALU_IMM:
							// imm
							debugDisassmbler;
							Serial.printf("OR %s, %s, %3d\t\t\t// %s = %s | %d", 
								ulpRegStr[prog->alu_imm.dreg], 
								ulpRegStr[prog->alu_imm.sreg], 
								(int16_t)prog->alu_imm.imm, 
								ulpRegStr[prog->alu_imm.dreg], 
								ulpRegStr[prog->alu_imm.sreg], 
								(int16_t)prog->alu_imm.imm);
							break;
							
						default:				
							// unknown
							debugDisassmbler;
							Serial.printf("OR unknown" );
							break;
					}
					break;
				
				case ALU_SEL_LSH:	// LSH
					switch( prog->alu_reg.sub_opcode ) {
						case SUB_OPCODE_ALU_REG:
							// reg
							debugDisassmbler;
							Serial.printf("LSH %s, %s, %s\t\t\t// %s = %s << %s", 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg], 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg]);
							break;
						
						case SUB_OPCODE_ALU_IMM:
							// imm
							debugDisassmbler;
							Serial.printf("LSH %s, %s, %3d\t\t\t// %s = %s << %d", 
							ulpRegStr[prog->alu_imm.dreg], 
							ulpRegStr[prog->alu_imm.sreg], 
							(int16_t)prog->alu_imm.imm, 
							ulpRegStr[prog->alu_imm.dreg], 
							ulpRegStr[prog->alu_imm.sreg], 
							(int16_t)prog->alu_imm.imm);
							break;

						default:				
							// unknown
							debugDisassmbler;
							Serial.printf("LSH unknown" );
							break;
					}
					break;

				case ALU_SEL_RSH:	// RSH
					switch( prog->alu_reg.sub_opcode ) {
						case SUB_OPCODE_ALU_REG:
							// reg
							debugDisassmbler;
							Serial.printf("RSH %s, %s, %s\t\t\t// %s = %s >> %s", 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg], 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.treg]);
							break;

						case SUB_OPCODE_ALU_IMM:
							// imm
							debugDisassmbler
							Serial.printf("RSH %s, %s, %3d\t\t\t// %s = %s >> %d", 
								ulpRegStr[prog->alu_imm.dreg], 
								ulpRegStr[prog->alu_imm.sreg], 
								(int16_t)prog->alu_imm.imm, 
								ulpRegStr[prog->alu_imm.dreg], 
								ulpRegStr[prog->alu_imm.sreg], 
								(int16_t)prog->alu_imm.imm);
							break;

						default:				
							// unknown
							debugDisassmbler;
							Serial.printf("LSH unknown" );
							break;
					}
					break;

				case ALU_SEL_MOV:	// MOVE
					switch( prog->alu_reg.sub_opcode ) {
						case SUB_OPCODE_ALU_REG:
							// reg
							debugDisassmbler;
							Serial.printf("MOVE %s, %s\t\t\t// %s = %s", 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg], 
								ulpRegStr[prog->alu_reg.dreg], 
								ulpRegStr[prog->alu_reg.sreg]);
							break;

						case SUB_OPCODE_ALU_IMM:
							// imm
							debugDisassmbler;
							Serial.printf("MOVE %s, %3u\t\t\t// %s = %u (0x%04X)", 
								ulpRegStr[prog->alu_imm.dreg], 
								(int16_t)prog->alu_imm.imm, 
								ulpRegStr[prog->alu_imm.dreg], 
								(int16_t)prog->alu_imm.imm,
								(int16_t)prog->alu_imm.imm);
							break;

						default:				
							// unknown
							debugDisassmbler;
							Serial.printf("MOVE unknown" );
							break;
					}
					break;
			}
			break;

		case OPCODE_BRANCH:		// JUML
			switch( prog->b.sub_opcode ) {
				case SUB_OPCODE_B:

			// if(prog->b.sub_opcode == SUB_OPCODE_B){
					// SUB_OPCODE_B(JUMP)
					debugDisassmbler;
					Serial.printf("JUMPR %s%3d, %3d, %s\t\t// IF %s%d THAN GOTO 0x%04X", 
						ulpSignStr[prog->b.sign], 
						prog->b.offset, 
						prog->b.imm, 
						ulpCmpStr[prog->b.cmp], 
						ulpCmpR0str[prog->b.cmp], 
						prog->b.imm, 
						prog->b.sign == 0 ? pos + prog->b.offset : pos - prog->b.offset);
					break;

				case SUB_OPCODE_BX:
			// } else {
					// SUB_OPCODE_BX(DIRECT JUMP)
					switch( prog->bx.type ) {

					// if( prog->bx.type == BX_JUMP_TYPE_DIRECT ){
						case BX_JUMP_TYPE_DIRECT:
							// BX_JUMP_TYPE_DIRECT
							if( prog->bx.reg == 1 ){
								// reg
								debugDisassmbler;
								Serial.printf("JUMP %s\t\t\t\t// GOTO %s", 
									ulpRegStr[prog->bx.dreg], 
									ulpRegStr[prog->bx.dreg]);
							} else {
								// imm
								debugDisassmbler;
								Serial.printf("JUMP 0x%04X\t\t\t// GOTO 0x%04X", 
									prog->bx.addr, 
									prog->bx.addr);
							}
							break;

						case BX_JUMP_TYPE_ZERO:
					// } else if( prog->bx.type == BX_JUMP_TYPE_ZERO ) {
							// EQ
							if( prog->bx.reg == 1 ){
								// reg
								debugDisassmbler;
								Serial.printf("JUMP %s, EQ\t\t\t// IF( EQ ) GOTO %s", 
									ulpRegStr[prog->bx.dreg],
									ulpRegStr[prog->bx.dreg]);
							} else {
								// imm
								debugDisassmbler;
								Serial.printf("JUMP 0x%04X, EQ\t\t\t// IF( EQ ) GOTO %04X",
									prog->bx.addr,
									prog->bx.addr);
							}
							break;

						case BX_JUMP_TYPE_OVF:
						// } else if( prog->bx.type == BX_JUMP_TYPE_OVF ){
							// OV
							if( prog->bx.reg == 1 ){
								// reg
								debugDisassmbler;
								Serial.printf("JUMP %s, OV\t\t\t// IF( OV ) GOTO %s",
									ulpRegStr[prog->bx.dreg],
									ulpRegStr[prog->bx.dreg]);
							} else {
								// imm
								debugDisassmbler;
								Serial.printf("JUMP 0x%04X, OV\t\t\t// IF( OV ) GOTO %04X",
									prog->bx.addr, 
									prog->bx.addr);
							}
							break;
						
						default:				
							// unknown
							debugDisassmbler;
							Serial.printf("JUMP unknown" );
							break;
					}
					break;

				case SUB_OPCODE_BS:
					// JUMPS
					debugDisassmbler;
					Serial.printf("JUMPS %s%3d, %3d, %s\t\t// IF %s%d THAN GOTO 0x%04X", 
						ulpSignStr[prog->bs.sign], 
						prog->bs.offset, 
						prog->bs.imm, 
						ulpCmpStr[prog->bs.cmp], 
						ulpCmpSstr[prog->bs.cmp], 
						prog->bs.imm, 
						prog->bs.sign == 0 ? pos + prog->bs.offset : pos - prog->bs.offset);
					break;
			}
			break;
		
		case OPCODE_END:		// WAKE
			switch( prog->end.sub_opcode ) {
				case SUB_OPCODE_END:
					debugDisassmbler;
					Serial.printf("WAKE\t\t\t\t// WAKE");
					break;

				case SUB_OPCODE_SLEEP:
					debugDisassmbler;
					Serial.printf("SLEEP %i\t\t\t\t// Select Sleep Register %i",
						prog->sleep.cycle_sel,
						prog->sleep.cycle_sel );
					break;
			}
			break;

		case OPCODE_TSENS:		// TSENS
			debugDisassmbler;
			Serial.printf("TSENS %s, %3d", 
				ulpRegStr[prog->tsens.dreg], 
				prog->tsens.wait_delay);
			break;
		
		case OPCODE_HALT:		// HALT
			debugDisassmbler;
			Serial.printf("HALT\t\t\t\t// HALT");
			break;
		
		case OPCODE_LD:			// LD
			debugDisassmbler;
			Serial.printf("LD %s, %s, %3d\t\t\t// %s = MEM[%s+%d] (0x%04X)", 
				ulpRegStr[prog->ld.dreg], 
				ulpRegStr[prog->ld.sreg], 
				prog->ld.offset, 
				ulpRegStr[prog->ld.dreg], 
				ulpRegStr[prog->ld.sreg], 
				prog->ld.offset,
				prog->st.offset );
			break;
		
		case OPCODE_MACRO:		// MACRO
			debugDisassmbler;
			Serial.printf("MACRO");
			break;

		default:				// Unknown
			debugDisassmbler;
			Serial.printf("OPCODE Unknown");
			break;
	}
	Serial.printf("\n");

} // hulpx_disassemblerPrint()

// End of "HULPx.cpp"
