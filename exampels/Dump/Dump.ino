#include <Arduino.h>

#include "esp32/ulp.h"
#include "driver/rtc_io.h"

#include <hulp.h>
#include "HULPx.h"

void ULP_disassemberTest();

void setup() {
	Serial.begin(115200);

	ULP_disassemberTest();

} // setup()

void loop() {

} // loop()

void ULP_disassemberTest()
{
	// Slow memory initialization
	memset(RTC_SLOW_MEM, 0, 0x7f * sizeof(ulp_insn_t) );		// ULP program area only.

	enum slow_memory {
		var_1,
		var_2,
		var_3,
		ulp_prog_start			// ULP program start.
	};

	enum lable_id {
		LBL_PROG_START,
		LBL_NEXT_RUN,
		LBL_1,
		LBL_2,
		LBL_3,
		LBL_4,
		LBL_5,
		LBL_PROG_END
	};

	// ULP Program
	const ulp_insn_t  ulp_prog[] = {

		M_LABEL( LBL_PROG_START ),
		M_LABEL( LBL_NEXT_RUN ),
			I_MOVI( R0, 0 ),
			I_MOVI( R1, 1 ),
			I_MOVI( R2, 2 ),
			I_MOVI( R3, 0 ),
			
			I_ADDR( R0, R1, R2 ),
			I_ADDI( R0, R1, 10 ),
			I_SUBR( R0, R1, R2 ),
			I_SUBI( R0, R1, 11 ),

			I_ANDR( R0, R1, R2 ),
			I_ANDI( R0, R1, 12 ),
			
			I_ORR( R0, R1, R2 ),
			I_ORI( R0, R1, 13 ),

			I_LSHR( R0, R1, R2 ),
			I_LSHI( R0, R1, 2 ),

			I_RSHR( R0, R1, R2 ),
			I_RSHI( R0, R1, 3 ),


			I_MOVR( R0, R1 ),
			I_MOVI( R0, 14 ),

			I_ST( R0, R3, var_1 ),
			I_LD( R0, R3, var_2 ),

			M_MOVL( R0, LBL_PROG_END ),
			I_BXR( R0 ),
			I_BXI( 10 ),

			I_BXZR( R0 ),
			I_BXZI( 11 ),
			I_BXFR( R0 ),
			I_BXFI( 12 ),

			I_BL( -5, 23 ),
			I_BGE( 5, 24 ),

			I_STAGE_RST(),
			I_STAGE_INC( 2 ),
			I_STAGE_DEC( 2 ),

			M_BSLT( LBL_3, 25 ),
			M_BSGE( LBL_4, 26 ),
			M_BSLE( LBL_5, 27 ),
			M_BSEQ( LBL_1, 28 ),
			M_BSGT( LBL_2, 29 ),

			I_JUMPS( LBL_3, 30, JUMPS_LT ),
			I_JUMPS( LBL_4, 32, JUMPS_GE ),
			I_JUMPS( LBL_5, 33, JUMPS_LE ),

			I_HALT(),
			I_END(),
			I_WAKE(),
			I_SLEEP_CYCLE_SEL( 0 ),
			I_DELAY( 10 ),

			I_TSENS( R0, 10 ),
			I_ADC( R0, 0, 1 ),

			I_I2C_READ( 0, 0 ),
			I_I2C_WRITE( 0, 0, 10 ),

			I_GPIO_SET_RD( ( GPIO_NUM_33) ),		/* R0 = pin state -> 0/1 */ \
			I_GPIO_SET( (GPIO_NUM_33), 1 ),			/* Pin = 0 */ \
			//I_WR_REG_BIT(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA_S + 0, 0),

			M_MOVL( R0, LBL_1 ),
		M_LABEL( LBL_1 ),

			M_BX( LBL_1 ),

			M_BL( LBL_2, 40 ),
		M_LABEL( LBL_2 ),
			M_BGE( LBL_3, 41 ),
		M_LABEL( LBL_3 ),
			M_BXZ( LBL_4 ),
		M_LABEL( LBL_4 ),
			M_BXF( LBL_5 ),
		M_LABEL( LBL_5 ),
			
			//M_SET_ENTRY_LBL( LBL_NEXT_RUN, ulp_prog ),
			//I_RD_REG( RTC_I2C_TIMEOUT_REG, 0, 19 ),

		M_LABEL( LBL_PROG_END ),

			I_HALT()
	};

	size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
	Serial.printf( "Prog Size: %x\n", size );

	ulp_process_macros_and_load( ulp_prog_start, ulp_prog, &size );

	hulpx_Dump( 0, 0x7F, ulp_prog_start );

} // ULP_disassemberTest()

