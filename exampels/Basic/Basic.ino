#include <Arduino.h>

#include "esp32/ulp.h"
#include "driver/rtc_io.h"

#include <hulp.h>
#include "HULPx.h"


void ULP_PROG( void );


RTC_DATA_ATTR uint32_t pwm_var = 30;
RTC_DATA_ATTR uint32_t multi_var1 = 110;
RTC_DATA_ATTR uint32_t multi_var2 = 19;
RTC_DATA_ATTR uint32_t div_var1 = 508;
RTC_DATA_ATTR uint32_t div_var2 = 27;
RTC_DATA_ATTR uint32_t add_var1 = 406;
RTC_DATA_ATTR uint32_t add_var2 = 305;
RTC_DATA_ATTR uint32_t sub_var1 = 204;
RTC_DATA_ATTR uint32_t sub_var2 = 103;
RTC_DATA_ATTR uint32_t pct_var1 = 1002;
RTC_DATA_ATTR uint32_t pct_var2 = 31;

enum slow_memory {
		// 0: Start of RTC_SLOW_MEMORY.
		ulp_memory_start = 0x00,

		mul_result = ulp_memory_start,
		mul_ov,

		div_result,
		div_rest,

		add_result,
		add_ov,
		add_eq,

		sub_result,
		sub_ov,
		sub_eq,

		pct_result,
		pct_ov,

		ulp_memory_end = 0x7F,

		// 0x80 - 0x1FF is RTC_DATA_ATTR memory

		// ULP program
		ulp_prog_start = 0x200,				// ULP program start.

		ulp_prog_end = 0x7FF,				// End of ulp program.

		// Labels for ULP program.
		LBL_PROG_START = 0,

		LBL_PWM,
		LBL_PWM_ends = LBL_PWM + LBL_PWM_Count,

		LBL_LED_BLINK_START,
		LBL_LED_BLINK_END = LBL_LED_BLINK_START + LBL_PWM_Count,

		LBL_ADD_OV,
		LBL_SUB_OV,
		LBL_ADD_EQ,
		LBL_SUB_EQ,

	};


#define LED_PIN GPIO_NUM_4
#define PWM_PIN GPIO_NUM_27
#define TEST_PIN GPIO_NUM_2

void setup() {
	// For debug output
	Serial.begin(115200);

	ULP_PROG();

	delay( 100 );

	hulpx_Dump( ulp_memory_start, ulp_memory_end, ulp_memory_end );
	
} // setup()

void loop() {

} // loop()

void ULP_PROG( void )
{
	hulp_configure_pin( PWM_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 1 );
	hulp_configure_pin( LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0 );
	hulp_configure_pin( TEST_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0 );

	// ULP Program
	const ulp_insn_t ulp_prog[] = {

		M_LABEL( LBL_PROG_START ),

			M_UPDATE_TICKS(),		// Wait until ticks read is ready.

			// ! PWM
			I_MOVI( R3, 0 ),
			I_GET( R0, R3, pwm_var ),
			M_PWMR( LBL_PWM, TEST_PIN, 5 ),		// 5 mS PWM signal

			M_PWMI( LBL_LED_BLINK_START, LED_PIN, 5000, 10 ),	// Blink 4500-500 mS

			// ! ADD
			I_MOVI( R3, ulp_memory_start ),
			I_GET( R0, R3, add_var1 ),
			I_GET( R1, R3, add_var2 ),
			I_ADDR( R0, R0, R1 ),
			M_BXZ( LBL_ADD_EQ ),
			I_MOVI( R2, 0 ),
			I_ST( R2, R3, add_eq ),
		M_LABEL( LBL_ADD_EQ ),
			I_ST( R0, R3, add_result ),
			I_MOVI( R2, 1 ),
			M_BXF( LBL_ADD_OV ),
			I_MOVI( R2, 0 ),
		M_LABEL( LBL_ADD_OV ),
			I_ST( R2, R3, add_ov ),

			// ! SUB
			I_MOVI( R3, ulp_memory_start ),
			I_GET( R0, R3, sub_var1 ),
			I_GET( R1, R3, sub_var2 ),
			I_SUBR( R0, R0, R1 ),		// R0 = R0 - R1, EQ & OV set if needed
			M_BXZ( LBL_SUB_EQ ),
			I_MOVI( R2, 0 ),
			I_ST( R2, R3, sub_eq ),
		M_LABEL( LBL_SUB_EQ ),
			I_MOVI( R2, 1 ),
			I_ST( R0, R3, sub_result ),
			M_BXF( LBL_SUB_OV ),
			I_MOVI( R2, 0 ),
		M_LABEL( LBL_SUB_OV ),
			I_ST( R2, R3, sub_ov ),

			// ! MUL
			I_MOVI( R3, 0 ),
			I_GET( R0, R3, multi_var1 ),
			I_GET( R1, R3, multi_var2 ),
			M_JSRL( SUB_MULR, HULP_LBLA() ),	// R0 = R0 * R1, R1 = OV
			I_MOVI( R3, ulp_memory_start ),
			I_ST( R0, R3, mul_result ),
			I_ST( R1, R3, mul_ov ),

			// ! DIV
			I_MOVI( R3, 0 ),
			I_GET( R0, R3, div_var1 ),	// Dividend
			I_GET( R1, R3, div_var2 ),	// Divisor
			M_JSRL( SUB_DIVR, HULP_LBLA() ),	// R0 = R0 / R1, R1 = rest
			I_MOVI( R3, ulp_memory_start ),
			I_ST( R0, R3, div_result ),
			I_ST( R1, R3, div_rest ),
			
			// ! PCT
			I_MOVI( R3, 0 ),
			I_GET( R0, R3, pct_var1 ),
			I_GET( R1, R3, pct_var2 ),
			M_JSRL( SUB_PCTR, HULP_LBLA() ),	// R0 = R0 % R1, R1 = OV
			I_MOVI( R3, ulp_memory_start ),
			I_ST( R0, R3, pct_result ),
			I_ST( R1, R3, pct_ov ),

			I_HALT(),

			/* Sub Functions */
			M_PCTR( SUB_PCTR ),
			M_DIVR( SUB_DIVR ),
			M_MULR( SUB_MULR )
	};
	
	memset(RTC_SLOW_MEM, 0, (ulp_memory_end-ulp_memory_start)*sizeof(ulp_insn_t));		// ULP memory area only.

	size_t size = sizeof( ulp_prog ) / sizeof( ulp_insn_t );
	esp_err_t err = HULPx_process_macros_and_load( ulp_prog_start, ulp_prog, &size );

    ulp_set_wakeup_period( 0, 100 );
	ulp_run( ulp_prog_start );

} // ULP_PROG()

