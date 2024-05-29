/***********************************
 * @file	HULPx.h
 * @author	Gfy63 (mrgoofy@gmx.net)
 * @brief	Extention for HULP https://github.com/boarchuz/HULP.git
 * 			Also copy and complete misssing code from UlpDebug https://github.com/tanakamasayuki/UlpDebug.git
 *			This provide additional functions:
 *				- HULPX_GPIO_PWM_MS()	PWM generator on one GPIO in mSec.
 *				- HULPX_MULTIR()		Multiplication as subfunction.
 *				- HULPX_DIVR()			Division as subfunction.
				- hulpx_Dump()			Print slow memory and disassembel.
 * 
 * @version 0.0.1
 * @date 2024-05-08
 * 
 * GPLv2 Licence https://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 * 
 * @copyright 2024
 **********************************/

#ifndef HULPX_H
#define HULPX_H

	/*----------------------------------
		INCLUDE
	----------------------------------*/

	#include <Arduino.h>

	#include <hulp.h>
	#include "esp32/ulp.h"

	/*----------------------------------
		PUBLIC FUNCTIONS PROTOTYPE
	----------------------------------*/
	
	/**
	 * Disassembel the code in the ULP memory.
	 * 
	 * start		- Start address to disassembel. (optional: = 0)
	 * end			- End address to disassembel or 0 -> to 5 NOP (optional: = 2047)
	 * progAddr		- Start of the ULP code. (optional: =0)
	*/
	void hulpx_Dump(int start, int end, int programAddr);
	void hulpx_Dump(int start, int end);
	void hulpx_Dump(int start );
	void hulpx_Dump( void );

	/**
 	 * Get the optimal bit shift for a given time (in milliseconds), to be used in ranged 16-bit reading of RTC ticks. 
	 * The ticks count is limited to 32000.
	*/
	uint8_t HULPx_ms_to_ulp_tick_shift( uint32_t time_ms );

	/**
	 * Convert a time (in milliseconds) to an optimally-shifted 16-bit RTC tick count.
	 * The ticks count is limited to 32000.
	*/
	uint16_t HULPx_ms_to_ulp_ticks( uint32_t time_ms );

	/**
	 * Same as process_macros_and_load() but no limitation on program size.
	*/
	esp_err_t HULPx_process_macros_and_load(uint32_t load_addr, const ulp_insn_t* program, size_t* psize);

////////////////////////////////////

	// Reserved label for functions. 50000 - 59999
	#define LBL_MULR_start			50000
	#define LBL_DIVR_start			50020
	#define LBL_PCTR_start			50040

////////////////////////////////////

	/**
	 * Load line address in register.
	 * Use with HULP_LBLA().
	 * Exp:
	 * 		M_LBLR( R3, HULP_LBLA() )
	*/
	// #define M_LBLR( Rdst, lbl ) \
	// 	M_LABEL( lbl ), \
	// 		M_MOVL( Rdst, lbl )

////////////////////////////////////

	/**
	 * Jump to subroutine via label.
	 * 
	 * Example:
	 * 	M_JSRL( SUB_MULR, HULP_LBLA() )
	 * 
	 * Register:
	 * 	R2 			- Return address from subroutine.
	 * 
	 * Parameter:
	 *  sub_lbl		- Label to subroutine.
	 *  return_lbl	- Label for return from subroutine.
	*/
	#define M_JSRL( sub_lbl, return_lbl ) \
			M_MOVL( R2, return_lbl ), \
			M_BX( sub_lbl ), \
		M_LABEL( return_lbl )

////////////////////////////////////

	/**
	 * Jump to subroutine via address pointer.
	 * 
	 * Example:
	 * 	M_JSRP( SUB_MULR. HULP_LBLA() )
	 * 
	 * Register:
	 * 	R2 		- Return address from subroutine.
	 *  R3		- Index pointer for calling.
	 * 
	 * Parameter:
	 *  addrPnt_sub		- Address pointer to subroutine.
	 *  return_lbl		- Label for return from subroutine.
	*/
	#define M_JSRP( addrPnt_sub, return_lbl ) \
			I_MOVI( R3, 0 ), \
			I_LD( R3, R3, addrPnt_sub ), \
			M_MOVL( R2, return_lbl ), \
			I_BXR( R3 ), \
		M_LABEL( return_lbl )

////////////////////////////////////

	enum mulr_data_list
	{
		// Variable intern for M_MULR.
		VAR_MULR_return = 0,		// Return address to subroutine caller.

		// Labels for M_MULR.
		SUB_MULR = LBL_MULR_start,
		LBL_MULR_nextMulti,
		LBL_MULR_OV,
		LBL_MULR_noAdd,
		LBL_MULR_doneMulti,
		LBL_MULR_return,
		LBL_MULR_variables
	};

	/**
	 * Multiplication. R0 = R0 * R1
	 * Use SUB_MULR to call it.
	 * All Registers are used.
	 * 
	 * Exp:
	 * 	M_MOVL( R0, 100 ),
	 * 	M_MOVL( R1, 25 ),
	 * 	M_JSRL( SUB_MULR, HULP_LBLA() ),
	 *  I_HALT(),
	 *  M_MULR( SUB_MULR ),
	 * 
	 * Parameter:
	 * 	R0	- Multiplier.
	 *	R1	- Multiplicator.
	 *	R2	- Return address from this function.
	 * 
	 * Return:
	 *	R0	- Product.
	 *	R1	- 1 if OV, Product = 0
	*/
	#define M_MULR( lbl_mulr ) \
		M_LABEL( lbl_mulr ), \
			/* Init multiplication */ \
			M_MOVL( R3, LBL_MULR_variables ), \
			I_ST( R2, R3, VAR_MULR_return ), 	/* Save return address. */ \
			I_MOVI( R2, 0 ),					/* R2 = Result of multiplication */ \
		M_LABEL( LBL_MULR_nextMulti ), \
			I_ANDI( R3, R0, 1 ),			/* Is 1te bit set? */ \
			M_BXZ( LBL_MULR_noAdd ),		/* if( EQ ) Jump -> noAdd */ \
			I_ADDR( R2, R2, R1 ),			/* Increment result */ \
			M_BXF( LBL_MULR_OV ), 			/* OV -> Abort */ \
		M_LABEL( LBL_MULR_noAdd ), \
			/* Next iteration */ \
			I_RSHI( R0, R0, 1 ),			/* R0 = R0 / 2 */ \
			M_BXZ( LBL_MULR_doneMulti ),		/* if( EQ ) Jump -> nextMultiply */ \
			I_ADDR( R1, R1, R1 ),  			/* R1 * 2 */ \
			M_BXF( LBL_MULR_OV ),			/* OV -> Abort */ \
			M_BX( LBL_MULR_nextMulti ), \
		M_LABEL( LBL_MULR_OV ), \
			/* Calc overflow -> ABORT */ \
			I_MOVI( R0, 0 ), 				/* Result = 0 */ \
			I_MOVI( R1, 1 ),				/* OV set */ \
			M_BX( LBL_MULR_return ), \
		M_LABEL( LBL_MULR_doneMulti ), \
			/* Done multiply */ \
			I_MOVR( R0, R2 ),				/* Result in R0 */ \
			I_MOVI( R1, 0 ), 				/* no OV */ \
		M_LABEL( LBL_MULR_return ), \
			/* Return */ \
			M_MOVL( R3, LBL_MULR_variables ), \
			I_LD( R2, R3, VAR_MULR_return ), 	/* Restore return address */ \
			I_BXR( R2 ),						/* jump to caller return address */ \
		M_LABEL ( LBL_MULR_variables ), \
			/* Variable */ \
			I_HALT()		/* Caller return address */

////////////////////////////////////

	enum divr_data_list
	{
		// Variable intern for M_DIVR.
		VAR_DIVR_return = 0,		// Return address to subroutine caller.

		// Labels for M_DIVR.
		SUB_DIVR = LBL_DIVR_start,
		LBL_DIVR_nextShift,
		LBL_DIVR_startDivision,
		LBL_DIVR_nextDivision,
		LBL_DIVR_checkEndDivision,
		LBL_DIVR_doneDivision,
		LBL_DIVR_err,
		LBL_DIVR_variables
	};

	/**
	 * Division. R0 = R0 / R1 -> R1 is rest.
	 * Use SUB_DIVR to call it.
	 * All Registers are used.
	 * 
	 * Exp:
	 * 	M_MOVL( R0, 100 ),
	 * 	M_MOVL( R1, 25 ),
	 * 	M_JSRL( SUB_DIVR, HULP_LBLA() ),
	 *  I_HALT(),
	 *  M_DIVR( SUB_DIVR ),
	 * 
	 * Parameter:
	 * 	R0	- Dividend.
	 * 	R1	- Divisor.
	 * 	R2	- Return address from this function.
	 * 
	 * Return:
	 * 	R0	- Factor.
	 * 	R1	- Rest of division.
	*/
	#define M_DIVR( lbl_divr ) \
		M_LABEL( lbl_divr ), \
			/* Init division */ \
			M_MOVL( R3, LBL_DIVR_variables ), \
			I_ST( R2, R3, VAR_DIVR_return ), 	/* Save return address. */ \
			I_MOVR( R2, R1 ),					/* Divisor in R2 */ \
			I_MOVR( R1, R0 ),					/* Dividend in R1, also rest of division */ \
			I_MOVR( R0, R2 ),					/* Divisor to R0 to test EQ */ \
			M_BXZ( LBL_DIVR_err ),				/* Divisor == 0, result = 0, end */ \
			/* R0 = R1 / R2 , rest in R1 */ \
			I_STAGE_RST(),						/* Shift counter = 0 */ \
			I_STAGE_INC( 1 ), 					/* Start at Scnt = 1 */ \
		M_LABEL( LBL_DIVR_nextShift ), \
			/* Shift divisor to left until bit 15 is set */ \
			I_ANDI( R0, R2, 0x8000 ),			/* Check if bit 15 is set */ \
			M_BGE( LBL_DIVR_startDivision, 1 ),	/* R0 >= 1 -> bit 15 set, start division */ \
			I_LSHI( R2, R2, 1 ),				/* R2 << 1 */ \
			I_STAGE_INC( 1 ), 					/* Scnt++ */ \
			M_BX( LBL_DIVR_nextShift ),			/* Next shift */ \
		M_LABEL( LBL_DIVR_startDivision ), \
			I_MOVI( R0, 0 ),					/* Set result = 0 */ \
		M_LABEL( LBL_DIVR_nextDivision ), \
			I_LSHI( R0, R0, 1 ),				/* R0 << 1 ( shift result ) */ \
			I_SUBR( R3, R1, R2 ), 				/* R1 < R2 = OV */ \
			M_BXF( LBL_DIVR_checkEndDivision ), /* OV set, result unchanged */ \
			I_ADDI( R0, R0, 1 ),				/* R0++, result */ \
			I_SUBR( R1, R1, R2 ),				/* Correct dividend */ \
		M_LABEL( LBL_DIVR_checkEndDivision ), \
			I_RSHI( R2, R2, 1 ),				/* R2 >> 1 ( shift divisor )*/ \
			I_STAGE_DEC( 1 ),					/* Shift counter - 1 */ \
			M_BSGE( LBL_DIVR_nextDivision, 1 ),	/* if( Scnt >= 1 ) Jump */ \
		M_LABEL( LBL_DIVR_doneDivision ), \
			/* Division done */ \
			M_MOVL( R3, LBL_DIVR_variables ), \
			I_LD( R2, R3, VAR_DIVR_return ),	 /* Restore return address */ \
			I_BXR( R2 ),						/* Jump to return address */ \
		M_LABEL( LBL_DIVR_err ), \
			/* Results are 0 */ \
			I_MOVI( R0, 0 ), \
			I_MOVI( R1, 0 ), \
			M_BX( LBL_DIVR_doneDivision ), \
		M_LABEL ( LBL_DIVR_variables ), \
			/* Variables */ \
			I_HALT()		/* Caller return address */

////////////////////////////////////

	enum  pctr_data_list
	{
		// Variable intern for M_DIVR.
		VAR_PCTR_return = 0,		// Return address to subroutine caller.
		VAR_PCTR_R1,				// Copy of R1 (percentage)

		// Labels for I_DIVR.
		SUB_PCTR = LBL_PCTR_start,
		LBL_PCTR_divr_return,
		LBL_PCTR_mulr_return,
		LBL_PCTR_noOV,
		LBL_PCTR_variables
	};

	/**
	 * Calc percentage.
	 * Use SUB_PCTR to call it.
	 * All Registers are used.
	 * 
	 * Dependency:
	 * 	SUB_MULR, SUB_DIVR
	 * 
	 * Exp:
	 * 	M_MOVL( R0, 1000 ),		// Value
	 * 	M_MOVL( R1, 25 ),		// Percentage
	 * 	M_JSRL( SUB_PCTR, HULP_LBLA() ), 	// R0 is result
	 *  I_HALT(),
	 *  M_PCTR( SUB_PCTR ),
	 *  M_DIVR( SUB_MULR ),
	 *  M_DIVR( SUB_DIVR ),
	 * 
	 * Parameter:
	 * R0	- Value 100%.
	 * R1	- Percentage to calc %.
	 * R2	- Return address from this function.
	 * Return:
	 * R0	- Result of %.
	 * R1	- 1 if OV. Result = 0
	*/
	#define M_PCTR( lbl_pctr ) \
		M_LABEL( lbl_pctr ), \
			M_MOVL( R3, LBL_PCTR_variables ), \
			I_ST( R2, R3, VAR_PCTR_return ), 	/* Save return address. */ \
			I_ST( R1, R3, VAR_PCTR_R1 ),		/* Save percentsge value. */ \
			/* R0 = R0 / 100 */ \
			I_MOVI( R1, 100 ),					/* Divisor */ \
			M_MOVL( R2, LBL_PCTR_divr_return ), /* R2 = return address from division */ \
			M_BX( SUB_DIVR ),  					/* Call sub division */ \
		M_LABEL( LBL_PCTR_divr_return ), \
			/* R0 = R0 / 100 */ \
			M_MOVL( R3, LBL_PCTR_variables ), \
			I_LD( R1, R3, VAR_PCTR_R1 ),		/* Restore percentsge value. */ \
			M_MOVL( R2, LBL_PCTR_mulr_return ), /* R2 = return address from multiplication */ \
			M_BX( SUB_MULR ),  					/* Call sub multiplication */ \
		M_LABEL( LBL_PCTR_mulr_return ), \
			I_ANDI( R1, R1, 1 ),				/* R1 = OV */ \
			M_BXZ( LBL_PCTR_noOV ),				/* EQ */ \
			/* Calc OV, result = 0 */ \
			I_MOVI( R0, 0 ), \
		M_LABEL( LBL_PCTR_noOV ), \
			M_MOVL( R3, LBL_PCTR_variables ), \
			I_LD( R2, R3, VAR_PCTR_return ), 	/* Restore return address */ \
			I_BXR( R2 ),						/* Jump to return address */ \
		M_LABEL ( LBL_PCTR_variables ), \
			/* Variables */ \
			I_HALT(),		/* Caller return address */ \
			I_HALT()		/* Copy R1 (percentage) */

////////////////////////////////////

	enum pwm_data_list {
		// Data Struct.
		_VAR_PWN_TicksNow = 0,		// Ticks now.
		_VAR_PWM_Duty,				// High duty in % from periode.
		_VAR_PWM_OldDuty,			// Old high duty to check if it has changed.
		_VAR_PWM_High,				// High signal ticks time.
		_VAR_PWM_Low,				// Low signal ticks time.
		_VAR_PWM_Target,			// Next target ticks time.
		_VAR_PWM_Flag,				// Flags for calculation.

		// Lable for M_PWM_
		_LBL_PWM_Variables = 0,
		_LBL_PWM_Init,
		_LBL_PWM_CalcDuty,
		_LBL_PWM_DoPWM,
		_LBL_PWM_CheckExpired,
		_LBL_PWM_TargetReached,
		_LBL_PWM_SetTarget,
		_LBL_PWM_CorrectTicksNow,
		_LBL_PWM_SaveTarget,
		_LBL_PWM_PinHigh,
		_LBL_PWM_PinLow,
		_LBL_PWM_End,
		LBL_PWM_Count,			/* Use to calc lbl_number use for PWM */
	};

	/**
	 * Generate PWM signal on a pin, with fix duty (high) cycle.
	 * !! Shortes periode is 5 mSeconds !!
	 * All Registers are used.
	 * 
	 * Example:
	 * 	I_MOVI( R3, 0 ),
	 *	I_MOVI( R1, struct_pwm_1 ),
	 *	M_PWMR( LBL_PWM, GPIO_NUM_4, 1000, 50 )
	 *
	 * Parameter:
	 * 	_lblNbr	- Start Label, use up to _lblNbr + LBL_PWM_Count.
	 * 	_pin	- Pin to use for the PWM.
	 * 	_ms 	- Periode of the PWM in ms. (u16)
	 * 	_duty	- Duty (high) cycle of the PWM in % of _ms. (u16)
	*/
	#define M_PWMI( _lblNbr, _pin, _ms, _duty ) \
			I_MOVI( R0, _duty ),	/* R0 = imm */ \
			M_PWM_( _lblNbr, _pin, HULPx_ms_to_ulp_ticks(_ms), HULPx_ms_to_ulp_tick_shift(_ms), R0 )

	/**
	 * Generate PWM signal on a pin.
	 * !! Shortes periode is 5 mSeconds !!
	 * All Registers are used.
	 * 
	 * Example:
	 * 	I_MOVI( R3, 0 ),
	 *	I_GET( R0, R3, var ),
	 *	I_MOVI( R1, struct_pwm_1 ),
	 *	M_PWMR( LBL_PWM, GPIO_NUM_4, 10000 )
	 * 
	 * Register:
	 *  R0		- Duty (high) cycle of the PWM in % of _ms. (u16)
	 * 
	 * Parameter:
	 * 	_lblNbr	- Start Label, use up to _lblNbr + LBL_PWM_Count.
	 * 	_pin	- Pin to use for the PWM.
	 * 	_ms 	- Periode of the PWM in ms. (u16)
	*/
	#define M_PWMR( _lblNbr, _pin, _ms ) \
			M_PWM_( _lblNbr, _pin, HULPx_ms_to_ulp_ticks(_ms), HULPx_ms_to_ulp_tick_shift(_ms), R0 )

	/**
	 * Generate PWM signal on a pin.
	 * All Registers are used.
	 * 
	 * Parameter:
	 * 	_lblNbr		- Start Label, use up to _lblNbr + LBL_PWM_Count.
	 * 	_pin		- in to use for the PWM.
	 * 	_ticks, _tick_shift	- Periode of the PWM in ms converted to ticks & tick_shift. (ticks are limited to 32000)
	 * 	_R0_duty	- R0, duty (high) cycle of the PWM in percent of _ms. (u16)
	*/
	#define M_PWM_( _lblNbr, _pin, _ticks, _tick_shift, _R0_duty ) \
			M_MOVL( R3, _lblNbr + _LBL_PWM_Variables ), \
			I_LD( R1, R3, _VAR_PWM_OldDuty ),		/* R1 = old duty */ \
			I_SUBR( R1, _R0_duty, R1 ), 			/* duty - oldDuty == 0 -> unchanged */ \
			M_BXZ( _lblNbr + _LBL_PWM_DoPWM ),		/* if( EQ ) do PWM signal */ \
			M_BX( _lblNbr + _LBL_PWM_CalcDuty ), 	/* Calc new duty data */ \
		M_LABEL( _lblNbr + _LBL_PWM_Variables ), \
			/* Variables */ \
			I_HALT(),	/* Ticks now. Last read of ticks. */ \
			I_HALT(),	/* Duty ( high % of mS ) */ \
			I_HALT(),	/* Old duty, to detect new duty */ \
			I_HALT(),	/* High ticks */ \
			I_HALT(),	/* Low ticks */ \
			I_HALT(),	/* Target ticks */ \
			I_HALT(),	/* Flags */ \
		M_LABEL( _lblNbr + _LBL_PWM_CalcDuty ), \
			/* Calc duty hi/lo ticks */ \
			I_ST( _R0_duty, R3, _VAR_PWM_Duty ),	/* Save duty in % */ \
			I_ST( _R0_duty, R3, _VAR_PWM_OldDuty ),	/* Old dusy == duty */ \
			I_MOVR( R1, _R0_duty ), 				/* Duty to R1 */ \
			I_MOVI( R0, _ticks ), \
			M_JSRL( SUB_PCTR, HULP_LBLA() ), 		/* Calc % -> R0 = high ticks */ \
			M_MOVL( R3, _lblNbr + _LBL_PWM_Variables ), \
			I_ST( R0, R3, _VAR_PWM_High ),			/* Save high ticks */ \
			I_MOVI( R2, _ticks ),					/* R2 = ticks */ \
			I_SUBR( R0, R2, R0 ),					/* low ticks = ticks - high */ \
			I_ST( R0, R3, _VAR_PWM_Low ),			/* Save low ticks */ \
		M_LABEL( _lblNbr + _LBL_PWM_DoPWM ), \
			/* Check if duty is 0 or 100 %, force pin to state */ \
			I_LD( R0, R3, _VAR_PWM_Duty ),			/* R0 = duty */ \
			M_BL( _lblNbr + _LBL_PWM_PinLow, 1 ),	/* Duty < 1, set pin to low */ \
			M_BGE( _lblNbr + _LBL_PWM_PinHigh, 100 ),	/* Duty >= 100, set pin to high */ \
			/* Check tick expired */ \
			I_RD_TICKS_REG( _tick_shift ),			/* R0 = ticks_now */ \
			I_ST( R0, R3, _VAR_PWN_TicksNow ),		/* Save ticks_now */ \
			I_LD( R1, R3, _VAR_PWM_Flag ),			/* R1 = correct ticks_now flag */ \
			I_ANDI( R1, R1, 1 ),					/* Is flag set? */ \
			M_BXZ( _lblNbr + _LBL_PWM_CheckExpired ),	/* Flag not set */ \
			/* Do ticks_now correction */ \
			I_SUBI( R0, R0, _ticks ), 				/* Correct ticks_now */ \
		M_LABEL( _lblNbr + _LBL_PWM_CheckExpired ), \
			I_LD( R1, R3, _VAR_PWM_Target ), 		/* R1 = target ticks */ \
			I_SUBR( R1, R1, R0 ), 					/* Traget - ticks now */ \
			M_BXF( _lblNbr + _LBL_PWM_TargetReached ),	/* OV - traged reached */ \
			M_BX( _lblNbr + _LBL_PWM_End ),			/* Still waiting */ \
		M_LABEL( _lblNbr + _LBL_PWM_TargetReached ), \
			/* Toggle pin */ \
			I_GPIO_SET_RD( ( _pin ) ),				/* R0 = pin state -> 0/1 */ \
			I_GPIO_SET( (_pin), 1 ),				/* Set pin to high */ \
			I_LD( R2, R3, _VAR_PWM_High ),			/* Load high ticks, signal ticks */ \
			M_BL( _lblNbr + _LBL_PWM_SetTarget, 1 ), 	/* if( R0 < 1 ) -> pin is 0 */ \
			I_GPIO_SET( (_pin), 0 ),				/* Set pin to high */ \
			I_LD( R2, R3, _VAR_PWM_Low ),			/* Load low ticks, signal ticks */ \
		M_LABEL( _lblNbr + _LBL_PWM_SetTarget ), \
			/* Calc new target */ \
			/* If new target > 63000 or OV then correct the ticks_now by the ticks periode. */ \
			I_MOVI( R1, 0 ),						/* R1 = correct ticks_now flag */ \
			I_LD( R0, R3, _VAR_PWN_TicksNow ), 		/* R0 = ticks_now */ \
			I_ADDR( R0, R0, R2 ),					/* ticks_now + ticks signal = new target */ \
			M_BGE( _lblNbr + _LBL_PWM_CorrectTicksNow, 63000 ),	/* New target > 63000 -> correct ticks_now */ \
			M_BXF( _lblNbr + _LBL_PWM_CorrectTicksNow ),	/* New target get OV -> correct ticks_now */ \
			M_BX( _lblNbr + _LBL_PWM_SaveTarget ),	/* No correction needed */ \
		M_LABEL( _lblNbr + _LBL_PWM_CorrectTicksNow ), \
			/* Correct ticks_now to be sure that the target still in bounce */ \
			I_LD( R0, R3, _VAR_PWN_TicksNow ), 		/* R0 = ticks_now */ \
			I_SUBI( R0, R0, _ticks ),				/* Correct ticks_now */ \
			I_ADDR( R0, R0, R2 ), 					/* target = ticks_now + signal ticks */ \
			I_MOVI( R1, 1 ),						/* R1 = correct ticks_now flag */ \
		M_LABEL( _lblNbr + _LBL_PWM_SaveTarget ), \
			/* Save the new target and flags */ \
			I_ST( R0, R3, _VAR_PWM_Target ),		/* Save target */ \
			I_ST( R1, R3, _VAR_PWM_Flag	), 			/* Save flags */ \
			M_BX( _lblNbr + _LBL_PWM_End), \
		M_LABEL( _lblNbr + _LBL_PWM_PinLow ), \
			/* Force pin low */ \
			I_GPIO_SET( (_pin), 0 ),				/* Set pin to low */ \
			M_BX( _lblNbr + _LBL_PWM_End), \
		M_LABEL( _lblNbr + _LBL_PWM_PinHigh ), \
			/* Force pin high */ \
			I_GPIO_SET( (_pin), 1 ),				/* Set pin to high */ \
		M_LABEL( _lblNbr + _LBL_PWM_End )


#endif
