# HULPx

Extention for boarchuz's hulp  helping ULP marcos programming. [github ](https://github.com/boarchuz/HULP.git)

***

## INSTALLATION & DOWNLOAD
Install via the download from [github](https://github.com/Gfy63/HULPx.git)



## Upload ULP macros

### HULPx_process_macros_and_load(uint32_t load_addr, const ulp_insn_t* program, size_t* psize)
Same as ```process_macros_and_load()``` but no limitation on program size.

## Debugg

### hulpx_Dump(int start, int end, int programAddr)
Disassembler for UPL memory, print it to serial.

## Marcos

### M_JSRL( sub_lbl, return_lbl )
Jump to subroutine via label.

### M_PWMI( _lblNbr, _pin, _ms, _duty )
Generate PWM signal on a pin, with fix duty (high) cycle.
Use labels from ```_lblNbr``` to ```_lblNbr + LBL_PWM_Count```.

### M_PWMR( _lblNbr, _pin, _ms )
Generate PWM signal on a pin. R0 = duty (high) cycle.
Use labels from ```_lblNbr``` to ```_lblNbr + LBL_PWM_Count```.

## Subroutine

### M_MULR( lbl_mulr )
Multiplication. R0 = R0 * R1 -> R1 is OV, on OV the R0 = 0

### M_DIVR( lbl_divr )
Division. R0 = R0 / R1 -> R1 is rest.

### M_PCTR( lbl_pctr )
Percentage. R0 = R0 % R1 -> R1 is OV





