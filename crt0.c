/** @file crt0.c
 *
 * @brief A C runtime
 *
 * Use only one of crt0.S and crt0.c
 *
 * This file is adapted from grct1.S in the avr source.
 *
 * For use in Mantis Cheng's CSC 460 
 * To use this as startup code in AVR Studio,
 * add "-nostartfiles" to [Linker Options]
 * in Project>Configuration options>Custom Options.
 *
 * The name of the function in the last call is where the program starts.
 * (For project 2 it should be "OS_Init".)
 *
 * @author Scott Craig
 * @author Justin Tanner
 */

#include <avr/io.h>
#include <avr/sfr_defs.h>

#include "os.h"

/** The "zero" register*/
#define zero_reg  "r1"

/**
 * @brief A macro to simplify the vector list.
 *
 * The symbol "__vector_i" is weakly bound to this spot in the
 * object file. Later, other object files can reference this
 * spot using this symbol.
 *
 * The value of the symbol is set to "__vector_not_set",
 * which is the label of a function below. Other files will
 * change this if an ISR is declared.
 *
 * The instruction at this spot is "jmp (addr)" (4 bytes).
 * These addresses are hardwired in the mcu.
 */
#define   vector(name)  asm(\
    ".weak "  name "\n\t"\
    ".set  "  name " , __vector_not_set\n\t"\
    "jmp   "  name "\n\t"::);


/**
 * @brief The vectors section.
 *
 * The numbers are off by 1 from the hardware manual,
 * but consistent with iousbxx6_7.h.
 * Vector "0" is the reset vector, which jumps to the
 * executable code.
 *
 * Any interrupt ISR definition in the C code will
 * overwrite these default definitions.
 */
void __vectors (void) __attribute__ ((naked)) __attribute__ ((section (".vectors")));
void __vectors (void)
{
    asm("jmp        __init\n"::);

    vector("__vector_1");
    vector("__vector_2");
    vector("__vector_3");
    vector("__vector_4");
    vector("__vector_5");
    vector("__vector_6");
    vector("__vector_7");
    vector("__vector_8");
    vector("__vector_9");
    vector("__vector_10");
    vector("__vector_11");
    vector("__vector_12");
    vector("__vector_13");
    vector("__vector_14");
    vector("__vector_15");
    vector("__vector_16");
    vector("__vector_17");
    vector("__vector_18");
    vector("__vector_19");
    vector("__vector_20");
    vector("__vector_21");
    vector("__vector_22");
    vector("__vector_23");
    vector("__vector_24");
    vector("__vector_25");
    vector("__vector_26");
    vector("__vector_27");
    vector("__vector_28");
    vector("__vector_29");
    vector("__vector_30");
    vector("__vector_31");
    vector("__vector_32");
    vector("__vector_33");
    vector("__vector_34");
    vector("__vector_35");
    vector("__vector_36");
    vector("__vector_37");
}


/**
 * @fn __vector_not_set
 *
 * @brief A default routine that is called when an interrupt occurs
 * for which no ISR was assigned. 
 *
 * The default action is to reset,
 * but it could be changed to do something else.
 */   
void __vector_not_set (void) __attribute__ ((naked)) __attribute__ ((section (".text")));
void __vector_not_set (void)
{
    asm("jmp    __vectors\n\t"::);
}


/**
 * @brief The beginning of the executable code in this file.
 *
 * The section names tell the linker where to place the
 * code as specified in the linker script. eg. avr5.x
 */
void __init (void) __attribute__ ((naked)) __attribute__ ((section (".init0")));
void __init (void)
{
    asm(".weak    __init\n\t"
        "__init:\n\t"
        ".weak    __heap_end\n\t"
        ".set    __heap_end, 0\n\t"::);
}

    
/**
 * @brief init2
 *
 * Clear the "zero" register,
 * clear the status register, and
 * set the stack pointer.
 */
void init2 (void) __attribute__ ((naked)) __attribute__ ((section (".init2")));
void init2 (void)
{
    asm("clr   " zero_reg "\n\t"::);

    SREG = 0;
    SP = RAMEND;
}


/** 
 * @brief init4
 *
 * Copy data from __data_load_start in program memory
 * to __data_start in SRAM, initializing data in the process.
 * A similar routine with the same name is defined in libgcc.S.
 * This routine overrides it.
 */
void __do_copy_data (void) __attribute__ ((naked)) __attribute__ ((section (".init4")));
void __do_copy_data (void)
{
    asm(
    "ldi    r17, hi8(__data_end) \n"
    "ldi    r26, lo8(__data_start) \n"
    "ldi    r27, hi8(__data_start) \n"
    "ldi    r30, lo8(__data_load_start) \n"
    "ldi    r31, hi8(__data_load_start) \n"
    "ldi    r16, hh8(__data_load_start) \n"
    "out    %0, r16 \n"
    "rjmp    .L__do_copy_data_start \n"
".L__do_copy_data_loop: \n"
    "elpm    r0, Z+ \n"
    "st    X+, r0 \n"
".L__do_copy_data_start: \n"
    "cpi    r26, lo8(__data_end) \n"
    "cpc    r27, r17 \n"
    "brne    .L__do_copy_data_loop \n"::"I" (_SFR_IO_ADDR(RAMPZ)));

}

    /* set all unitialized data in .bss to 0
     * Already included in libgcc.S */
/*
    .global __do_clear_bss
__do_clear_bss:
    ldi    r17, hi8(__bss_end)
    ldi    r26, lo8(__bss_start)
    ldi    r27, hi8(__bss_start)
    rjmp    .do_clear_bss_start
.do_clear_bss_loop:
    st    X+, __zero_reg__
.do_clear_bss_start:
    cpi    r26, lo8(__bss_end)
    cpc    r27, r17
    brne    .do_clear_bss_loop
*/

/** @brief init9
 *
 * The last of the init functions.
 * Usually this would be the jump to "main()"
 */
void init9 (void) __attribute__ ((naked)) __attribute__ ((section (".init9")));
void init9 (void)
{
    OS_Init();

    for(;;);


    /** exit is defined in libgcc.S. It is an rjmp to itself.
     * If the function called in init9 returns, it returns
     * here.
     */
    /* jmp    exit */

}