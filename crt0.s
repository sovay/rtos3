/**
 * @file crt0.S
 *
 * @brief Doxygen doesn't handle assembly code well. Look at the source code instead.
 *
 * Use only one of crt0.S and crt0.c
 *
 * An assembler file modified by Scott Craig from gcrt1.S
 *
 * For use in Mantis Cheng's CSC 460
 *
 * To use this as startup code in AVR Studio,
 * add "-nostartfiles" to [Linker Options]
 * in Project>Configuration options>Custom Options.
 *
 * The name of the function in the last call is where the program starts.
 * (For project 2 it should be "OS_Init".)
 */

/* Copyright (c) 2002, Marek Michalkiewicz <marekm@amelek.gda.pl>
   Copyright (c) 2007, Eric B. Weddington
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE. */

#include <avr/io.h>
#include <avr/sfr_defs.h>

/**
 * gcc expects this register to contain 0. This loses the use
 * of one register, but it speeds up operations such as
 * clearing a memory location.
 */
#define __zero_reg__ r1

    /**
     * A macro to simplify the vector list.
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
    .macro    vector name
    .weak   \name
    .set    \name, __vector_not_set
    jmp     \name
    .endm

    /** The vectors section.
     *
     * The numbers are off by 1 from the hardware manual,
     * but consistent with iousbxx6_7.h.
     * Vector "0" is the reset vector, which jumps to the
     * executable code.
     *
     * Any interrupt ISR definition in the C code will
     * overwrite these default definitions.
     */
    .section .vectors,"ax",@progbits
    .global    __vectors
    .func       __vectors
__vectors:
    jmp         __init
    vector      __vector_1
    vector      __vector_2
    vector      __vector_3
    vector      __vector_4
    vector      __vector_5
    vector      __vector_6
    vector      __vector_7
    vector      __vector_8
    vector      __vector_9
    vector      __vector_10
    vector      __vector_11
    vector      __vector_12
    vector      __vector_13
    vector      __vector_14
    vector      __vector_15
    vector      __vector_16
    vector      __vector_17
    vector      __vector_18
    vector      __vector_19
    vector      __vector_20
    vector      __vector_21
    vector      __vector_22
    vector      __vector_23
    vector      __vector_24
    vector      __vector_25
    vector      __vector_26
    vector      __vector_27
    vector      __vector_28
    vector      __vector_29
    vector      __vector_30
    vector      __vector_31
    vector      __vector_32
    vector      __vector_33
    vector      __vector_34
    vector      __vector_35
    vector      __vector_36
    vector      __vector_37
    .endfunc

    /**
     * A default routine that is called when an interrupt occurs
     * for which no ISR was assigned. The default action is to reset,
     * but it could be changed to do something else.
     */
    .text
    .global     __vector_not_set
    .func       __vector_not_set
__vector_not_set:
    jmp         __vectors
    .endfunc

    /**
     * The beginning of the executable code in this file.
     * The section names tell the linker where to place the
     * code as specified in the linker script. eg. avr5.x
     */
    .section .init0,"ax",@progbits
    .weak       __init
__init:
    .weak       __stack
    .set        __stack, RAMEND
    .weak       __heap_end
    .set        __heap_end, 0

    /**
     * init2
     *
     * Clear the "zero" register,
     * clear the status register, and
     * set the stack pointer.
     */
    .section .init2,"ax",@progbits
    clr     __zero_reg__
    out     _SFR_IO_ADDR(SREG), __zero_reg__
    ldi     r28,lo8(__stack)
    ldi     r29,hi8(__stack)
    out     _SFR_IO_ADDR(SPH), r29
    out     _SFR_IO_ADDR(SPL), r28

    /**
     * init4
     *
     * Copy data from __data_load_start in program memory
     * to __data_start in SRAM, initializing data in the process.
     * A similar routine with the same name is defined in libgcc.S.
     * This routine overrides it.
     */
    .section .init4,"ax",@progbits
    .global __do_copy_data
__do_copy_data:
    ldi     r17, hi8(__data_end)
    ldi     r26, lo8(__data_start)
    ldi     r27, hi8(__data_start)
    ldi     r30, lo8(__data_load_start)
    ldi     r31, hi8(__data_load_start)
    ldi     r16, hh8(__data_load_start)
    out     _SFR_IO_ADDR(RAMPZ), r16
    rjmp    .L__do_copy_data_start
.L__do_copy_data_loop:
    elpm    r0, Z+
    st      X+, r0
.L__do_copy_data_start:
    cpi     r26, lo8(__data_end)
    cpc     r27, r17
    brne    .L__do_copy_data_loop


    /* set all unitialized data in .bss to 0
     * Already included in libgcc.S */
/*
    .global __do_clear_bss
__do_clear_bss:
    ldi     r17, hi8(__bss_end)
    ldi     r26, lo8(__bss_start)
    ldi     r27, hi8(__bss_start)
    rjmp    .do_clear_bss_start
.do_clear_bss_loop:
    st      X+, __zero_reg__
.do_clear_bss_start:
    cpi     r26, lo8(__bss_end)
    cpc     r27, r17
    brne    .do_clear_bss_loop
*/

    /** init9
     *
     * The last of the init functions.
     * Usually this would be the jump to "main()"
     */
    .section .init9,"ax",@progbits
    call    OS_Init

    /** exit is defined in libgcc.S. It is an rjmp to itself.
     * If the function called in init9 returns, it returns
     * here.
     */
    jmp     exit
