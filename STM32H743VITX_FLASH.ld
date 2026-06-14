/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                2048Kbytes FLASH and 1056Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2023 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* 栈顶 */

_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);

/* heap / stack */

_Min_Heap_Size  = 0x4000;
_Min_Stack_Size = 0x4000;

/*************************************************
 * Memory Layout
 *************************************************/

MEMORY
{
/* Bootloader专用 */
/*    FLASH    (rx)  : ORIGIN = 0x08020000, LENGTH = 1920K*/
    FLASH    (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K

    ITCMRAM  (xrw) : ORIGIN = 0x00000000, LENGTH = 64K

    DTCMRAM  (xrw) : ORIGIN = 0x20000000, LENGTH = 128K

    RAM_D1   (xrw) : ORIGIN = 0x24000000, LENGTH = 512K

    RAM_D2   (xrw) : ORIGIN = 0x30000000, LENGTH = 288K

    RAM_D3   (xrw) : ORIGIN = 0x38000000, LENGTH = 64K
}

/*************************************************
 * Sections
 *************************************************/

SECTIONS
{

/*************************************************
 * Vector Table
 *************************************************/

.isr_vector :
{
    . = ALIGN(4);

    KEEP(*(.isr_vector))

    . = ALIGN(4);

} > FLASH


/*************************************************
 * Code
 *************************************************/

.text :
{
    . = ALIGN(4);

    *(.text)
    *(.text*)

    *(.glue_7)
    *(.glue_7t)

    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);

    _etext = .;

} > FLASH


/*************************************************
 * Read Only Data
 *************************************************/

.rodata :
{
    . = ALIGN(4);

    *(.rodata)
    *(.rodata*)

    . = ALIGN(4);

} > FLASH


/*************************************************
 * Exception Table
 *************************************************/

.ARM.extab :
{
    *(.ARM.extab*)
} > FLASH

.ARM :
{
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
} > FLASH


/*************************************************
 * ITCM CODE
 *************************************************/

_siitcm = LOADADDR(.itcm_text);

.itcm_text :
{
    . = ALIGN(4);

    _sitcm = .;

    *(.itcm)
    *(.itcm*)

    . = ALIGN(4);

    _eitcm = .;

} > ITCMRAM AT > FLASH


/*************************************************
 * DTCM DATA
 *************************************************/

_sidtcm = LOADADDR(.dtcm_data);

.dtcm_data :
{
    . = ALIGN(4);

    _sdtcm = .;

    *(.dtcm)
    *(.dtcm*)

    . = ALIGN(4);

    _edtcm = .;

} > DTCMRAM AT > FLASH


/*************************************************
 * Initialized Data
 *************************************************/

_sidata = LOADADDR(.data);

.data :
{
    . = ALIGN(4);

    _sdata = .;

    *(.data)
    *(.data*)

    . = ALIGN(4);

    _edata = .;

} > RAM_D1 AT > FLASH


/*************************************************
 * DMA BUFFER
 *************************************************/

.dma_buffer (NOLOAD):
{
    . = ALIGN(32);

    *(.dma_buffer)
    *(.dma_buffer*)

    . = ALIGN(32);

} > RAM_D2


/*************************************************
 * RTT / SystemView Buffer
 *************************************************/

.rtt_memory (NOLOAD):
{
    . = ALIGN(32);

    *(.rtt_memory)
    *(.rtt_memory*)

    . = ALIGN(32);

} > RAM_D2


/*************************************************
 * FAST STACK
 *************************************************/

.fast_stack (NOLOAD):
{
    . = ALIGN(8);

    *(.fast_stack)
    *(.fast_stack*)

    . = ALIGN(8);

} > DTCMRAM


/*************************************************
 * BSS
 *************************************************/

.bss :
{
    . = ALIGN(4);

    _sbss = .;

    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);

    _ebss = .;

} > RAM_D1


/*************************************************
 * Heap Stack
 *************************************************/

._user_heap_stack :
{
    . = ALIGN(8);

    PROVIDE(end = .);
    PROVIDE(_end = .);

    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;

    . = ALIGN(8);

} > RAM_D1


/*************************************************
 * Remove Standard Lib
 *************************************************/

DISCARD :
{
    libc.a(*)
    libm.a(*)
    libgcc.a(*)
}

.ARM.attributes 0 :
{
    *(.ARM.attributes)
}

}
