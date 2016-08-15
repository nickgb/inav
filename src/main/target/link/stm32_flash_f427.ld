/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32F427 Device with
**                2048 KByte FLASH, 192KByte RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Environment : Atollic TrueSTUDIO(R)
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0;      /* required amount of heap  */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
	FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 0x200000 - 0x128
	INFOX (rx)      : ORIGIN = 0x08000000 + 0x200000 - 0x64, LENGTH = 0x128
	RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 192K
	MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
}

INCLUDE "stm32_flash.ld"