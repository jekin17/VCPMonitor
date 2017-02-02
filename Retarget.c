/******************************************************************************/
/* RETARGET.C: 'Retarget' layer for target-dependent low level functions      */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005 Keil Software. All rights reserved.                     */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/
#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <rt_misc.h>

#pragma import(__use_no_semihosting_swi)

uint8_t transmit[1] = {0x00};
uint8_t recive[10];
extern  UART_HandleTypeDef huart1;
struct __FILE 
{ 
    int handle; 
};

FILE __stdout;
FILE __stdin;


int fputc(int ch, FILE *f)
    {
        UART_WaitOnFlagUntilTimeout(&huart1, UART_FLAG_TC, RESET, 100);
        HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 100);
    }

int fgetc(FILE *f)
    {
  //return (sendchar(getkey()));
}


int ferror(FILE *f) 
    {
  /* Your implementation of ferror */
  return EOF;
}


void _ttywrch(int ch) {  
    //sendchar (ch);
    }


void _sys_exit(int return_code) 
    {
  while (1);    /* endless loop */
}
