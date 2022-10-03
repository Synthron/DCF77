

/** @file dcf77.h
 *
 * @brief DCF77-lib
 * 
 * includes the µs timer and serial-out function
 *
 */

#ifndef DCF77_H
#define DCF77_H

#include "main.h"
#include "stdbool.h"
#include "tim.h"

/* 
    define us-Timer functions
  */

void timer_init();
void delay_us(uint16_t us);


#endif /* DCF77_H */

/*** EOF ***/
