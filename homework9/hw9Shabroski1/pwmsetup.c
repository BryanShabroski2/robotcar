/**************************************************
* pwmsetup.c  - file
* pwmsetup.h
* 
* Function:  void pwm_setup(int pwm_range, struct io_peripherals *io)
*   Configure the PWM channels
*     set the range value, typically 100 level steps
*     set pwm mode and polarity, enable channel 1 and 2, etc.
* 
* By Kyusun Choi
* Revision V1.0   On 7/10/2022
* 
***************************************************/

#include <stdint.h>
#include "../include/pwm.h"
#include "../include/io_peripherals.h"


/* setting the RPi4 hardware PWM range, power level=pwm.DAT/PWM_RANGE   */
/* Minimum range is 32, typical range is 100, stepping by 1% => pwm.DAT */
/* If 0.1% stepping needed for control, use range of 1000               */
/* Example: pwm.DAT1/PWM_RANGE = 789/1000 = 78.9% level                 */


void pwm_setup(int pwm_range, struct io_peripherals *io)
{
    /* set up the PWM channels */
    io->pwm->RNG1 = pwm_range;     /* the range value, 100 level steps */
    io->pwm->RNG2 = pwm_range;     /* the range value, 100 level steps */
    io->pwm->DAT1 = 1;             /* initial beginning level=1/100=1% */
    io->pwm->DAT2 = 1;             /* initial beginning level=1/100=1% */
    io->pwm->CTL.field.MODE1 = 0;  /* PWM mode */
    io->pwm->CTL.field.MODE2 = 0;  /* PWM mode */
    io->pwm->CTL.field.RPTL1 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm->CTL.field.RPTL2 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm->CTL.field.SBIT1 = 0;  /* idle low */
    io->pwm->CTL.field.SBIT2 = 0;  /* idle low */
    io->pwm->CTL.field.POLA1 = 0;  /* non-inverted polarity */
    io->pwm->CTL.field.POLA2 = 0;  /* non-inverted polarity */
    io->pwm->CTL.field.USEF1 = 0;  /* do not use FIFO */
    io->pwm->CTL.field.USEF2 = 0;  /* do not use FIFO */
    io->pwm->CTL.field.MSEN1 = 1;  /* use M/S algorithm, level=pwm.DAT1/PWM_RANGE */
    io->pwm->CTL.field.MSEN2 = 1;  /* use M/S algorithm, level=pwm.DAT2/PWM_RANGE */
    io->pwm->CTL.field.CLRF1 = 1;  /* clear the FIFO, even though it is not used */
    io->pwm->CTL.field.PWEN1 = 1;  /* enable the PWM channel */
    io->pwm->CTL.field.PWEN2 = 1;  /* enable the PWM channel */
}
