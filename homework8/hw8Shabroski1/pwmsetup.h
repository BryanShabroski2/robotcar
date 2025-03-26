/**************************************************
* pwmsetup.h  - file
* pwmsetup.c
* 
* Function:  void pwm_config(int pwm_range, struct io_peripherals *io)
*   Configure the PWM channels
*     set the range value, typically 100 level steps
*     set pwm mode and polarity, enable channel 1 and 2, etc.
* 
* By Kyusun Choi
* Revision V1.0   On 7/10/2022
* 
***************************************************/


#ifndef PWM_SETUP_H_
#define PWM_SETUP_H_

void pwm_setup(int pwm_range, struct io_peripherals *io);

#endif /* PWM_SETUP_H_ */
