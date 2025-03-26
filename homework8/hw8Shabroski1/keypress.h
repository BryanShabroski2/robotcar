/**************************************************
* keypress.h  - file
* keypress.c
* 
* Function:  int get_pressed_key(void)
*   Get a single key hit (without Enter Key hit)
*     set STDIO mode to NONBLOCK
*     (non-blocking and non-waiting mode)
*     key character show on the terminal (echo print)
* 
* By Kyusun Choi
* Revision V1.0   On 7/03/2022
* 
***************************************************/

#ifndef KEY_PRESS_H_
#define KEY_PRESS_H_

int get_pressed_key(void);

#endif /* KEY_PRESS_H_ */
