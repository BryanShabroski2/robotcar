/**************************************************
* keypress.c  - file
* keypress.h
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


#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>


int get_pressed_key(void)
{
  struct termios  original_attributes;
  struct termios  modified_attributes;
  long   oldf, newf;
  int    ch;

  /* Make input available immediately, without 'enter' key,       */
  /* no input processing, disable line editing, noncanonical mode */
  tcgetattr( STDIN_FILENO, &original_attributes );
  modified_attributes = original_attributes;
  modified_attributes.c_lflag &= ~(ICANON | ECHO);
  modified_attributes.c_cc[VMIN] = 1;
  modified_attributes.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );

  /* Save the mode of STDIN, then      <use fcntl>   */
  /*   change the mode to NONBLOCK, then get char,   */
  /*   and then restore the STDIN mode back.         */
  /* Key character show on the terminal (echo print) */
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  newf = oldf | O_NONBLOCK;
  fcntl(STDIN_FILENO, F_SETFL, newf);
  ch = getchar();                       //get the character
  fcntl(STDIN_FILENO, F_SETFL, oldf);   //restore the STDIN mode back

  //reset the input to the orginal settings
  tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );

  if (ch != -1) {                    // '0' <= valid key range <= 'z'
    if (ch > 47) {                   // ignore all other keys
      if (ch < 123) {
        } //printf( "\n ch= %d\n", ch);}
      else {ch = -1;}
    }
    else {ch = -1;}
  }

  return ch;
  // this is keyboard reading without 'Enter' key and in
  // non-blocking mode.  This function returns single key
  // value (ASCII code), and returns -1 if no key was hit
}
