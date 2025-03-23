/**************************************************
* Homework 7, pthread and FIFO queue Sample 3 Program
* By Bryan Shabroski
* CMPEN 473, Spring 2023, Penn State University
*
* Revision V2.2 On 02/10/2023
* Revision V2.1 On 03/07/2022
* Revision V1.0 On 02/04/2018
*
* Manual control and camera line following with camera.
*
***************************************************/
//Added laser thread, laser thread param and stuff in main. Added mode3.
//Added enumeration for box for laser.
// header files - at /usr/include and ../include and .
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include "../include/import_registers.h"
#include "../include/cm.h"
#include "../include/gpio.h"
#include "../include/uart.h"
#include "../include/spi.h"
#include "../include/bsc.h"
#include "../include/pwm.h"
#include "../include/enable_pwm_clock.h"
#include "../include/io_peripherals.h"
#include "../include/wait_period.h"
#include "../include/FIFO.h"
#include "../include/MPU6050.h"
#include "../include/MPU9250.h"
#include "../include/wait_key.h"
#include "keypress.h"
#include "pwmsetup.h"
#include <gdk/gdk.h>
#include <cairo/cairo.h>
#include <gtk/gtk.h>
#include <linux/videodev2.h>
#include <time.h>
#include "pixel_format_RGB.h"
#include "video_interface.h"
#include "wait_key.h"
#include "scale_image_data.h"
#include "draw_bitmap_multiwindow.h"

#define GET_FRAMES                5  /* the number of frame times to average when determining the FPS */
#define FIFO_LENGTH     1024
#define THREE_QUARTERS  (FIFO_LENGTH*3/4)

typedef enum {
  BOX_NONE,
  BOX_TOP_LEFT,
  BOX_TOP_CENTER,
  BOX_TOP_RIGHT,
  BOX_MID_LEFT,
  BOX_MID_CENTER,
  BOX_MID_RIGHT,
  BOX_BOTTOM_LEFT,
  BOX_BOTTOM_CENTER,
  BOX_BOTTOM_RIGHT
} LaserBox;

volatile bool forward = false;
volatile bool backward = false;
volatile bool stop = false;
volatile bool left = false;
volatile bool right = false;
volatile bool mode1 =  true;
volatile bool mode2 = false;
volatile bool mode3 = false;
volatile bool lineTracing = false;
volatile int speed = 50;
volatile unsigned int angle = 30;
struct draw_bitmap_multiwindow_handle_t *handle_GUI_color = NULL;
struct draw_bitmap_multiwindow_handle_t *handle_GUI_gray = NULL;
struct draw_bitmap_multiwindow_handle_t *handle_GUI_black_white = NULL;
struct draw_bitmap_multiwindow_handle_t *handle_GUI_shrinked = NULL;
volatile bool showColor = false;
volatile bool showGray = false;
volatile bool showBlackWhite = false;
volatile bool showShrinked = false;

#define SHRUNK_W 64
#define SHRUNK_H 48

static struct pixel_format_RGB g_shrunk_data[SHRUNK_W* SHRUNK_H];
static pthread_mutex_t g_shrunk_data_lock = PTHREAD_MUTEX_INITIALIZER;
static struct pixel_format_RGB g_shrunk_data2[SHRUNK_W * SHRUNK_H];
static pthread_mutex_t g_shrunk_data_lock2 = PTHREAD_MUTEX_INITIALIZER;
struct pixel_format_RGB *master_buffer;


unsigned int global_scaled_width = 0;
unsigned int global_scaled_height = 0;
struct pixel_format_RGB *g_scaled_bw_data = NULL;

pthread_mutex_t g_scaled_bw_data_lock = PTHREAD_MUTEX_INITIALIZER;

struct thread_command
{
    uint8_t command;
    uint8_t argument;
};


FIFO_TYPE(struct thread_command, FIFO_LENGTH, fifo_t);


struct key_thread_param
{
  const char                    * name;
  struct fifo_t                 * key_fifo;
  bool                          * quit_flag;
};


struct motor_thread_param
{
  const char                    * name;
  volatile struct gpio_register * gpio;
  int                             pinNumber1;
  int                             pinNumber2;
  struct fifo_t                 * fifo;
  bool                          * quit_flag;
};


struct pwm_thread_param
{
  const char                    * name;
  volatile struct gpio_register * gpio;
  int                             pinNumber1;
  int                             pinNumber2;
  struct io_peripherals         * io;
  struct fifo_t                 * fifo;
  bool                          * quit_flag;
};

struct control_thread_param
{
  const char                    * name;
  struct fifo_t                 * key_fifo;
  struct fifo_t                 * motor1Fifo;
  struct fifo_t                 * motor2Fifo;
  struct fifo_t                 * pwmFifo;
  bool                          * quit_flag;
};

struct line_trace_thread_param
{
    const char                    * name;      
    struct fifo_t                 * motor1Fifo;
    struct fifo_t                 * motor2Fifo;
    struct fifo_t                 * pwmFifo;    
    bool                          * quit_flag;  
    volatile struct gpio_register * gpio;
    struct pixel_format_RGB       * scaled_RGB_data;
    unsigned int                    scaled_width;
    unsigned int                    scaled_height;
};

struct laser_thread_param
{
  const char                    * name;      
    struct fifo_t                 * motor1Fifo;
    struct fifo_t                 * motor2Fifo;
    struct fifo_t                 * pwmFifo;    
    bool                          * quit_flag;  
    struct pixel_format_RGB       * scaled_RGB_data;
    unsigned int                    scaled_width;
    unsigned int                    scaled_height;
};


struct camera_thread_param
{
    struct video_interface_handle_t * handle;
    unsigned char *scaled_data;
    struct pixel_format_RGB *scaled_RGB_data;
    unsigned int scaled_height;
    unsigned int scaled_width;
    bool *quit_flag;
    int argc;
    char **argv;
    struct draw_bitmap_multiwindow_handle_t *window;
};


#define SCALE_REDUCTION_PER_AXIS 2

void *KeyRead(void * arg)
{
  struct  key_thread_param * param = (struct key_thread_param *)arg;
  struct  thread_command cmd = {0, 0};
  int     keyhit = 0;
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait, ie. set interrupt
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  while (!*(param->quit_flag))
  {
    keyhit = get_pressed_key();  // read once every 10ms
    if ( keyhit != -1)
    {
      switch (keyhit)
      {
        case 10:      // 'Enter' key hit => no action
        {
                      // nothing placed on param->key_fifo
        }
        break;

        case 113:  // 'q' for quit
        {
          cmd.command = 113;
          cmd.argument = 0;
          if (!(FIFO_FULL( param->key_fifo )))
          {FIFO_INSERT( param->key_fifo, cmd );}
          else {printf( "key_fifo queue full\n" );}
          *param->quit_flag = true;
        }
        break;

        default:
        {
          cmd.command  = keyhit;  // other key hit
          cmd.argument = 0;
          if (!(FIFO_FULL( param->key_fifo )))
          {FIFO_INSERT( param->key_fifo, cmd );}
          else {printf( "key_fifo queue full\n" );}
        }

      }
    }

    wait_period( &timer_state, 10u ); /* 10ms */

  }

  printf( "KeyRead function done\n" );

  return NULL;

}


void *Control(void * arg)
{
  struct  control_thread_param * param = (struct control_thread_param *)arg;
  struct  thread_command cmd1 = {0, 0};  // copy of input, cmd from key_thread
  struct  thread_command cmd2 = {0, 0};  // copy of output, cmd to put on LED_thread
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms
             
  int ct = 0;
  bool modeSwitchFlag = false;
  // start 10ms timed wait, ie. set interrupt
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

//Pick up key presses and handle the cases.

  while (!*(param->quit_flag))
  {
    if (!(FIFO_EMPTY( param->key_fifo )))
    {
      FIFO_REMOVE( param->key_fifo, &cmd1 );  // read once every 10ms

            if (mode2)
            {
                switch (cmd1.command)
                {
                    case 'm':
                    {
                        modeSwitchFlag = true;
                      }
                        break;

                    case '1':
                    {
                        if (modeSwitchFlag)
                        {
                            mode1 = true;
                            mode2 = false;
                            mode3 = false;
                            stop = false;
                            forward = false;
                            backward = false;
                            lineTracing = false;
                            printf("Switched to Mode 1\n");
                            modeSwitchFlag = false;
                     
                      }
                    }
                        break;
                        case '3':
                        {
                            if (modeSwitchFlag)
                            {
                              mode1 = false;
                              mode2 = false;
                              mode3 = true;
                              stop = false;
                              forward = false;
                              backward = false;
                              lineTracing = false;
                              printf("Switched to Mode 3\n");
                              modeSwitchFlag = false;
                         
                          }
                        }
                            break;
                     
                    case 'i':     // 'i' increase pwm by 5
                    {
                        cmd2.command = '+'; //+ for +5
                     
                      if (!(FIFO_FULL( param->pwmFifo)))
                      {FIFO_INSERT( param->pwmFifo, cmd2 );}
                      else {printf( "pwm fifo queue full\n" );}
                    }
                    break;
                   
                    case 'j':     // 'j' decrease pwm by 5
                    {
                       
                        cmd2.command = '-'; //- for -5
                       
                      if (!(FIFO_FULL( param->pwmFifo)))
                      {FIFO_INSERT( param->pwmFifo, cmd2 );}
                      else {printf( "pwm fifo queue full\n" );}
                    }
                    break;
                    case 'o':     // 'o' increase angle by 5
                    {
                        if(angle != 180)
                        {
                          printf("Angle increased!");
                          angle += 10;
                          }
                        else{printf("Max Angle Reached!");}
                    }
                    break;
                   
                    case 'k':     // 'k' decrease angle by 5
                    {
                       if(angle != 5)
                        {
                          printf("Angle decreased!");
                          angle -= 10;
                          }
                        else{printf("Min Angle Reached!");}
                    }
                    break;

                    case 's':
                    {
                        if (!stop)
                        {
                            printf("Stopping in Mode 2!\n");
                            stop = true;
                            lineTracing = false;
                            cmd2.command = 's';
                            cmd2.argument = 0;
                            FIFO_INSERT(param->motor1Fifo, cmd2);
                            FIFO_INSERT(param->motor2Fifo, cmd2);
                        }
                        else
                        {
                            printf("Already stopped in Mode 2.\n");
                        }
                      }
                        break;

                    case 'w':
                    {
                        if (stop)
                        {
                            printf("Starting Line Tracing!\n");
                            stop = false;
                            lineTracing = true;
                        }
                      }
                        break;
                     
                    case 'c':
                    {
                        showColor = !showColor;
                    }
                    break;

                    case 'v':
                    {
                        showGray = !showGray;
                    }
                    break;

                    case 'b':
                    {
                        showBlackWhite = !showBlackWhite;
                    }
                    break;

                    case 'n':
                    {
                        showShrinked = !showShrinked;
                    }
                    break;


                    case 113:
                    {
                        cmd2.command = 113;
                        cmd2.argument = 0;
                        FIFO_INSERT(param->motor1Fifo, cmd2);
                        FIFO_INSERT(param->motor2Fifo, cmd2);
                      }
                        break;

                    default:
                    {
                        modeSwitchFlag = false;
                        printf("Ignored input: Mode 2 is active.\n");
                      }
                        break;
                }
            }
            else if(mode3)
            {
              switch (cmd1.command)
              {
                  case 'm':
                  {
                      modeSwitchFlag = true;
                    }
                      break;

                  case '1':
                  {
                      if (modeSwitchFlag)
                      {
                        mode1 = true;
                        mode2 = false;
                        stop = true;
                        forward = false;
                        backward = false;
                        lineTracing = false;
                        printf("Switched to Mode 1\n");
                        modeSwitchFlag = false;
                   
                    }
                  }
                  break;
                  case '2':
                  {
                      if (modeSwitchFlag)
                      {
                        mode1 = false;
                        mode2 = true;
                        mode3 = false;
                        printf("Switched to Mode 2\n");
                        modeSwitchFlag = false;
                   
                    }
                  }
                      break;
                   
                  case 'i':     // 'i' increase pwm by 5
                  {
                      cmd2.command = '+'; //+ for +5
                   
                    if (!(FIFO_FULL( param->pwmFifo)))
                    {FIFO_INSERT( param->pwmFifo, cmd2 );}
                    else {printf( "pwm fifo queue full\n" );}
                  }
                  break;
                 
                  case 'j':     // 'j' decrease pwm by 5
                  {
                     
                      cmd2.command = '-'; //- for -5
                     
                    if (!(FIFO_FULL( param->pwmFifo)))
                    {FIFO_INSERT( param->pwmFifo, cmd2 );}
                    else {printf( "pwm fifo queue full\n" );}
                  }
                  break;
                  case 'o':     // 'o' increase angle by 5
                  {
                      if(angle != 180)
                      {
                        printf("Angle increased!");
                        angle += 10;
                        }
                      else{printf("Max Angle Reached!");}
                  }
                  break;
                 
                  case 'k':     // 'k' decrease angle by 5
                  {
                     if(angle != 5)
                      {
                        printf("Angle decreased!");
                        angle -= 10;
                        }
                      else{printf("Min Angle Reached!");}
                  }
                  break;

                  case 's':
                  {
                      if (!stop)
                      {
                          printf("Stopping in Mode 3!\n");
                          stop = true;
                          lineTracing = false;
                          cmd2.command = 's';
                          cmd2.argument = 0;
                          FIFO_INSERT(param->motor1Fifo, cmd2);
                          FIFO_INSERT(param->motor2Fifo, cmd2);
                      }
                      else
                      {
                          printf("Already stopped in Mode 2.\n");
                      }
                    }
                      break;

                  case 'w':
                  {
                      if (stop)
                      {
                          printf("Starting Line Tracing!\n");
                          stop = false;
                          lineTracing = true;
                      }
                    }
                      break;
                   
                  case 'c':
                  {
                      showColor = !showColor;
                  }
                  break;

                  case 'v':
                  {
                      showGray = !showGray;
                  }
                  break;

                  case 'b':
                  {
                      showBlackWhite = !showBlackWhite;
                  }
                  break;

                  case 'n':
                  {
                      showShrinked = !showShrinked;
                  }
                  break;


                  case 113:
                  {
                      cmd2.command = 113;
                      cmd2.argument = 0;
                      FIFO_INSERT(param->motor1Fifo, cmd2);
                      FIFO_INSERT(param->motor2Fifo, cmd2);
                    }
                      break;

                  default:
                  {
                      modeSwitchFlag = false;
                      printf("Ignored input: Mode 3 is active.\n");
                    }
                      break;
              }


            }
            else{
                  switch (cmd1.command)
                  {
                    case 'm':
                    {
                    modeSwitchFlag = true;
                  }
                    break;

                  case '2':
                  {
                  if (modeSwitchFlag)
                  {
                    mode1 = false;
                    mode2 = true;
                    mode3 = false;
                    printf("Switched to Mode 2\n");
                    modeSwitchFlag = false;
                    }
                  }
                    break;
                    case '3':
                    {
                    if (modeSwitchFlag)
                    {
                      mode1 = false;
                      mode2 = false;
                      mode3 = true;
                      forward = false;
                      backward = false;
                      stop = false;
                      lineTracing = false;
                      printf("Switched to Mode 3\n");
                      modeSwitchFlag = false;
                      }
                    }
                      break;
                  
                    case 'c':
                    {
                        showColor = !showColor;
                    }
                    break;

                    case 'v':
                    {
                        showGray = !showGray;
                    }
                    break;

                    case 'b':
                    {
                          showBlackWhite = !showBlackWhite;
                    }
                    break;

                    case 'n':
                    {
                          showShrinked = !showShrinked;
                    }
                    break;
                  
                    case 's':     // stop engines
                    {
                      if(!stop)
                      {
                        printf("Stop!\n");
                        stop = true;
                        backward = false;
                        left = false;
                        right = false;
                        forward = false;
                        cmd2.command = 's'; //s for stop
                        cmd2.argument = 0;
                      }
                      else {printf( "Already stopped\n" );}
                      if (!(FIFO_FULL( param->motor1Fifo )) && !(FIFO_FULL( param->motor2Fifo )))
                      {FIFO_INSERT( param->motor1Fifo, cmd2 );
                        FIFO_INSERT( param->motor2Fifo, cmd2 );}
                      else {printf( "FIFO FULL\n" );}
                    }
                    break;

                    case 'w':     // 'w; for forward
                    {
                      if(!forward)
                      {
                        printf("Going forwards!\n");
                        backward = false;
                        left = false;
                        right = false;
                        stop = false;
                        forward = true;
                        cmd2.command = 'w'; //w for forward
                        cmd2.argument = 0;
                      }
                      else {printf( "Already going forward\n" );}
                    
                      if (!(FIFO_FULL( param->motor1Fifo )) && !(FIFO_FULL( param->motor2Fifo )) )
                      {FIFO_INSERT( param->motor1Fifo, cmd2 );
                        FIFO_INSERT( param->motor2Fifo, cmd2 );}
                      else {printf( "FIFO FULL\n" );}
                    }
                    break;

                    case 'x':     // x for backwards
                    {
                      if(!backward)
                      {
                        printf("Going backwards!\n");
                        forward = false;
                        left = false;
                        right = false;
                        stop = false;
                        backward = true;
                        cmd2.command = 'x'; //w for forward
                      }
                      if (!(FIFO_FULL( param->motor1Fifo)) && !(FIFO_FULL( param->motor2Fifo)))
                      {FIFO_INSERT( param->motor1Fifo, cmd2 );
                        FIFO_INSERT( param->motor2Fifo, cmd2 );}
                      else {printf( "Already going backwards\n" );}
                    }
                    break;
                  
                    case 'i':     // 'i' increase pwm by 5
                    {
                        cmd2.command = '+'; //+ for +5
                    
                      if (!(FIFO_FULL( param->pwmFifo)))
                      {FIFO_INSERT( param->pwmFifo, cmd2 );}
                      else {printf( "pwm fifo queue full\n" );}
                    }
                    break;
                  
                    case 'j':     // 'j' decrease pwm by 5
                    {
                      
                        cmd2.command = '-'; //- for -5
                      
                      if (!(FIFO_FULL( param->pwmFifo)))
                      {FIFO_INSERT( param->pwmFifo, cmd2 );}
                      else {printf( "pwm fifo queue full\n" );}
                    }
                    break;
                    case 'o':     // 'o' increase angle by 5
                    {
                        if(angle != 180)
                        {
                          printf("Angle increased!");
                          angle += 10;
                        }
                        else{printf("Max Angle Reached!");}
                    }
                    break;
                  
                    case 'k':     // 'k' decrease angle by 5
                    {
                      
                        if(angle != 5)
                        {
                          printf("Angle decreased!");
                        angle -= 10;
                        }
                        else{printf("Min Angle Reached!");}
                    }
                    break;
                  
                    case 'a':     // 'a' for left turn
                    {
                        if(stop == false && forward == false && backward == false){stop=true;}
                        left = true;
                        if(stop == true)
                          {
                            if(!(FIFO_FULL( param->motor1Fifo )) && !(FIFO_FULL( param->motor2Fifo )) )
                            {//stopped turn
                              printf("Turning right!\n");
                              cmd2.command = 't';
                              FIFO_INSERT( param->pwmFifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              wait_period( &timer_state, angle );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                            }
                            else {printf( "fifo queue full\n" );}
                          }
                          else if(forward == true)
                          {
                            if(!(FIFO_FULL( param->motor1Fifo )) && !(FIFO_FULL( param->motor2Fifo )) )
                            {//forwards turn
                              printf("Turning right!\n");
                              cmd2.command = 't';
                              FIFO_INSERT( param->pwmFifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              wait_period( &timer_state, angle );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                            }
                            else {printf( "fifo queue full\n" );}
                          }
                          else if(backward == true)
                          {
                            if(!(FIFO_FULL( param->motor1Fifo )) && !(FIFO_FULL( param->motor2Fifo )) )
                            { //backwards turn
                              printf("Turning right!\n");
                              cmd2.command = 't';
                              FIFO_INSERT( param->pwmFifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              wait_period( &timer_state, angle );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                            }
                            else {printf( "fifo queue full\n" );}
                          }
                      
                    }
                    break;
                  
                    case 'd':     // 'd' for right turn
                    {
                        if(stop == false && forward == false && backward == false){stop=true;}
                        right = true;
                        if(stop == true)
                          {
                            if (!(FIFO_FULL( param->motor1Fifo )) && !(FIFO_FULL( param->motor2Fifo )) )
                            { //Stopped turn
                              printf("Turning left!\n");
                              cmd2.command = 't';
                              FIFO_INSERT( param->pwmFifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              wait_period( &timer_state, angle );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                            }
                            else {printf( "fifo queue full\n" );}
                          }
                          else if(forward == true)
                          { //Forwards turn
                            if (!(FIFO_FULL( param->motor1Fifo )) && !(FIFO_FULL( param->motor2Fifo )) )
                            {
                              printf("Turning left!\n");
                              cmd2.command = 't';
                              FIFO_INSERT( param->pwmFifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              wait_period( &timer_state, angle );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                            }
                            else {printf( "fifo queue full\n" );}
                          } //Backwards turn
                          else if(backward == true)
                          {
                            if (!(FIFO_FULL( param->motor1Fifo )) && !(FIFO_FULL( param->motor2Fifo )) )
                            {
                              printf("Turning left!\n");
                              cmd2.command = 't';
                              FIFO_INSERT( param->pwmFifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              wait_period( &timer_state, angle );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                            }
                            else {printf( "fifo queue full\n" );}
                          }
                    }
                    break;

                    case 113:  // 'q' for quit both red and green LED threads
                    {
                      cmd2.command = 113;
                      cmd2.argument = 0;
                      if (!(FIFO_FULL( param->motor1Fifo )) && !(FIFO_FULL( param->motor2Fifo )) )
                      {FIFO_INSERT( param->motor1Fifo, cmd2 );
                        FIFO_INSERT( param->motor2Fifo, cmd2 );}
                      else {printf( "FIFO FULL\n" );}
                    }
                    break;

                    default:
                    {
                      modeSwitchFlag = false;
                      printf("Ignored input: Mode 2 is active.\n");

                    }
                    break;

                  }
    }
    }

    wait_period( &timer_state, 10u ); /* 10ms */

  }
  cmd2.command = 's';
  FIFO_INSERT(param->motor1Fifo, cmd2);
  FIFO_INSERT(param->motor2Fifo, cmd2);
  printf( "Control function done\n" );

  return NULL;

}

void *LaserThread(void *arg)
{
  struct line_trace_thread_param *param = (struct line_trace_thread_param *)arg;
  struct thread_command cmd = {0, 0};
  struct timespec timer_state;
  wait_period_initialize(&timer_state);
  
  
  //SAME AS VISUAL CROSSHAIR
  //rows
  int topEnd = 20;
  int midEnd = 28;
  //Columns
  int leftEnd   = 24;
  int centerEnd = 40;


  while (!*(param->quit_flag))
  {
      if (mode3 && !stop)
      {
          //Default no laser
          LaserBox foundBox = BOX_NONE;
          bool foundLaser = false;

          //Scan for laser untill found
          pthread_mutex_lock(&g_shrunk_data_lock);
          for (int y = 0; y < SHRUNK_H && !foundLaser; y++)
          {
              for (int x = 0; x < SHRUNK_W && !foundLaser; x++)
              {
                //Find laser
                  if (g_shrunk_data[y * SHRUNK_W + x].R == 255)
                  {
                      //top row
                      if (y < topEnd) {
                          if (x < leftEnd)
                          { foundBox = BOX_TOP_LEFT;}
                          else if (x < centerEnd)
                          {foundBox = BOX_TOP_CENTER;}
                          else
                          {foundBox = BOX_TOP_RIGHT;}
                      }
                      else if (y < midEnd) {
                          // middle row
                          if (x < leftEnd)
                          {foundBox = BOX_MID_LEFT;}
                          else if (x < centerEnd)
                          {foundBox = BOX_MID_CENTER;}
                          else
                          {foundBox = BOX_MID_RIGHT;}
                      }
                      else {
                          //Bottom row
                          if (x < leftEnd)
                          {foundBox = BOX_BOTTOM_LEFT;}
                          else if (x < centerEnd)      
                          {foundBox = BOX_BOTTOM_CENTER;}
                          else                         
                          {foundBox = BOX_BOTTOM_RIGHT;}
                      }
                      foundLaser = true;
                  }
              }
          }
          pthread_mutex_unlock(&g_shrunk_data_lock);

          //Depending on box pick command
          switch (foundBox)
          {
              case BOX_NONE:
                  //No laser do nothing
                  cmd.command = 's';
                  FIFO_INSERT(param->motor1Fifo, cmd);
                  FIFO_INSERT(param->motor2Fifo, cmd);
                  break;

              //In left boxes turn left
              case BOX_TOP_LEFT:
              case BOX_MID_LEFT:
              case BOX_BOTTOM_LEFT:
                  printf("Laser in Left\n");
                  if (!(FIFO_FULL(param->motor1Fifo)) && !(FIFO_FULL(param->motor2Fifo))) {
                      cmd.command = 's';
                      FIFO_INSERT(param->motor1Fifo, cmd);
                      FIFO_INSERT(param->motor2Fifo, cmd);
                      wait_period(&timer_state, 60u);

                      cmd.command = 't';
                      FIFO_INSERT(param->pwmFifo, cmd);
                      cmd.command = 'x';
                      FIFO_INSERT(param->motor1Fifo, cmd);
                      cmd.command = 'w';
                      FIFO_INSERT(param->motor2Fifo, cmd);
                      wait_period(&timer_state, 30u);
                    }
                  break;

              //In right boxes turn right
              case BOX_TOP_RIGHT:
              case BOX_MID_RIGHT:
              case BOX_BOTTOM_RIGHT:
                  printf("Laser in Right\n");
                  if (!(FIFO_FULL(param->motor1Fifo)) && !(FIFO_FULL(param->motor2Fifo))) {
                      cmd.command = 's';
                      FIFO_INSERT(param->motor1Fifo, cmd);
                      FIFO_INSERT(param->motor2Fifo, cmd);
                      wait_period(&timer_state, 60u);

                      cmd.command = 't';
                      FIFO_INSERT(param->pwmFifo, cmd);
                      cmd.command = 'w';
                      FIFO_INSERT(param->motor1Fifo, cmd);
                      cmd.command = 'x';
                      FIFO_INSERT(param->motor2Fifo, cmd);
                      wait_period(&timer_state, 30u);

                  }
                  break;

              //In top center go forward
              case BOX_TOP_CENTER:
                  printf("Laser in Top Center\n");
                  if (!(FIFO_FULL(param->motor1Fifo)) && !(FIFO_FULL(param->motor2Fifo))) {
                      cmd.command = 'w';
                      FIFO_INSERT(param->motor1Fifo, cmd);
                      FIFO_INSERT(param->motor2Fifo, cmd);
                      wait_period(&timer_state, 50u);
                  }
                  break;

              //Laser centered do not move.
              case BOX_MID_CENTER:
                  printf("Laser in Middle\n");
                  if (!(FIFO_FULL(param->motor1Fifo)) && !(FIFO_FULL(param->motor2Fifo))) {
                    cmd.command = 's';
                    FIFO_INSERT(param->motor1Fifo, cmd);
                    FIFO_INSERT(param->motor2Fifo, cmd);
                    wait_period(&timer_state, 50u);
                }
                  break;
              
                  //Laser close to car move backwards
              case BOX_BOTTOM_CENTER:
                  printf("Laser in Bottom Center\n");
                  if (!(FIFO_FULL(param->motor1Fifo)) && !(FIFO_FULL(param->motor2Fifo))) {
                      cmd.command = 'x';
                      FIFO_INSERT(param->motor1Fifo, cmd);
                      FIFO_INSERT(param->motor2Fifo, cmd);
                  }
                  break;
                }
      }
      else{                    
        cmd.command = 's';
        FIFO_INSERT(param->motor1Fifo, cmd);
        FIFO_INSERT(param->motor2Fifo, cmd);}

      wait_period(&timer_state, 10u);
        
      }
    
  printf("LaserThread done\n");
  return NULL;

}

void *LineTraceThread(void *arg)
{
    struct line_trace_thread_param *param = (struct line_trace_thread_param *)arg;
    struct thread_command cmd2 = {0, 0};
    struct timespec timer_state;
    wait_period_initialize(&timer_state);
    
    //Scan bottom rows
    int startRow = 20;
    int endRow = SHRUNK_H - 1; 
    
    int halfRegion = SHRUNK_W / 4;
    int centerCol = SHRUNK_W / 2;
    
    //Threshold for center
    int centerThreshold = 3;
    
    while (!*(param->quit_flag))
    {
        if (mode2 && !stop)
        {
            int leftCount = 0;
            int rightCount = 0;
            
            pthread_mutex_lock(&g_shrunk_data_lock2);
            for (int row = startRow; row < endRow; row++)
            {
                for (int x = centerCol - halfRegion; x < centerCol; x++) {
                    if (g_shrunk_data2[row * SHRUNK_W + x].R == 0)
                        leftCount++;
                }
                for (int x = centerCol; x < centerCol + halfRegion; x++) {
                    if (g_shrunk_data2[row * SHRUNK_W + x].R == 0)
                        rightCount++;
                }
            }
            pthread_mutex_unlock(&g_shrunk_data_lock2);

                //Go forward if in threshold
            if (abs(leftCount - rightCount) <= centerThreshold && !forward) {
                if (!(FIFO_FULL(param->motor1Fifo)) && !(FIFO_FULL(param->motor2Fifo))) {
                    forward = true;
                    cmd2.command = 'w';
                    FIFO_INSERT(param->motor1Fifo, cmd2);
                    FIFO_INSERT(param->motor2Fifo, cmd2);
                }
            }
            else if (leftCount > rightCount) {
                //Turn left
                if (!(FIFO_FULL(param->motor1Fifo)) && !(FIFO_FULL(param->motor2Fifo))) {
                    forward = false;
                    cmd2.command = 's';
                    FIFO_INSERT(param->motor1Fifo, cmd2);
                    FIFO_INSERT(param->motor2Fifo, cmd2);
                    wait_period(&timer_state, 60u);
                    
                    cmd2.command = 'l';
                    FIFO_INSERT(param->pwmFifo, cmd2);
                    cmd2.command = 'x';
                    FIFO_INSERT(param->motor1Fifo, cmd2);
                    cmd2.command = 'w';
                    FIFO_INSERT(param->motor2Fifo, cmd2);
                    wait_period(&timer_state, 150u);
                    
                    cmd2.command = 'w';
                    FIFO_INSERT(param->motor1Fifo, cmd2);
                    FIFO_INSERT(param->motor2Fifo, cmd2);
                }
            }
            else { //(Right count higher)
                //Turn right
                if (!(FIFO_FULL(param->motor1Fifo)) && !(FIFO_FULL(param->motor2Fifo))) {
                    forward = false;
                    cmd2.command = 's';
                    FIFO_INSERT(param->motor1Fifo, cmd2);
                    FIFO_INSERT(param->motor2Fifo, cmd2);
                    wait_period(&timer_state, 60u);
                    
                    cmd2.command = 'l';
                    FIFO_INSERT(param->pwmFifo, cmd2);
                    cmd2.command = 'w';
                    FIFO_INSERT(param->motor1Fifo, cmd2);
                    cmd2.command = 'x';
                    FIFO_INSERT(param->motor2Fifo, cmd2);
                    wait_period(&timer_state, 150u);
                    
                    cmd2.command = 'w';
                    FIFO_INSERT(param->motor1Fifo, cmd2);
                    FIFO_INSERT(param->motor2Fifo, cmd2);
                }
            }
        }
        wait_period(&timer_state, 10u);
    }
    
    printf("LineTrace function done\n");
    return NULL;
}


      


void *Motor1Thread(void * arg)
{
  struct  motor_thread_param * param = (struct motor_thread_param *)arg;
  struct  thread_command cmd = {0, 0};
  bool    busy = false;
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */
 
  while (!*(param->quit_flag))
  {
    param->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
    param->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
    param->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    param->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;
    param->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    param->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;
    if (!busy)
    {
      if (!(FIFO_EMPTY( param->fifo )))
      {
        FIFO_REMOVE( param->fifo, &cmd );  // read once every 10ms
        //Adding an always stop.
        //This is a short pause before any command goes through, the 's' case will just be used for prompting the user they are stopped (because it will stay stopped anyways).
        switch (cmd.command)
        {
          case 's':  // Stop
          {
              GPIO_SET(param->gpio, 12);
              GPIO_SET(param->gpio, 13);
              GPIO_CLR(param->gpio, 05);
              GPIO_CLR(param->gpio, 06);
              if(right == true || left == true)
              {
                right = false;
                left = false;
                }
             
          }
          break;

          case 'w': //Forward
          {
            printf("Motor thread %s processing 'w'\n", param->name);
              GPIO_SET(param->gpio, 12);
              GPIO_SET(param->gpio, 13);
              GPIO_CLR(param->gpio, 05);
              GPIO_SET(param->gpio, 06);
              if(right == true || left == true)
              {
                right = false;
                left = false;
                }
         
          }
          break;
         
          case 'x':  // backwards
          {
              GPIO_SET(param->gpio, 12);
              GPIO_SET(param->gpio, 13);
              GPIO_SET(param->gpio, 05);
              GPIO_CLR(param->gpio, 06);
              if(right == true || left == true)
              {
                right = false;
                left = false;
                }
          }
          break;

          case 113:  // 'q' for quit the program
          {
            // done = true;
            printf( " %s quit\n", param->name);
          }
          break;

          case 0:   // Delay time command,
          {
            if (cmd.argument != 0)
            {busy = true;}
            else
            { /* Delay = 0 => no operation needed */ }
          }
          break;

          default:
          {
 
            printf( " %s wrong cmd\n", param->name);
          }

        }
      }
    }
    else
    {
      if (cmd.argument != 0)
      {cmd.argument = cmd.argument - 1;}
      else
      {busy = false;}
    }

    wait_period( &timer_state, 10u ); /* 10ms */

  }

  printf( "%s function done\n", param->name );
  return NULL;

}

void *Motor2Thread(void * arg)
{
  struct  motor_thread_param * param = (struct motor_thread_param *)arg;
  struct  thread_command cmd = {0, 0};
  bool    busy = false;
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */
 
  while (!*(param->quit_flag))
  {
    param->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
    param->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
    param->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    param->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;
    param->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    param->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;
    if (!busy)
    {
      if (!(FIFO_EMPTY( param->fifo )))
      {
        FIFO_REMOVE( param->fifo, &cmd );  // read once every 10ms
        switch (cmd.command)
        {
          case 's':  // Stop
          {
              GPIO_SET(param->gpio, 12);
              GPIO_SET(param->gpio, 13);
              GPIO_CLR(param->gpio, 22);
              GPIO_CLR(param->gpio, 23);
              if(right == true || left == true)
              {
                right = false;
                left = false;
                }
          }
          break;

          case 'w': //Forward
          {
              printf("Motor thread %s processing 'w'\n", param->name);
              GPIO_SET(param->gpio, 12);
              GPIO_SET(param->gpio, 13);
              GPIO_SET(param->gpio, 22);
              GPIO_CLR(param->gpio, 23);
              if(right == true || left == true)
              {
                right = false;
                left = false;
                }
         
          }
          break;
         
          case 'x':  // backwards
          {
              GPIO_SET(param->gpio, 12);
              GPIO_SET(param->gpio, 13);
              GPIO_CLR(param->gpio, 22);
              GPIO_SET(param->gpio, 23);
              if(right == true || left == true)
              {
                right = false;
                left = false;
                }
          }
          break;

          case 113:  // 'q' for quit the program
          {
            // done = true;
            printf( " %s quit\n", param->name);
          }
          break;

          case 0:   // Delay time command,
          {
            if (cmd.argument != 0)
            {busy = true;}
            else
            { /* Delay = 0 => no operation needed */ }
          }
          break;

          default:
          {
 
            printf( " %s wrong cmd\n", param->name);
          }

        }
      }
    }
    else
    {
      if (cmd.argument != 0)
      {cmd.argument = cmd.argument - 1;}
      else
      {busy = false;}
    }

    wait_period( &timer_state, 10u ); /* 10ms */

  }

  printf( "%s function done\n", param->name );

  return NULL;

}


void *SpeedThread(void * arg)
{
  struct  pwm_thread_param * param = (struct pwm_thread_param *)arg;
  struct  thread_command cmd = {0, 0};
  bool    busy = false;
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */
 
  while (!*(param->quit_flag))
  {
    param->io->pwm->DAT1 = speed;
    param->io->pwm->DAT2 = speed;
    if (!busy)
    {
      if (!(FIFO_EMPTY( param->fifo )))
      {
        FIFO_REMOVE( param->fifo, &cmd );  // read once every 10ms
        switch (cmd.command)
        {
          case 't':  //T for turn, we change the speed to 100 and wait the same period the turn takes.
          {
            int temp = speed;
            speed = 100;
            param->io->pwm->DAT1 = speed;
            param->io->pwm->DAT2 = speed;
            printf("Speed for turn. Speed is now: %d%\n", speed);
            wait_period( &timer_state, 75u );
            speed = temp;
          }
          break;
          case 'l':  //L for  line trace turn, we change the speed to 100 and wait the same period the turn takes.
          {
            int temp = speed;
            speed = 40;
            param->io->pwm->DAT1 = speed;
            param->io->pwm->DAT2 = speed;
            printf("Speed for turn. Speed is now: %d%\n", speed);
            wait_period( &timer_state, 75u );
            speed = temp;
          }
          break;
         
          case '+':  // Increase in speed
          {
            if(speed == 100)
            {
             printf("MAX SPEED REACHED!\n") ;
            }
            else
            {
              speed += 5;
              param->io->pwm->DAT1 = speed;
              param->io->pwm->DAT2 = speed;
            }
            printf("Increasing speed. Speed is now: %d%\n", speed);
          }
          break;

          case '-':  // Decrease in speed
          {
            if(speed == 00)
            {
             printf("MIN SPEED REACHED!\n");
            }
            else
            {
              speed -= 5;
              param->io->pwm->DAT1 = speed;
              param->io->pwm->DAT2 = speed;
            }
            printf("Decreasing speed. Speed is now: %d%\n", speed);
          }
          break;
         
          case 113:  // 'q' for quit the program
          {
            // done = true;
            printf( " %s quit\n", param->name);
          }
          break;

          case 0:   // Delay time command,
          {
            if (cmd.argument != 0)
            {busy = true;}
            else
            { /* Delay = 0 => no operation needed */ }
          }
          break;

          default:
          {
 
            printf( " %s wrong cmd\n", param->name);
          }

        }
      }
    }
    else
    {
      if (cmd.argument != 0)
      {cmd.argument = cmd.argument - 1;}
      else
      {busy = false;}
    }
   

    wait_period( &timer_state, 10u ); /* 10ms */

  }

  printf( "%s function done\n", param->name );

  return NULL;

}


//Takes in our image buffer width and heigh than creates a cross hair.
void overlayCrosshair(struct pixel_format_RGB *buffer, unsigned int width, unsigned int height) 
{
    unsigned int centerX = width / 2;
    unsigned int centerY = height / 2;
    for (unsigned int x = 0; x < width; x++) {
        buffer[centerY * width + x].R = 0;
        buffer[centerY * width + x].G = 255;
        buffer[centerY * width + x].B = 0;
    }
    for (unsigned int y = 0; y < height; y++) {
        buffer[y * width + centerX].R = 0;
        buffer[y * width + centerX].G = 255;
        buffer[y * width + centerX].B = 0;
    }
}

void overlay3x3Grid(struct pixel_format_RGB *buffer, unsigned int width, unsigned int height)
{
    int topEnd    = 20;
    int midEnd    = 28;
    int leftEnd   = 24;
    int centerEnd = 40;  

    if (topEnd    >= (int)height) topEnd    = height - 1;
    if (midEnd    >= (int)height) midEnd    = height - 1;
    if (leftEnd   >= (int)width ) leftEnd   = width  - 1;
    if (centerEnd >= (int)width ) centerEnd = width  - 1;

    for (unsigned int x = 0; x < width; x++) {
        buffer[topEnd * width + x].R = 255;
        buffer[topEnd * width + x].G = 0;
        buffer[topEnd * width + x].B = 0;
    }
    
    for (unsigned int x = 0; x < width; x++) {
        buffer[midEnd * width + x].R = 255;
        buffer[midEnd * width + x].G = 0;
        buffer[midEnd * width + x].B = 0;
    }


    for (unsigned int y = 0; y < height; y++) {
        buffer[y * width + leftEnd].R = 255;
        buffer[y * width + leftEnd].G = 0;
        buffer[y * width + leftEnd].B = 0;
    }

    for (unsigned int y = 0; y < height; y++) {
        buffer[y * width + centerEnd].R = 255;
        buffer[y * width + centerEnd].G = 0;
        buffer[y * width + centerEnd].B = 0;
    }
}



void *DisplayThread(void *arg)
{
    struct camera_thread_param *param = (struct camera_thread_param *)arg;
    struct timespec timer_state;
    wait_period_initialize(&timer_state);
    
    size_t numPixels = param->scaled_width * param->scaled_height;
    
    
    struct draw_bitmap_multiwindow_handle_t * handle_GUI_color = draw_bitmap_create_window(param->scaled_width, param->scaled_height);
    struct draw_bitmap_multiwindow_handle_t * handle_GUI_gray = draw_bitmap_create_window(param->scaled_width, param->scaled_height);
    struct draw_bitmap_multiwindow_handle_t * handle_GUI_bw = draw_bitmap_create_window(param->scaled_width, param->scaled_height);
    struct draw_bitmap_multiwindow_handle_t * handle_GUI_shrinked = draw_bitmap_create_window(param->scaled_width, param->scaled_height);
    struct draw_bitmap_multiwindow_handle_t * handle_GUI_shrinked2 = draw_bitmap_create_window(param->scaled_width, param->scaled_height);
    
    struct pixel_format_RGB *colorBuffer  = malloc(numPixels * sizeof(struct pixel_format_RGB));
    struct pixel_format_RGB *grayBuffer   = malloc(numPixels * sizeof(struct pixel_format_RGB));
    struct pixel_format_RGB *bwBuffer     = malloc(numPixels * sizeof(struct pixel_format_RGB));
    struct pixel_format_RGB *shrinkBuffer = malloc(numPixels * sizeof(struct pixel_format_RGB));
    struct pixel_format_RGB *shrinkBuffer2 = malloc(numPixels * sizeof(struct pixel_format_RGB));
  
    const int targetWidth  = 64;
    const int targetHeight = 48;
    struct pixel_format_RGB *shrinkedData = malloc(targetWidth * targetHeight * sizeof(struct pixel_format_RGB));
    struct pixel_format_RGB *shrinkedData2 = malloc(targetWidth * targetHeight * sizeof(struct pixel_format_RGB));
    struct pixel_format_RGB *upscaledData = malloc(param->scaled_width * param->scaled_height * sizeof(struct pixel_format_RGB));
    struct pixel_format_RGB *upscaledData2 = malloc(param->scaled_width * param->scaled_height * sizeof(struct pixel_format_RGB));
    int scaleFactor = param->scaled_width / targetWidth;
    if (scaleFactor == 0)
        scaleFactor = 1;

    while (!*(param->quit_flag))
    {
      
        if(showColor == false)
        {

        }
        else
        {

        }
        if(showGray == false)
        {

        }
        else
        {
          
        }
        if(showBlackWhite == false)
        {

        }
        else
        {
          
        }
        if(showShrinked == false)
        {

        }
        else
        {
          
        }
        
        memcpy(colorBuffer,  param->scaled_RGB_data, numPixels * sizeof(struct pixel_format_RGB));
        memcpy(grayBuffer,   param->scaled_RGB_data, numPixels * sizeof(struct pixel_format_RGB));
        memcpy(bwBuffer,     param->scaled_RGB_data, numPixels * sizeof(struct pixel_format_RGB));
        memcpy(shrinkBuffer, param->scaled_RGB_data, numPixels * sizeof(struct pixel_format_RGB));
        memcpy(shrinkBuffer2, param->scaled_RGB_data, numPixels * sizeof(struct pixel_format_RGB));


            
        struct image_t localImage;
        if (video_interface_get_image(param->handle, &localImage))
        {
            scale_image_data((struct pixel_format_RGB *)&localImage,param->handle->configured_height,param->handle->configured_width,param->scaled_RGB_data,SCALE_REDUCTION_PER_AXIS,SCALE_REDUCTION_PER_AXIS);
        }
        
        if (showColor && handle_GUI_color != NULL)
        {
            overlayCrosshair(colorBuffer, param->scaled_width, param->scaled_height);
            draw_bitmap_display(handle_GUI_color, colorBuffer);
        }

        if (showGray && handle_GUI_gray != NULL)
        {
            for (size_t i = 0; i < numPixels; i++)
            {
                uint8_t gray = (grayBuffer[i].R + grayBuffer[i].G + grayBuffer[i].B) / 3;
                grayBuffer[i].R = gray;
                grayBuffer[i].G = gray;
                grayBuffer[i].B = gray;
            }
            draw_bitmap_display(handle_GUI_gray, grayBuffer);
        }

        if (showBlackWhite && handle_GUI_bw != NULL)
        {
            for (size_t i = 0; i < numPixels; i++)
            {
                uint8_t gray = (bwBuffer[i].R + bwBuffer[i].G + bwBuffer[i].B) / 3;
                uint8_t bw = (gray > 128) ? 255 : 0;
                bwBuffer[i].R = bw;
                bwBuffer[i].G = bw;
                bwBuffer[i].B = bw;
            }
          
            draw_bitmap_display(handle_GUI_bw, bwBuffer);
        }

        if (showShrinked && handle_GUI_shrinked != NULL)
        {
            for (int y = 0; y < targetHeight; y++) {
                for (int x = 0; x < targetWidth; x++) {
                    int x2 = x * scaleFactor;
                    int y2 = y * scaleFactor;
                    if (x2 >= (int)param->scaled_width)  x2 = param->scaled_width - 1;
                    if (y2 >= (int)param->scaled_height) y2 = param->scaled_height - 1;
                    
                    struct pixel_format_RGB spixel = shrinkBuffer[y2 * param->scaled_width + x2];
                    uint8_t r = spixel.R, g = spixel.G, b = spixel.B;
                    uint8_t max_val = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
                    uint8_t min_val = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);

                    float saturation = (max_val > 0) ? ((float)(max_val - min_val)) / max_val : 0.0;

                    uint8_t bw = (max_val > 220 && (max_val - min_val) > 28 && saturation > 0.27) ? 255 : 0;

                    shrinkedData[y * targetWidth + x].R = bw;
                    shrinkedData[y * targetWidth + x].G = bw;
                    shrinkedData[y * targetWidth + x].B = bw;
                }
            }
            pthread_mutex_lock(&g_shrunk_data_lock);
            memcpy(g_shrunk_data, shrinkedData, targetWidth * targetHeight * sizeof(struct pixel_format_RGB));
            pthread_mutex_unlock(&g_shrunk_data_lock);
            overlay3x3Grid(shrinkedData, 64, 48);
            for (int y = 0; y < (int)param->scaled_height; y++) {
                for (int x = 0; x < (int)param->scaled_width; x++) {
                    int u = x * targetWidth / param->scaled_width;
                    int v = y * targetHeight / param->scaled_height;
                    if (u >= targetWidth)  u = targetWidth - 1;
                    if (v >= targetHeight) v = targetHeight - 1;
                    upscaledData[y * param->scaled_width + x] = shrinkedData[v * targetWidth + u];
                }
            }
            draw_bitmap_display(handle_GUI_shrinked, upscaledData);
            
            for (int y = 0; y < targetHeight; y++) {
                for (int x = 0; x < targetWidth; x++) {
                    int x2 = x * scaleFactor;
                    int y2 = y * scaleFactor;
                    if (x2 >= (int)param->scaled_width)  x2 = param->scaled_width - 1;
                    if (y2 >= (int)param->scaled_height) y2 = param->scaled_height - 1;
                    
                    struct pixel_format_RGB spixel = shrinkBuffer2[y2 * param->scaled_width + x2];
                    uint8_t gray = (spixel.R + spixel.G + spixel.B) / 3;
                    uint8_t bw   = (gray > 30) ? 255 : 0;
                    shrinkedData2[y * targetWidth + x].R = bw;
                    shrinkedData2[y * targetWidth + x].G = bw;
                    shrinkedData2[y * targetWidth + x].B = bw;
                }
            }
            
            pthread_mutex_lock(&g_shrunk_data_lock2);
            memcpy(g_shrunk_data2, shrinkedData2, targetWidth * targetHeight * sizeof(struct pixel_format_RGB));
            pthread_mutex_unlock(&g_shrunk_data_lock2);
            
            for (int y = 0; y < (int)param->scaled_height; y++) {
                for (int x = 0; x < (int)param->scaled_width; x++) {
                    int u = x * targetWidth / param->scaled_width;
                    int v = y * targetHeight / param->scaled_height;
                    if (u >= targetWidth)  u = targetWidth - 1;
                    if (v >= targetHeight) v = targetHeight - 1;
                    upscaledData2[y * param->scaled_width + x] = shrinkedData2[v * targetWidth + u];
                }
            }
            draw_bitmap_display(handle_GUI_shrinked2, upscaledData2);
        }
        
        wait_period(&timer_state, 10);
    }
    free(upscaledData);
    free(colorBuffer);
    free(grayBuffer);
    free(bwBuffer);
    free(shrinkBuffer);
    free(shrinkedData);
    free(shrinkedData2);
    free(upscaledData2);
    return NULL;
}



int main( int argc, char * argv[] )
{
  struct io_peripherals *io;
  struct video_interface_handle_t * handle;
  
  
  //Create 5 threads 2 for the motors one for the speed, one for keys and one for control. All except key need their respective fifos, and thread paramaters
  pthread_t tMotor1;
  pthread_t tMotor2;
  pthread_t tPWM;
  pthread_t tk;
  pthread_t tc;
  pthread_t tLineTrace;
  pthread_t tLaser;
  pthread_t tDisplay;
  struct fifo_t key_fifo   = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t motor1Fifo    = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t motor2Fifo    = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t pwmFifo = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  bool quit_flag = false;
  io = import_registers();
  handle = video_interface_open( "/dev/video0" );
  video_interface_set_mode_auto(handle);
  draw_bitmap_start(argc, argv);

  unsigned int scaled_width = handle->configured_width / SCALE_REDUCTION_PER_AXIS;
  unsigned int scaled_height = handle->configured_height / SCALE_REDUCTION_PER_AXIS;
  global_scaled_width = scaled_width;
  global_scaled_height = scaled_height;
  unsigned char *scaled_data = (unsigned char *)malloc(sizeof(struct image_t) / (SCALE_REDUCTION_PER_AXIS * SCALE_REDUCTION_PER_AXIS));
  g_scaled_bw_data = malloc(scaled_width * scaled_height * sizeof(struct pixel_format_RGB));
  struct pixel_format_RGB *scaled_RGB_data = malloc(scaled_width * scaled_height * sizeof(struct pixel_format_RGB));
  struct motor_thread_param     motor1Param  = {"m1", NULL, 05, 06,  &motor1Fifo, &quit_flag};
  struct motor_thread_param     motor2Param  = {"m2", NULL, 22, 23,  &motor2Fifo, &quit_flag};
  struct pwm_thread_param       pwmParam    = {"pwm", NULL, 12, 13, io, &pwmFifo, &quit_flag};  
  struct key_thread_param       key_param   = {"key", &key_fifo, &quit_flag};
  struct control_thread_param   con_param   = {"con", &key_fifo, &motor1Fifo, &motor2Fifo, &pwmFifo, &quit_flag};
  struct line_trace_thread_param lineTraceParam = {"linetrace", &motor1Fifo, &motor2Fifo, &pwmFifo, &quit_flag, NULL, scaled_RGB_data, scaled_height, scaled_width};
  struct camera_thread_param cameraParam = {handle, scaled_data, scaled_RGB_data, scaled_height, scaled_width, &quit_flag, argc, argv, handle_GUI_color};
  struct laser_thread_param laserParam = {"laser", &motor1Fifo, &motor2Fifo, &pwmFifo, &quit_flag, NULL, scaled_RGB_data, scaled_height, scaled_width};
  
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );
    pwm_setup(100, io);
    enable_pwm_clock(io->cm, io->pwm);

   
    lineTraceParam.gpio = io->gpio;
    motor1Param.gpio = io->gpio;
    motor2Param.gpio = io->gpio;
    pwmParam.gpio = io->gpio;
   
   
    printf("\n\n\n Welcome!\n\n w: forward\n s: stop\n x:backward\n a: left \n d: right \n o: increase degrees \n k: decrease degrees \n \n i: power up 5%\n j: power down 5%\nc: color image\nv: gray image\nb: black white image\nn: shrink image (black white)\n\n\n");

    // Create three threads our threase and then join them once the q command is hit
    pthread_create(&tMotor1, NULL, Motor1Thread, (void *)&motor1Param);
    pthread_create(&tMotor2, NULL, Motor2Thread, (void *)&motor2Param);
    pthread_create(&tPWM, NULL, SpeedThread, (void *)&pwmParam);
    pthread_create(&tk, NULL, KeyRead,   (void *)&key_param);
    pthread_create(&tc, NULL, Control,   (void *)&con_param);
    pthread_create(&tLineTrace, NULL, LineTraceThread, (void *)&lineTraceParam);
    pthread_create(&tDisplay, NULL, DisplayThread, (void *)&cameraParam);
    pthread_create(&tLaser, NULL, LaserThread, (void *)&laserParam);
    // Join threads
    pthread_join(tMotor1, NULL);
    pthread_join(tMotor2, NULL);
    pthread_join(tPWM, NULL);
    pthread_join(tk, NULL);
    pthread_join(tc, NULL);
    pthread_join(tLineTrace, NULL);
    pthread_join(tDisplay, NULL);
    pthread_join(tLaser, NULL);
   
    free(scaled_data);
    video_interface_close(handle);
    //draw_bitmap_close_window(handle_GUI_color);
    //draw_bitmap_close_window(handle_GUI_gray);
    //draw_bitmap_close_window(handle_GUI_black_white);
    //draw_bitmap_close_window(handle_GUI_shrinked);
    draw_bitmap_stop();

    /* main task finished  */
    /* clean the GPIO pins */
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_INPUT;
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_INPUT;

  }
  else
  {
    ; /* warning message already issued */
  }

  printf( "main function done\n" );
  return 0;
}



