/**************************************************
* Homework 8, Robocar with manual control, linetracing, and laser following. (Mode1), (Mode2), (Mode3)
* Hardware: Rasperry PI.
* 
* By Bryan Shabroski
* CMPEN 473, Spring 2023, Penn State University
*
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
#define APB_CLOCK 250000000

#define ROUND_DIVISION(x,y) (((x) + (y)/2)/(y))


union uint16_to_2uint8
{
  struct uint16_to_2uint8_field
  {
    uint8_t   L;  /* Little Endian byte order means that the least significant byte goes in the lowest address */
    uint8_t   H;
  }         field;
  uint16_t  unsigned_value;
  int16_t   signed_value;
};


struct calibration_data
{
  float scale;
  float offset_x;
  float offset_y;
  float offset_z;
};


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
int speed = 50;
pthread_mutex_t speedMutex = PTHREAD_MUTEX_INITIALIZER;
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


int laserPointX = -1;
int laserPointY = -1;
pthread_mutex_t laserPointMutex = PTHREAD_MUTEX_INITIALIZER;

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
  struct fifo_t                 * shapesFifo;
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


struct shapes_thread_param
{
  const char                    * name;
  struct fifo_t                 * shapesFifo;
  struct fifo_t                 * motor1Fifo;
  struct fifo_t                 * motor2Fifo;
  struct fifo_t                 * pwmFifo;
  bool                          * quit_flag;
};

struct imu_thread_param
{
  const char                    * name;
  struct io_peripherals         * io;
  struct calibration_data       * calibration_accelerometer;
  struct calibration_data       * calibration_gyroscope;
  struct calibration_data       * calibration_magnetometer;
  bool                          * quit_flag;
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
                      
                        if(angle != 0)
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
                        mode3 = false;
                        stop = true;
                        forward = false;
                        backward = false;
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
                  //Square
                  case 'w':
                  {
                      cmd2.command = 'S';
                      if(!(FIFO_FULL(param->shapesFifo)))
                      {
                        FIFO_INSERT(param->shapesFifo, cmd2);
                        }
                    }
                      break;
                  //Triangle
                  case 'u':
                  {
                      cmd2.command = 't';
                      if(!(FIFO_FULL(param->shapesFifo)))
                      {
                        FIFO_INSERT(param->shapesFifo, cmd2);
                        }
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
                        stop = false;
                        forward = false;
                        backward = false;
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
                  //Square
                  case 'w':
                  {
                      cmd2.command = 'S';
                      if(!(FIFO_FULL(param->shapesFifo)))
                      {
                        FIFO_INSERT(param->shapesFifo, cmd2);
                        }
                    }
                      break;
                  //Triangle
                  case 'u':
                  {
                      cmd2.command = 't';
                      if(!(FIFO_FULL(param->shapesFifo)))
                      {
                        FIFO_INSERT(param->shapesFifo, cmd2);
                        }
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
/*  struct line_trace_thread_param *param = (struct line_trace_thread_param *)arg;
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
    if(mode3){
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
                      pthread_mutex_lock(&laserPointMutex);
                      laserPointX = x * 5 + 5/2;
                      laserPointY = y * 5 + 5/2;
                      pthread_mutex_unlock(&laserPointMutex);
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
      }
      wait_period(&timer_state, 10u);
       
      }
    
  printf("LaserThread done\n");
  return NULL;*/

}

void *LineTraceThread(void *arg)
{
/*    struct line_trace_thread_param *param = (struct line_trace_thread_param *)arg;
    struct thread_command cmd2 = {0, 0};
    struct timespec timer_state;
    //Rows we will scan
    const int scanRow1 = 34; 
    const int scanRow2 = 38;  
    const int scanRow3 = 40;     

    //Check only this section for left and right side
    const int regionWidth = SHRUNK_W / 4;

    while (!*(param->quit_flag))
    {
        if (mode2 && !stop)
        {
            //Set our bools initially to false, declare the rows we will scan, then scan the rows left and right side.
            bool leftBlackDetected  = false;
            bool rightBlackDetected = false;
            
            pthread_mutex_lock(&g_shrunk_data_lock2);
            //Go through each of our rows.
            int scanRows[3] = {scanRow1, scanRow2, scanRow3};
            for (int i = 0; i < 3; i++)
            {
                int row = scanRows[i];
                //Check Left side
                if (!leftBlackDetected)
                {
                    for (int x = 0; x < regionWidth; x++)
                    {
                        if (g_shrunk_data2[row * SHRUNK_W + x].R == 0)
                        {
                            leftBlackDetected = true;
                            break;
                        }
                    }
                }
                //Check right side
                if (!rightBlackDetected)
                {
                    for (int x = SHRUNK_W - regionWidth; x < SHRUNK_W; x++)
                    {
                        if (g_shrunk_data2[row * SHRUNK_W + x].R == 0)
                        {
                            rightBlackDetected = true;
                            break;
                        }
                    }
                }
            }

            pthread_mutex_unlock(&g_shrunk_data_lock2);
            

            //Turn right
            if (!leftBlackDetected && rightBlackDetected)
            {
                printf("Turning right!\n");
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );

            }
            //Turn left
            else if (!rightBlackDetected && leftBlackDetected)
            {
                printf("Turning left!\n");
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 's';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
                              cmd2.command = 'x';
                              FIFO_INSERT( param->motor1Fifo, cmd2 );
                              cmd2.command = 'w';
                              FIFO_INSERT( param->motor2Fifo, cmd2 );
            }
            //Go forward
            else
            {
                cmd2.command = 'w';
                FIFO_INSERT(param->motor1Fifo, cmd2);
                FIFO_INSERT(param->motor2Fifo, cmd2);
            }
        }
        //If not stopped in mode2 we stop.
        else if (mode2)
        {
            if (stop)
            {
                cmd2.command = 's';
                FIFO_INSERT(param->motor1Fifo, cmd2);
                FIFO_INSERT(param->motor2Fifo, cmd2);
            }
        }

        wait_period(&timer_state, 10u);
    }

    printf("LineTrace function done\n");
    return NULL;*/
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
        printf("[Motor2Thread] Received command '%c'\n", cmd.command);
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
            //printf("Motor thread %s processing 'w'\n", param->name);
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
        printf("[Motor2Thread] Received command '%c'\n", cmd.command);
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
              //printf("Motor thread %s processing 'w'\n", param->name);
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
            wait_period( &timer_state, 75u );
            speed = temp;
          }
          break;
          case 'r':  //Triangle turn.
          {
            int temp = speed;
            speed = 100;
            param->io->pwm->DAT1 = speed;
            param->io->pwm->DAT2 = speed;
            wait_period( &timer_state, 500u );
            speed = temp;
          }
          break;
          case 's':  //Square turn
          {
            int temp = speed;
            speed = 100;
            param->io->pwm->DAT1 = speed;
            param->io->pwm->DAT2 = speed;
            wait_period( &timer_state, 365u );
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
/*void overlayCrosshair(struct pixel_format_RGB *buffer, unsigned int width, unsigned int height) 
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


//Draws our 3x3 grid to see where we can aim laser on blacked out window.
void overlay3x3Grid(struct pixel_format_RGB *buffer, unsigned int width, unsigned int height)
{
    int topEnd    = 20;
    int midEnd    = 28;
    int leftEnd   = 24;
    int centerEnd = 40;  
    
    //Because how arrays work 0th indexed we need to - 1
    if (topEnd    >= (int)height) topEnd    = height - 1;
    if (midEnd    >= (int)height) midEnd    = height - 1;
    if (leftEnd   >= (int)width ) leftEnd   = width  - 1;
    if (centerEnd >= (int)width ) centerEnd = width  - 1;

    //These draw the lines
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



//Draws a box around inputted pixel.
//Purpose is drawing a box around the laser pixel we input.

void drawBox(struct pixel_format_RGB *buffer, int imageWidth, int imageHeight,
              int x, int y, int boxSize)
{
    //Computations for where the lines need to go
    int halfSize = boxSize / 2;
    int left   = (x - halfSize < 0) ? 0 : x - halfSize;
    int right  = (x + halfSize >= imageWidth) ? imageWidth - 1 : x + halfSize;
    int top    = (y - halfSize < 0) ? 0 : y - halfSize;
    int bottom = (y + halfSize >= imageHeight) ? imageHeight - 1 : y + halfSize;
    
    //Draw the lines
    for (int i = left; i <= right; i++) {
        buffer[top * imageWidth + i].R = 0;
        buffer[top * imageWidth + i].G = 255;
        buffer[top * imageWidth + i].B = 0;
        
        buffer[bottom * imageWidth + i].R = 0;
        buffer[bottom * imageWidth + i].G = 255;
        buffer[bottom * imageWidth + i].B = 0;
    }
    
    for (int j = top; j <= bottom; j++) {
        buffer[j * imageWidth + left].R = 0;
        buffer[j * imageWidth + left].G = 255;
        buffer[j * imageWidth + left].B = 0;
        
        buffer[j * imageWidth + right].R = 0;
        buffer[j * imageWidth + right].G = 255;
        buffer[j * imageWidth + right].B = 0;
    }
}*/



void *DisplayThread(void *arg)
{
 /*   struct camera_thread_param *param = (struct camera_thread_param *)arg;
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
      
        //If we want to actually close the windows do it here. If we are just pausing them (saves computation time) we can just use global variables.
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
            
            int lx, ly;
            pthread_mutex_lock(&laserPointMutex);
            lx = laserPointX;
            ly = laserPointY;
            pthread_mutex_unlock(&laserPointMutex);
            
            if(lx != -1 && ly != -1) 
            {
              drawBox(colorBuffer, param->scaled_width, param->scaled_height, lx, ly, 20);
            }
                      
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
                    uint8_t bw   = (gray > 25) ? 255 : 0;
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
    return NULL;*/
}


void *ShapesThread(void *arg)
{
  struct  shapes_thread_param * param = (struct shapes_thread_param *)arg;
  struct  thread_command cmd2 = {0, 0};
  struct  thread_command cmd1 = {0, 0};
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
 
  while (!*(param->quit_flag))
  {
    if(mode3){
      if (!(FIFO_EMPTY( param->shapesFifo )))
      {
        FIFO_REMOVE( param->shapesFifo, &cmd2 );
        switch (cmd2.command)
        {
          case 'S':  //Make car go in square motion (forward, left 90 deg, forward, left 90 deg...)
          {
            printf("Square mode activated\n");
            for(int squareCount = 1; squareCount < 5; squareCount++)
            {
            printf("Line: %d\n", squareCount);
            cmd1.command = 'w';
            FIFO_INSERT( param->motor1Fifo, cmd1 );
            FIFO_INSERT( param->motor2Fifo, cmd1 );
            wait_period( &timer_state, 1000u );
            cmd1.command = 's';
            FIFO_INSERT( param->motor1Fifo, cmd1 );
            FIFO_INSERT( param->motor2Fifo, cmd1 );
            cmd1.command = 's';
            FIFO_INSERT( param->pwmFifo, cmd1);
            cmd1.command = 'x';
            FIFO_INSERT( param->motor1Fifo, cmd1 );
            cmd1.command = 'w';
            FIFO_INSERT( param->motor2Fifo, cmd1 );
            wait_period( &timer_state, 365u);
            cmd1.command = 's';
            FIFO_INSERT( param->motor1Fifo, cmd1 );
            FIFO_INSERT( param->motor2Fifo, cmd1 );
            }
          }
          break;
         
          case 't':
          {
            printf("Triangle mode activated");
            for(int triangleCount = 1; triangleCount < 4; triangleCount++)
            {
            printf("Line: %d\n", triangleCount);
            cmd1.command = 'w';
            printf("[ShapesThread] Received command '%c'\n", cmd1.command);
            FIFO_INSERT( param->motor1Fifo, cmd1 );
            FIFO_INSERT( param->motor2Fifo, cmd1 );
            wait_period( &timer_state, 1000u );
            cmd1.command = 's';
            FIFO_INSERT( param->motor1Fifo, cmd1 );
            FIFO_INSERT( param->motor2Fifo, cmd1 );
            cmd1.command = 'r';
            FIFO_INSERT( param->pwmFifo, cmd1);
            cmd1.command = 'x';
            FIFO_INSERT( param->motor1Fifo, cmd1 );
            cmd1.command = 'w';
            FIFO_INSERT( param->motor2Fifo, cmd1 );
            wait_period( &timer_state, 500);
            cmd1.command = 's';
            FIFO_INSERT( param->motor1Fifo, cmd1 );
            FIFO_INSERT( param->motor2Fifo, cmd1 );
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
          }
          break;

          default:
          {
 
            printf( " %s wrong cmd\n", param->name);

          }
        }
      }
    }
      wait_period( &timer_state, 10u );
  }

  printf( "%s function done\n", param->name );

  return NULL;
  
}

// READ REG
void read_MPU9250_registers(                          /* read a register */
    uint8_t                         I2C_address,      /* the address of the I2C device to talk to */
    MPU9250_REGISTER                register_address, /* the address to read from */
    uint8_t *                       read_data,        /* the data read from the SPI device */
    size_t                          data_length,      /* the length of data to send/receive */
    volatile struct bsc_register *  bsc )             /* the BSC address */
{
  bsc->S.field.DONE    = 1;
  bsc->A.field.ADDR    = I2C_address;
  bsc->C.field.READ    = 0;
  bsc->DLEN.field.DLEN = 1;
  bsc->FIFO.value      = register_address;
  bsc->C.field.ST      = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep( 100 );
  }
  bsc->S.field.DONE    = 1;
  bsc->A.field.ADDR    = I2C_address;
  bsc->C.field.READ    = 1;
  bsc->DLEN.field.DLEN = data_length;
  bsc->C.field.ST      = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep( 100 );
  }

  while (data_length > 0)
  {
    *read_data = bsc->FIFO.field.DATA;

    read_data++;
    data_length--;
  }

  return;
}

// READ REG
union MPU9250_transaction_field_data read_MPU9250_register( /* read a register, returning the read value */
    uint8_t                         I2C_address,            /* the address of the I2C device to talk to */
    MPU9250_REGISTER                register_address,       /* the address to read from */
    volatile struct bsc_register *  bsc )                   /* the BSC address */
{
  union MPU9250_transaction transaction;

  read_MPU9250_registers( I2C_address, register_address, &(transaction.value[1]), 1, bsc );

  return transaction.field.data;
}

// WRITE REG
void write_MPU9250_register(                                /* write a register */
    uint8_t                               I2C_address,      /* the address of the I2C device to talk to */
    MPU9250_REGISTER                      register_address, /* the address to read from */
    union MPU9250_transaction_field_data  value,            /* the value to write */
    volatile struct bsc_register *        bsc )             /* the BSC address */
{
  union MPU9250_transaction transaction;

  transaction.field.data = value;
  bsc->S.field.DONE    = 1;
  bsc->A.field.ADDR    = I2C_address;
  bsc->C.field.READ    = 0;
  bsc->DLEN.field.DLEN = 2;
  bsc->FIFO.value      = register_address;
  bsc->FIFO.value      = transaction.value[1];
  bsc->C.field.ST      = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep( 100 );
  }

  return;
}

// CAL AG
void calibrate_accelerometer_and_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc )
{
  union MPU9250_transaction_field_data  transaction;
  uint8_t                               data_block_fifo_count[2];
  union uint16_to_2uint8                reconstructor;
  uint16_t                              ii;
  uint16_t                              packet_count;
  int32_t                               gyro_bias_x;
  int32_t                               gyro_bias_y;
  int32_t                               gyro_bias_z;
  int32_t                               accel_bias_x;
  int32_t                               accel_bias_y;
  int32_t                               accel_bias_z;
  uint8_t                               data_block_fifo_packet[12];
  union uint16_to_2uint8                reconstructor_accel_x;
  union uint16_to_2uint8                reconstructor_accel_y;
  union uint16_to_2uint8                reconstructor_accel_z;
  union uint16_to_2uint8                reconstructor_gyro_x;
  union uint16_to_2uint8                reconstructor_gyro_y;
  union uint16_to_2uint8                reconstructor_gyro_z;

  // reset device
  transaction.PWR_MGMT_1.CLKSEL       = 0;
  transaction.PWR_MGMT_1.PD_PTAT      = 0;
  transaction.PWR_MGMT_1.GYRO_STANDBY = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.H_RESET      = 1;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_PWR_MGMT_1, transaction, bsc );
  usleep( 100000 );

  // get stable time source; auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator
  transaction.PWR_MGMT_1.CLKSEL       = 1;
  transaction.PWR_MGMT_1.PD_PTAT      = 0;
  transaction.PWR_MGMT_1.GYRO_STANDBY = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.H_RESET      = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_PWR_MGMT_1, transaction, bsc );
  transaction.PWR_MGMT_2.DIS_ZG   = 0;
  transaction.PWR_MGMT_2.DIS_YG   = 0;
  transaction.PWR_MGMT_2.DIS_XG   = 0;
  transaction.PWR_MGMT_2.DIS_ZA   = 0;
  transaction.PWR_MGMT_2.DIS_YA   = 0;
  transaction.PWR_MGMT_2.DIS_XA   = 0;
  transaction.PWR_MGMT_2.reserved = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_PWR_MGMT_2, transaction, bsc );
  usleep( 200000 );
  
   // configure device for bias calculation
  transaction.INT_ENABLE.RAW_RDY_EN    = 0; // disable all interrupts
  transaction.INT_ENABLE.reserved0     = 0;
  transaction.INT_ENABLE.FSYNC_INT_EN  = 0;
  transaction.INT_ENABLE.FIFO_OFLOW_EN = 0;
  transaction.INT_ENABLE.reserved1     = 0;
  transaction.INT_ENABLE.WOM_EN        = 0;
  transaction.INT_ENABLE.reserved2     = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_INT_ENABLE, transaction, bsc );
  transaction.FIFO_EN.SLV0         = 0; // disable FIFO
  transaction.FIFO_EN.SLV1         = 0;
  transaction.FIFO_EN.SLV2         = 0;
  transaction.FIFO_EN.ACCEL        = 0;
  transaction.FIFO_EN.GYRO_ZO_UT   = 0;
  transaction.FIFO_EN.GYRO_YO_UT   = 0;
  transaction.FIFO_EN.GYRO_XO_UT   = 0;
  transaction.FIFO_EN.TEMP_FIFO_EN = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_FIFO_EN, transaction, bsc );
  transaction.PWR_MGMT_1.CLKSEL       = 0;  // turn on internal clock source
  transaction.PWR_MGMT_1.PD_PTAT      = 0;
  transaction.PWR_MGMT_1.GYRO_STANDBY = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.H_RESET      = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_PWR_MGMT_1, transaction, bsc );
  transaction.I2C_MST_CTRL.I2C_MST_CLK   = 0; // disable I2C master
  transaction.I2C_MST_CTRL.I2C_MST_P_NSR = 0;
  transaction.I2C_MST_CTRL.SLV_3_FIFO_EN = 0;
  transaction.I2C_MST_CTRL.WAIT_FOR_ES   = 0;
  transaction.I2C_MST_CTRL.MULT_MST_EN   = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_I2C_MST_CTRL, transaction, bsc );
  transaction.USER_CTRL.SIG_COND_RST = 0; // disable FIFO and I2C master modes
  transaction.USER_CTRL.I2C_MST_RST  = 0;
  transaction.USER_CTRL.FIFO_RST     = 0;
  transaction.USER_CTRL.reserved0    = 0;
  transaction.USER_CTRL.I2C_IF_DIS   = 0;
  transaction.USER_CTRL.I2C_MST_EN   = 0;
  transaction.USER_CTRL.FIFO_EN      = 0;
  transaction.USER_CTRL.reserved1    = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_USER_CTRL, transaction, bsc );
  transaction.USER_CTRL.SIG_COND_RST = 0; // reset FIFO and DMP
  transaction.USER_CTRL.I2C_MST_RST  = 0;
  transaction.USER_CTRL.FIFO_RST     = 1;
  transaction.USER_CTRL.reserved0    = 0;
  transaction.USER_CTRL.I2C_IF_DIS   = 0;
  transaction.USER_CTRL.I2C_MST_EN   = 0;
  transaction.USER_CTRL.FIFO_EN      = 0;
  transaction.USER_CTRL.reserved1    = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_USER_CTRL, transaction, bsc );
  usleep( 15000 );
  
    // configure MPU9250 gyro and accelerometer for bias calculation
  transaction.CONFIG.DLPF_CFG     = 1;  // set low-pass filter to 188Hz
  transaction.CONFIG.EXT_SYNC_SET = 0;
  transaction.CONFIG.FIFO_MODE    = 0;
  transaction.CONFIG.reserved     = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_CONFIG, transaction, bsc );
  transaction.SMPLRT_DIV.SMPLRT_DIV = 0;  // set sample rate to 1kHz
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_SMPLRT_DIV, transaction, bsc );
  transaction.GYRO_CONFIG.FCHOICE_B   = 0; // set gyro full-scale to 250dps, maximum sensitivity
  transaction.GYRO_CONFIG.reserved    = 0;
  transaction.GYRO_CONFIG.GYRO_FS_SEL = 0;
  transaction.GYRO_CONFIG.ZGYRO_Cten  = 0;
  transaction.GYRO_CONFIG.YGYRO_Cten  = 0;
  transaction.GYRO_CONFIG.XGYRO_Cten  = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_GYRO_CONFIG, transaction, bsc );
  transaction.ACCEL_CONFIG.reserved     = 0; // set accelerometer full-scale to 2g, maximum sensitivity
  transaction.ACCEL_CONFIG.ACCEL_FS_SEL = 0;
  transaction.ACCEL_CONFIG.az_st_en     = 0;
  transaction.ACCEL_CONFIG.ay_st_en     = 0;
  transaction.ACCEL_CONFIG.ax_st_en     = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_ACCEL_CONFIG, transaction, bsc );

  calibration_accelerometer->scale = 2.0/32768.0;  // measurement scale/signed numeric range
  calibration_accelerometer->offset_x = 0;
  calibration_accelerometer->offset_y = 0;
  calibration_accelerometer->offset_z = 0;

  calibration_gyroscope->scale = 250.0/32768.0;
  calibration_gyroscope->offset_x = 0;
  calibration_gyroscope->offset_y = 0;
  calibration_gyroscope->offset_z = 0;

  // configure FIFO to capture accelerometer and gyro data for bias calculation
  transaction.USER_CTRL.SIG_COND_RST = 0; // enable FIFO
  transaction.USER_CTRL.I2C_MST_RST  = 0;
  transaction.USER_CTRL.FIFO_RST     = 0;
  transaction.USER_CTRL.reserved0    = 0;
  transaction.USER_CTRL.I2C_IF_DIS   = 0;
  transaction.USER_CTRL.I2C_MST_EN   = 0;
  transaction.USER_CTRL.FIFO_EN      = 1;
  transaction.USER_CTRL.reserved1    = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_USER_CTRL, transaction, bsc );
  transaction.FIFO_EN.SLV0         = 0; // enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU9250)
  transaction.FIFO_EN.SLV1         = 0;
  transaction.FIFO_EN.SLV2         = 0;
  transaction.FIFO_EN.ACCEL        = 1;
  transaction.FIFO_EN.GYRO_ZO_UT   = 1;
  transaction.FIFO_EN.GYRO_YO_UT   = 1;
  transaction.FIFO_EN.GYRO_XO_UT   = 1;
  transaction.FIFO_EN.TEMP_FIFO_EN = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_FIFO_EN, transaction, bsc );
  usleep( 40000 );  // accumulate 40 samples in 40 milliseconds = 480 bytes

  // at end of sample accumulation, turn off FIFO sensor read
  transaction.FIFO_EN.SLV0         = 0; // disable gyro and accelerometer sensors for FIFO
  transaction.FIFO_EN.SLV1         = 0;
  transaction.FIFO_EN.SLV2         = 0;
  transaction.FIFO_EN.ACCEL        = 0;
  transaction.FIFO_EN.GYRO_ZO_UT   = 0;
  transaction.FIFO_EN.GYRO_YO_UT   = 0;
  transaction.FIFO_EN.GYRO_XO_UT   = 0;
  transaction.FIFO_EN.TEMP_FIFO_EN = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_FIFO_EN, transaction, bsc );
  read_MPU9250_registers( MPU9250_ADDRESS, MPU9250_REGISTER_FIFO_COUNTH, data_block_fifo_count, sizeof(data_block_fifo_count), bsc ); // read FIFO sample count
  reconstructor.field.H = data_block_fifo_count[0];
  reconstructor.field.L = data_block_fifo_count[1];
  packet_count = reconstructor.unsigned_value / 12; // how many sets of full gyro and accelerometer data for averaging

  accel_bias_x = 0;
  accel_bias_y = 0;
  accel_bias_z = 0;
  gyro_bias_x = 0;
  gyro_bias_y = 0;
  gyro_bias_z = 0;
  for (ii = 0; ii < packet_count; ii++)
  {
    read_MPU9250_registers( MPU9250_ADDRESS, MPU9250_REGISTER_FIFO_R_W, data_block_fifo_packet, sizeof(data_block_fifo_packet), bsc ); // read data for averaging

    reconstructor_accel_x.field.H = data_block_fifo_packet[0];
    reconstructor_accel_x.field.L = data_block_fifo_packet[1];
    reconstructor_accel_y.field.H = data_block_fifo_packet[2];
    reconstructor_accel_y.field.L = data_block_fifo_packet[3];
    reconstructor_accel_z.field.H = data_block_fifo_packet[4];
    reconstructor_accel_z.field.L = data_block_fifo_packet[5];
    reconstructor_gyro_x.field.H  = data_block_fifo_packet[6];
    reconstructor_gyro_x.field.L  = data_block_fifo_packet[7];
    reconstructor_gyro_y.field.H  = data_block_fifo_packet[8];
    reconstructor_gyro_y.field.L  = data_block_fifo_packet[9];
    reconstructor_gyro_z.field.H  = data_block_fifo_packet[10];
    reconstructor_gyro_z.field.L  = data_block_fifo_packet[11];

    accel_bias_x += reconstructor_accel_x.signed_value; // sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias_y += reconstructor_accel_y.signed_value;
    accel_bias_z += reconstructor_accel_z.signed_value;
    gyro_bias_x  += reconstructor_gyro_x.signed_value;
    gyro_bias_y  += reconstructor_gyro_y.signed_value;
    gyro_bias_z  += reconstructor_gyro_z.signed_value;
  }
  accel_bias_x /= (int32_t)packet_count;
  accel_bias_y /= (int32_t)packet_count;
  accel_bias_z /= (int32_t)packet_count;
  gyro_bias_x /= (int32_t)packet_count;
  gyro_bias_y /= (int32_t)packet_count;
  gyro_bias_z /= (int32_t)packet_count;
  if (accel_bias_z > 0) // remove gravity from the z-axis accelerometer bias calculation
  {
    accel_bias_z -= (int32_t)(1.0/calibration_accelerometer->scale);
  }
  else
  {
    accel_bias_z += (int32_t)(1.0/calibration_accelerometer->scale);
  }

  // the code that this is based off of tried to push the bias calculation values to hardware correction registers
  // these registers do not appear to be functioning, so rely on software offset correction

  // output scaled gyro biases
  calibration_gyroscope->offset_x = ((float)gyro_bias_x)*calibration_gyroscope->scale;
  calibration_gyroscope->offset_y = ((float)gyro_bias_y)*calibration_gyroscope->scale;
  calibration_gyroscope->offset_z = ((float)gyro_bias_z)*calibration_gyroscope->scale;

  // output scaled accelerometer biases
  calibration_accelerometer->offset_x = ((float)accel_bias_x)*calibration_accelerometer->scale;
  calibration_accelerometer->offset_y = ((float)accel_bias_y)*calibration_accelerometer->scale;
  calibration_accelerometer->offset_z = ((float)accel_bias_z)*calibration_accelerometer->scale;

  return;
}

// INIT AG
void initialize_accelerometer_and_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc )
{
  union MPU9250_transaction_field_data  transaction;

  /* print WHO_AM_I */
  printf( "accel WHOAMI (0x71) = 0x%2.2X\n",
      read_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_WHO_AM_I, bsc ).WHO_AM_I.WHOAMI );

  // based off https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp

  calibrate_accelerometer_and_gyroscope( calibration_accelerometer, calibration_gyroscope, bsc );

  // reset MPU9205
  transaction.PWR_MGMT_1.CLKSEL        = 0;
  transaction.PWR_MGMT_1.PD_PTAT       = 0;
  transaction.PWR_MGMT_1.GYRO_STANDBY  = 0;
  transaction.PWR_MGMT_1.CYCLE         = 0;
  transaction.PWR_MGMT_1.SLEEP         = 0;
  transaction.PWR_MGMT_1.H_RESET       = 1;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_PWR_MGMT_1, transaction, bsc );
  usleep( 1000 ); // wait for all registers to reset

  // clock source
  transaction.PWR_MGMT_1.CLKSEL       = 1;
  transaction.PWR_MGMT_1.PD_PTAT      = 0;
  transaction.PWR_MGMT_1.GYRO_STANDBY = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.H_RESET      = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_PWR_MGMT_1, transaction, bsc );

  // enable acc & gyro
  transaction.PWR_MGMT_2.DIS_ZG   = 0;
  transaction.PWR_MGMT_2.DIS_YG   = 0;
  transaction.PWR_MGMT_2.DIS_XG   = 0;
  transaction.PWR_MGMT_2.DIS_ZA   = 0;
  transaction.PWR_MGMT_2.DIS_YA   = 0;
  transaction.PWR_MGMT_2.DIS_XA   = 0;
  transaction.PWR_MGMT_2.reserved = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_PWR_MGMT_1, transaction, bsc );

  // use DLPF set gyro bandwidth 184Hz, temperature bandwidth 188Hz
  transaction.CONFIG.DLPF_CFG     = 1;
  transaction.CONFIG.EXT_SYNC_SET = 0;
  transaction.CONFIG.FIFO_MODE    = 0;
  transaction.CONFIG.reserved     = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_CONFIG, transaction, bsc );

  // +-250dps
  transaction.GYRO_CONFIG.FCHOICE_B   = 0;
  transaction.GYRO_CONFIG.reserved    = 0;
  transaction.GYRO_CONFIG.GYRO_FS_SEL = 0;
  transaction.GYRO_CONFIG.ZGYRO_Cten  = 0;
  transaction.GYRO_CONFIG.YGYRO_Cten  = 0;
  transaction.GYRO_CONFIG.XGYRO_Cten  = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_GYRO_CONFIG, transaction, bsc );

  // +-2G
  transaction.ACCEL_CONFIG.reserved     = 0;
  transaction.ACCEL_CONFIG.ACCEL_FS_SEL = 0;
  transaction.ACCEL_CONFIG.az_st_en     = 0;
  transaction.ACCEL_CONFIG.ay_st_en     = 0;
  transaction.ACCEL_CONFIG.ax_st_en     = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_ACCEL_CONFIG, transaction, bsc );

  // set acc data rates,enable acc LPF, bandwidth 184Hz
  transaction.ACCEL_CONFIG_2.A_DLPF_CFG      = 0;
  transaction.ACCEL_CONFIG_2.ACCEL_FCHOICE_B = 0;
  transaction.ACCEL_CONFIG_2.reserved        = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_ACCEL_CONFIG_2, transaction, bsc );

  // force into I2C mode, disabling I2C master
  transaction.USER_CTRL.SIG_COND_RST = 0;
  transaction.USER_CTRL.I2C_MST_RST  = 0;
  transaction.USER_CTRL.FIFO_RST     = 0;
  transaction.USER_CTRL.reserved0    = 0;
  transaction.USER_CTRL.I2C_IF_DIS   = 0;
  transaction.USER_CTRL.I2C_MST_EN   = 0;
  transaction.USER_CTRL.FIFO_EN      = 0;
  transaction.USER_CTRL.reserved1    = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_USER_CTRL, transaction, bsc );

  // enable bypass mode
  transaction.INT_PIN_CFG.reserved          = 0;
  transaction.INT_PIN_CFG.BYPASS_EN         = 1;
  transaction.INT_PIN_CFG.FSYNC_INT_MODE_EN = 0;
  transaction.INT_PIN_CFG.ACTL_FSYNC        = 0;
  transaction.INT_PIN_CFG.INT_ANYRD_2CLEAR  = 0;
  transaction.INT_PIN_CFG.LATCH_INT_EN      = 0;
  transaction.INT_PIN_CFG.OPEN              = 0;
  transaction.INT_PIN_CFG.ACTL              = 0;
  write_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_INT_PIN_CFG, transaction, bsc );

  return;
}

// INIT M
void initialize_magnetometer(
    struct calibration_data *     calibration_magnetometer,
    volatile struct bsc_register *bsc )
{
  union MPU9250_transaction_field_data  transaction;
  uint8_t                               data_block[3];

  // read WHOAMI from the magnetometer
  transaction = read_MPU9250_register( AK8963_ADDRESS, AK8963_REGISTER_WIA, bsc );
  printf( "mag   WHOAMI (0x48) = 0x%2.2X\n", transaction.WIA.WIA );

  // reset AK8963
  transaction.CNTL2.SRST      = 1;
  transaction.CNTL2.reserved  = 0;
  write_MPU9250_register( AK8963_ADDRESS, AK8963_REGISTER_CNTL2, transaction, bsc );
  usleep( 1000 );

  // I2C slave 0 register address from where to being data transfer
  // register value to 100Hz continuous measurement in 14bit
  transaction.CNTL1.MODE      = 6;
  transaction.CNTL1.BIT       = 0;
  transaction.CNTL1.reserved  = 0;
  write_MPU9250_register( AK8963_ADDRESS, AK8963_REGISTER_CNTL1, transaction, bsc );
  usleep( 1000 );

  // get the magnetometer calibration... extracted from the "calib_mag" function at https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp
  read_MPU9250_registers( AK8963_ADDRESS, AK8963_REGISTER_ASAX, data_block, sizeof(data_block), bsc );
  calibration_magnetometer->scale = (float)1;
  calibration_magnetometer->offset_x = ((((float)data_block[0])-128.0)/256.0+1.0);
  calibration_magnetometer->offset_y = ((((float)data_block[1])-128.0)/256.0+1.0);
  calibration_magnetometer->offset_z = ((((float)data_block[2])-128.0)/256.0+1.0);

  return;
}

// READ AG
void read_accelerometer_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc )
{
  uint8_t                   data_block[6+2+6];
  union uint16_to_2uint8    ACCEL_XOUT;
  union uint16_to_2uint8    ACCEL_YOUT;
  union uint16_to_2uint8    ACCEL_ZOUT;
  union uint16_to_2uint8    GYRO_XOUT;
  union uint16_to_2uint8    GYRO_YOUT;
  union uint16_to_2uint8    GYRO_ZOUT;

  /*
   * poll the interrupt status register and it tells you when it is done
   * once it is done, read the data registers
   */
  do
  {
    usleep( 1000 );
  } while (read_MPU9250_register( MPU9250_ADDRESS, MPU9250_REGISTER_INT_STATUS, bsc ).INT_STATUS.RAW_DATA_RDY_INT == 0);

  // read the accelerometer values
  read_MPU9250_registers( MPU9250_ADDRESS, MPU9250_REGISTER_ACCEL_XOUT_H, data_block, sizeof(data_block), bsc );
  ACCEL_XOUT.field.H  = data_block[0];
  ACCEL_XOUT.field.L  = data_block[1];
  ACCEL_YOUT.field.H  = data_block[2];
  ACCEL_YOUT.field.L  = data_block[3];
  ACCEL_ZOUT.field.H  = data_block[4];
  ACCEL_ZOUT.field.L  = data_block[5];
  // TEMP_OUT.field.H = data_block[6];
  // TEMP_OUT.field.L = data_block[7];
  GYRO_XOUT.field.H   = data_block[8];
  GYRO_XOUT.field.L   = data_block[9];
  GYRO_YOUT.field.H   = data_block[10];
  GYRO_YOUT.field.L   = data_block[11];
  GYRO_ZOUT.field.H   = data_block[12];
  GYRO_ZOUT.field.L   = data_block[13];

  printf( "Gyro X: %.2f deg\ty=%.2f deg\tz=%.2f deg\n",
      GYRO_XOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_x,
      GYRO_YOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_y,
      GYRO_ZOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_z );

  printf( "Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
      (ACCEL_XOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_x)*9.81,
      (ACCEL_YOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_y)*9.81,
      (ACCEL_ZOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_z)*9.81 );

  return;
}

// READ M
void read_magnetometer(
    struct calibration_data *     calibration_magnetometer,
    volatile struct bsc_register *bsc )
{
  uint8_t                               data_block[7];
  union uint16_to_2uint8                MAG_XOUT;
  union uint16_to_2uint8                MAG_YOUT;
  union uint16_to_2uint8                MAG_ZOUT;
  union MPU9250_transaction_field_data  transaction;

  read_MPU9250_registers( AK8963_ADDRESS, AK8963_REGISTER_HXL, data_block, 7, bsc );
  // read must start from HXL and read seven bytes so that ST2 is read and the AK8963 will start the next conversion
  MAG_XOUT.field.L = data_block[0];
  MAG_XOUT.field.H = data_block[1];
  MAG_YOUT.field.L = data_block[2];
  MAG_YOUT.field.H = data_block[3];
  MAG_ZOUT.field.L = data_block[4];
  MAG_ZOUT.field.H = data_block[5];
  printf( "Mag X: %.2f uT\ty=%.2f uT\tz=%.2f uT\n",
      MAG_XOUT.signed_value*calibration_magnetometer->offset_x,
      MAG_YOUT.signed_value*calibration_magnetometer->offset_y,
      MAG_ZOUT.signed_value*calibration_magnetometer->offset_z );

  return;
}

void *ImuThread(void  *arg)
{
  struct  imu_thread_param * param = (struct imu_thread_param *)arg;
  struct  thread_command cmd2 = {0, 0};
  struct  thread_command cmd1 = {0, 0};
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  param->io->gpio->GPFSEL0.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
  param->io->gpio->GPFSEL0.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;
  initialize_accelerometer_and_gyroscope( param->calibration_accelerometer, param->calibration_gyroscope, param->io->bsc );
  initialize_magnetometer( param->calibration_magnetometer, param->io->bsc );
  param->io->bsc->DIV.field.CDIV  = (PERIPHERAL_CLOCK*10)/400000;
  param->io->bsc->DEL.field.REDL  = 0x30;
  param->io->bsc->DEL.field.FEDL  = 0x30;
  param->io->bsc->CLKT.field.TOUT = 0x40;
  param->io->bsc->C.field.INTD    = 0;
  param->io->bsc->C.field.INTT    = 0;
  param->io->bsc->C.field.INTR    = 0;
  param->io->bsc->C.field.I2CEN   = 1;
  param->io->bsc->C.field.CLEAR   = 1;
  while (!*(param->quit_flag))
  {
    read_accelerometer_gyroscope( param->calibration_accelerometer, param->calibration_gyroscope, param->io->bsc );
    read_magnetometer( param->calibration_magnetometer, param->io->bsc );
    printf( "\n" );
    wait_period( &timer_state, 10u );
  }

  printf( "%s function done\n", param->name );

  return NULL;
}


int main( int argc, char * argv[] )
{
  struct io_peripherals *io;
  struct video_interface_handle_t * handle;
  struct calibration_data calibration_accelerometer;
  struct calibration_data calibration_gyroscope;
  struct calibration_data calibration_magnetometer;
  
  //Create 5 threads 2 for the motors one for the speed, one for keys and one for control. All except key need their respective fifos, and thread paramaters
  pthread_t tMotor1;
  pthread_t tMotor2;
  pthread_t tPWM;
  pthread_t tk;
  pthread_t tc;
  pthread_t tLineTrace;
  pthread_t tLaser;
  pthread_t tDisplay;
  pthread_t tShapes;
  pthread_t tImu;
  struct fifo_t key_fifo   = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t motor1Fifo    = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t motor2Fifo    = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t pwmFifo = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t shapesFifo = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
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
  struct control_thread_param   con_param   = {"con", &key_fifo, &motor1Fifo, &motor2Fifo, &pwmFifo, &shapesFifo, &quit_flag};
  struct line_trace_thread_param lineTraceParam = {"linetrace", &motor1Fifo, &motor2Fifo, &pwmFifo, &quit_flag, NULL, scaled_RGB_data, scaled_height, scaled_width};
  struct camera_thread_param cameraParam = {handle, scaled_data, scaled_RGB_data, scaled_height, scaled_width, &quit_flag, argc, argv, handle_GUI_color};
  struct laser_thread_param laserParam = {"laser", &motor1Fifo, &motor2Fifo, &pwmFifo, &quit_flag, NULL, scaled_RGB_data, scaled_height, scaled_width};
  struct shapes_thread_param shapeParam = {"shapes",&shapesFifo, &motor1Fifo, &motor2Fifo, &pwmFifo, &quit_flag};
  struct imu_thread_param imuParam = {"imu", NULL , &calibration_accelerometer ,&calibration_gyroscope ,&calibration_magnetometer};
  
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
   
    printf("\n----------------------\n");
    printf("\n\n\n Welcome!\n\n w: forward\n s: stop\n x:backward\n a: left \n d: right \n o: increase degrees \n k: decrease degrees \n \n i: power up 5%\n j: power down 5%\nc: color image\nv: gray image\nb: black white image\nn: shrink image (black white)\n\n\n");
    printf("\n----------------------\n");
    // Create three threads our threase and then join them once the q command is hit
    pthread_create(&tMotor1, NULL, Motor1Thread, (void *)&motor1Param);
    pthread_create(&tMotor2, NULL, Motor2Thread, (void *)&motor2Param);
    pthread_create(&tPWM, NULL, SpeedThread, (void *)&pwmParam);
    pthread_create(&tk, NULL, KeyRead,   (void *)&key_param);
    pthread_create(&tc, NULL, Control,   (void *)&con_param);
    pthread_create(&tLineTrace, NULL, LineTraceThread, (void *)&lineTraceParam);
    pthread_create(&tDisplay, NULL, DisplayThread, (void *)&cameraParam);
    pthread_create(&tLaser, NULL, LaserThread, (void *)&laserParam);
    pthread_create(&tShapes, NULL, ShapesThread, (void *)&shapeParam);
    pthread_create(&tImu, NULL, ImuThread, (void *)&imuParam);
    // Join threads
    pthread_join(tMotor1, NULL);
    pthread_join(tMotor2, NULL);
    pthread_join(tPWM, NULL);
    pthread_join(tk, NULL);
    pthread_join(tc, NULL);
    pthread_join(tLineTrace, NULL);
    pthread_join(tDisplay, NULL);
    pthread_join(tLaser, NULL);
    pthread_join(tShapes, NULL);
    pthread_join(tImu, NULL);
   
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



