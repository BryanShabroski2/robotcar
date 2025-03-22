#include <stdlib.h>
#include <gdk/gdk.h>
#include <stdbool.h>
#include <cairo/cairo.h>
#include <gtk/gtk.h>
#include <pthread.h>
#include <string.h>
#include "pixel_format_RGB.h"
#include "draw_bitmap_multiwindow.h"

#define ARRAYSIZE(A) (sizeof(A)/sizeof(A[0]))

#define REPORT_TIME_DIFFERENCE  0
#define REPORT_PIXEL_VALUES     0

#if REPORT_TIME_DIFFERENCE
static long time_difference_ms( struct timespec *start_time, struct timespec *end_time )
{
  return (end_time->tv_sec - start_time->tv_sec)*1000 + (end_time->tv_nsec - start_time->tv_nsec)/(1000*1000);
}
#endif

static int                                        gtk_argc;
static char **                                    gtk_argv;
static pthread_t                                  thread_id;
static struct draw_bitmap_multiwindow_handle_t *  handle_list_head = NULL;
static struct draw_bitmap_multiwindow_handle_t *  handle_list_tail = NULL;

/* insert a new handle into the list of displayed windows */
static void insert_handle( struct draw_bitmap_multiwindow_handle_t * handle )
{
  /* insert the new handle at the beginning of the list */

  handle->next                  = handle_list_head;
  handle->previous              = NULL;
  if (handle_list_head != NULL)
  {
    handle_list_head->previous  = handle;
  }
  else
  {
    handle_list_tail            = handle;
  }
  handle_list_head              = handle;

  return;
}

/* remove a handle from the list of displayed windows */
static void remove_handle( struct draw_bitmap_multiwindow_handle_t * handle )
{
  /* remove the handle from the list */

  if (handle_list_head != handle)
  {
    handle->previous->next      = handle->next;
  }
  else
  {
    handle_list_head            = handle->next;
  }
  if (handle_list_tail != handle)
  {
    handle->next->previous      = handle->previous;
  }
  else
  {
    handle_list_tail            = handle->previous;
  }
  handle->next                  = NULL;
  handle->previous              = NULL;

  return;
}

/* Create a new backing pixmap of the appropriate size */
static gboolean configure_event( GtkWidget *widget, GdkEventConfigure *event, gpointer data )
{
  struct draw_bitmap_multiwindow_handle_t * handle = (struct draw_bitmap_multiwindow_handle_t *)data;

  if (handle->pixmap)
  {
    g_object_unref( handle->pixmap );
  }
  else
  {
    ; /* no pixmap to delete */
  }

  handle->pixmap = gdk_pixmap_new(
      widget->window,
      widget->allocation.width, widget->allocation.height,
      -1 );

  return TRUE;
}

/* Redraw the screen from the backing pixmap */
static gboolean expose_event( GtkWidget *widget, GdkEventExpose *event, gpointer data )
{
  struct draw_bitmap_multiwindow_handle_t * handle = (struct draw_bitmap_multiwindow_handle_t *)data;

  gdk_draw_drawable(
      widget->window,
      widget->style->fg_gc[gtk_widget_get_state( widget )],
      handle->pixmap,
      event->area.x,      event->area.y,
      event->area.x,      event->area.y,
      event->area.width,  event->area.height );

  return FALSE;
}

/* draw a window, creating the GUI drawing context and maintaining the inter-thread data interface to ensure proper image display */
static void draw_a_window( struct draw_bitmap_multiwindow_handle_t * handle )
{
  cairo_t *                                 cairo_context;
  unsigned int                              most_recently_updated;
  bool                                      bitmap_updated;
  unsigned int                              index;
  cairo_surface_t *                         image_surface;
#if REPORT_TIME_DIFFERENCE
  struct timespec                           start_time;
  struct timespec                           stop_time;

  clock_gettime( CLOCK_REALTIME, &start_time );
#endif

  /* determine what is going on and what data is being used when updating the window */
  pthread_mutex_lock( &(handle->data_interface.lock) );
  handle->data_interface.GUI_reading                = true;
  most_recently_updated                             = handle->data_interface.most_recently_updated;
  bitmap_updated                                    = handle->data_interface.updated_since_last_display;
  handle->data_interface.updated_since_last_display = false;
  pthread_mutex_unlock( &(handle->data_interface.lock) );

  if (bitmap_updated)
  {
    /* update the window */
    cairo_context = gdk_cairo_create( handle->drawing_area->window );

#if REPORT_PIXEL_VALUES
    printf( "R[%d]: [%3.0d, %3.0d, %3.0d] [%3.0d, %3.0d, %3.0d]\n",
        most_recently_updated,
        data_interface.bitmap[most_recently_updated][0].R,
        data_interface.bitmap[most_recently_updated][0].G,
        data_interface.bitmap[most_recently_updated][0].B,
        data_interface.bitmap[most_recently_updated][1].R,
        data_interface.bitmap[most_recently_updated][1].G,
        data_interface.bitmap[most_recently_updated][1].B );
#endif
    for (index = 0; index < handle->data_interface.width * handle->data_interface.height; index++)
    {
      handle->cairo_bitmap[index].R = handle->data_interface.bitmap[most_recently_updated][index].R;
      handle->cairo_bitmap[index].G = handle->data_interface.bitmap[most_recently_updated][index].G;
      handle->cairo_bitmap[index].B = handle->data_interface.bitmap[most_recently_updated][index].B;
      handle->cairo_bitmap[index].unused = 0;
    }
    image_surface = cairo_image_surface_create_for_data(
        (unsigned char *)handle->cairo_bitmap,
        handle->cairo_format,
        handle->data_interface.width,
        handle->data_interface.height,
        handle->cairo_stride );
    cairo_set_source_surface( cairo_context, image_surface, 0, 0 );
    cairo_paint( cairo_context );
    cairo_show_page( cairo_context );
    cairo_surface_destroy( image_surface );
#if REPORT_PIXEL_VALUES
    printf( "R[%d]: done\n", most_recently_updated );
#endif

    cairo_destroy( cairo_context );
  }
  else
  {
    ; /* there is no sense displaying data that has not been updated */
  }

  /* release the bitmap for future updates */
  pthread_mutex_lock( &(handle->data_interface.lock) );
  handle->data_interface.GUI_reading = false;
  pthread_mutex_unlock( &(handle->data_interface.lock) );

#if REPORT_TIME_DIFFERENCE
  clock_gettime( CLOCK_REALTIME, &stop_time );
  printf( "%6.6ld\n", time_difference_ms( &start_time, &stop_time ) );
#endif

  return;
}

/* timeout event handler */
static gint timeout_event( gpointer data )
{
  struct draw_bitmap_multiwindow_handle_t * handle;

  for (handle = handle_list_head; handle != NULL; handle = handle->next)
  {
    draw_a_window( handle );
  }

  return TRUE;
}

/* the function that shuts down the event loop when the window is destroyed */
static void destroy( GtkWidget *widget, gpointer data )
{
  struct draw_bitmap_multiwindow_handle_t * handle = (struct draw_bitmap_multiwindow_handle_t *)data;

  /* unlink the window so that it will not receive bitmap updates */
  remove_handle( handle );

  /* indicate that the window is no longer being refreshed */
  handle->data_interface.window_closed = true;

  return;
}

/* the GUI thread, creates the window and waits for events */
static void *GUI_thread( void *thread_parameter )
{
  gint  timeout_handle;

  gtk_init( &gtk_argc, &gtk_argv );

  /* Signals and events */
  timeout_handle = g_timeout_add( 30, timeout_event, NULL );

  /* start the global GUI event loop */
  gtk_main();

  /* clean up */
  g_source_remove( timeout_handle );

  return NULL;
}

/* use an idle handler to register a new window for display, setting up GUI events and linking it into the list of windows to poll for bitmap updates */
static gboolean idle_add_window( gpointer data )
{
  struct draw_bitmap_multiwindow_handle_t * handle = (struct draw_bitmap_multiwindow_handle_t *)data;
  GtkWidget *                               vbox;

  /* create the window and lay it out */
  handle->window = gtk_window_new( GTK_WINDOW_TOPLEVEL );
  gtk_widget_set_name( handle->window, "Draw Bitmap" );

  vbox = gtk_vbox_new( FALSE, 0 );
  gtk_container_add( GTK_CONTAINER( handle->window ), vbox );
  gtk_widget_show( vbox );

  /* Create the drawing area */
  handle->drawing_area = gtk_drawing_area_new();
  gtk_widget_set_size_request( GTK_WIDGET( handle->drawing_area ), handle->data_interface.width, handle->data_interface.height );
  gtk_box_pack_start( GTK_BOX( vbox ), handle->drawing_area, TRUE, TRUE, 0 );
  gtk_widget_show( handle->drawing_area );

  /* Signals and events */
  g_signal_connect( handle->window,       "destroy",              G_CALLBACK( destroy ),              handle );
  g_signal_connect( handle->drawing_area, "expose_event",         G_CALLBACK( expose_event ),         handle );
  g_signal_connect( handle->drawing_area, "configure_event",      G_CALLBACK( configure_event ),      handle );

  /* display the window and handle events */
  gtk_widget_show( handle->window );

  /* link the newly created window in with the others to monitor for bitmap updates */
  insert_handle( handle );

  return FALSE;
}

/* use an idle handler to destroy a window */
static gboolean idle_remove_window( gpointer data )
{
  struct draw_bitmap_multiwindow_handle_t * handle = (struct draw_bitmap_multiwindow_handle_t *)data;

  /* unregister callbacks for the window being destroyed, the destroy event will handle the unlinking */
  g_signal_handlers_disconnect_by_func( handle->drawing_area, G_CALLBACK( expose_event ),     handle );
  g_signal_handlers_disconnect_by_func( handle->drawing_area, G_CALLBACK( configure_event ),  handle );
  gtk_widget_destroy( handle->window );

  /* notify the handle owner that it is now safe to clean up */
  pthread_mutex_lock( &(handle->window_operation_lock) );
  handle->window_operation_complete     = true;
  pthread_cond_signal( &(handle->window_operation_cond) );
  pthread_mutex_unlock( &(handle->window_operation_lock) );

  return FALSE;
}

/* use an idle handler to remove all windows and shut down the GUI thread */
static gboolean idle_GUI_shutdown( gpointer data )
{
  /* dereference and close all open windows */
  while (handle_list_head != NULL)
  {
    idle_remove_window( handle_list_head );
  }

  /* shut down the GUI event loop */
  gtk_main_quit();

  return FALSE;
}

/* see header for function descriptions */
int draw_bitmap_start(
    int           argc,
    char **       argv )
{
  int return_value;

  /* put argc and argv somewhere where the GUI thread can access and start the thread */
  gtk_argc = argc;
  gtk_argv = argv;

  return_value = pthread_create( &thread_id, NULL, GUI_thread, NULL );

  return return_value;
}

struct draw_bitmap_multiwindow_handle_t * draw_bitmap_create_window(
    unsigned int  width,
    unsigned int  height )
{
  struct draw_bitmap_multiwindow_handle_t * handle;
  unsigned int                              index;
  static const pthread_mutex_t              pthread_mutex_static_initializer = PTHREAD_MUTEX_INITIALIZER;
  static const pthread_cond_t               pthread_cond_static_initializer = PTHREAD_COND_INITIALIZER;

  handle = malloc( sizeof(*handle) );
  if (handle != NULL)
  {
    /* initialize the handle */
    handle->data_interface.lock                       = pthread_mutex_static_initializer;
    handle->data_interface.most_recently_updated      = 0;
    handle->data_interface.GUI_reading                = false;
    handle->data_interface.updated_since_last_display = false;
    for (index = 0; index < ARRAYSIZE(handle->data_interface.bitmap); index++)
    {
      handle->data_interface.bitmap[index]            = malloc( width*height*sizeof(*handle->data_interface.bitmap[index]) );
      memset( handle->data_interface.bitmap[index], 0, width*height*sizeof(*handle->data_interface.bitmap[index]) );
    }
    handle->data_interface.window_closed              = false;
    handle->data_interface.width                      = width;
    handle->data_interface.height                     = height;
    handle->pixmap                                    = NULL;
    handle->cairo_format                              = CAIRO_FORMAT_RGB24;
    handle->cairo_stride                              = cairo_format_stride_for_width( handle->cairo_format, handle->data_interface.width );
    handle->cairo_bitmap                              = (struct draw_bitmap_multiwindow_cairo_RGB24_t *)malloc( handle->cairo_stride * handle->data_interface.height );
    handle->drawing_area                              = NULL;
    handle->window                                    = NULL;
    handle->next                                      = NULL;
    handle->previous                                  = NULL;
    handle->window_operation_lock                     = pthread_mutex_static_initializer;
    handle->window_operation_cond                     = pthread_cond_static_initializer;
    handle->window_operation_complete                 = true;

    /* hand the handle off to the GUI thread to create the widget, register callbacks for window events, and link into the list of handles to monitor for bitmap updates */
    g_idle_add( idle_add_window, handle );
  }
  else
  {
    // could not allocate memory
  }

  return handle;
}

void draw_bitmap_display(
    struct draw_bitmap_multiwindow_handle_t * handle,
    struct pixel_format_RGB *                 bitmap )
{
  unsigned int  index;
  unsigned int  bitmap_to_update;

  /* the writer always updates the oldest data, the reader only ever reads the newest data */
  pthread_mutex_lock( &(handle->data_interface.lock) );
  bitmap_to_update = (handle->data_interface.most_recently_updated + 1) % ARRAYSIZE(handle->data_interface.bitmap);
  pthread_mutex_unlock( &(handle->data_interface.lock) );

  /* update the stale bitmap */
  for (index = 0; index < handle->data_interface.width*handle->data_interface.height; index++)
  {
    handle->data_interface.bitmap[bitmap_to_update][index] = bitmap[index];
  }
#if REPORT_PIXEL_VALUES
  printf( "W[%d]: -%3.0d, %3.0d, %3.0d- -%3.0d, %3.0d, %3.0d-\n",
      bitmap_to_update,
      data_interface.bitmap[bitmap_to_update][0].R,
      data_interface.bitmap[bitmap_to_update][0].G,
      data_interface.bitmap[bitmap_to_update][0].B,
      data_interface.bitmap[bitmap_to_update][1].R,
      data_interface.bitmap[bitmap_to_update][1].G,
      data_interface.bitmap[bitmap_to_update][1].B );
#endif

  /* if there is no reader present, let the reader know that there is updated data */
  pthread_mutex_lock( &(handle->data_interface.lock) );
  if (handle->data_interface.GUI_reading)
  {
    ; /* there is a reader using the newest data, do not try to touch it */
  }
  else
  {
    handle->data_interface.most_recently_updated      = bitmap_to_update;
    handle->data_interface.updated_since_last_display = true;
  }
  pthread_mutex_unlock( &(handle->data_interface.lock) );

  return;
}

void draw_bitmap_close_window(
    struct draw_bitmap_multiwindow_handle_t * handle )
{
  unsigned int  index;

  if (!handle->data_interface.window_closed)
  {
    /* indicate that a window operation is needed and kick off a remove_window operation */
    handle->window_operation_complete = false;
    g_idle_add( idle_remove_window, handle );

    /* wait for the window operation to complete */
    pthread_mutex_lock( &(handle->window_operation_lock) );
    while (!handle->window_operation_complete)
    {
      pthread_cond_wait( &(handle->window_operation_cond), &(handle->window_operation_lock) );
    }
    pthread_mutex_unlock( &(handle->window_operation_lock) );
  }
  else
  {
    ; /* nothing to wait for, the window was closed through the GUI and has already been dereferenced */
  }

  /* clean up the window now that it is no longer referenced by the GUI system */
  for (index = 0; index < ARRAYSIZE(handle->data_interface.bitmap); index++)
  {
    free( handle->data_interface.bitmap[index] );
  }
  free( handle->cairo_bitmap );
  free( handle );

  return;
}

bool draw_bitmap_window_closed(
    struct draw_bitmap_multiwindow_handle_t * handle )
{
  return handle->data_interface.window_closed;
}

void draw_bitmap_stop( void )
{
  /* initiate GUI shutdown */
  g_idle_add( idle_GUI_shutdown, NULL );

  /* wait for the GUI thread to shut down */
  pthread_join( thread_id, NULL );

  return;
}
