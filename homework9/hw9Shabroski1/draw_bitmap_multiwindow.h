/*
 * What is the following interface?
 * Normally, I prefer inter-thread communication to come in one of two forms:
 * [1] a "flag" where no history is maintained, every write stomps on whatever was there
 * [2] a "queue" where all history is maintained, there is a backlog of previous writes to be read
 *
 * The following structure is somewhat similar to the queue option in that there is some history.
 * In this case, a reader active in the communication channel switches the mode of operation of the interface to that of a flag; the writer stomps on the data not being read.
 * When no reader is in the channel, a writer will ping-pong between the two bitmaps.
 * The reader only ever cares about the most recent bitmap.
 * The writer tries to update the oldest bitmap. Sometimes, the writer does not have that opportunity because the reader is using the oldest bitmap.
 *
 * Note:
 * If the GUI thread cannot display fast enough to keep up with the timer, the display will not update.
 * Neither the GUI thread nor the main thread will block, but the display will not make progress.
 */
struct draw_bitmap_multiwindow_data_interface_t
{
    /* dynamic data */
    pthread_mutex_t           lock;                       /* a lock for the "most_recently_updated" and "GUI_reading" data */
    unsigned int              most_recently_updated;      /* the index of the most recent bitmap */
    bool                      GUI_reading;                /* set to true when a reader is active in the channel */
    bool                      updated_since_last_display; /* true when this bitmap has been written to since the last time it was displayed on screen */
    struct pixel_format_RGB * bitmap[2];                  /* the data within the bitmaps are read/written with no lock held, the index above is read while locked to figure out which data to read/write */
    bool                      window_closed;              /* set to true when the window was prematurely closed */

    /* static configuration */
    unsigned int              width;
    unsigned int              height;
};
struct draw_bitmap_multiwindow_cairo_RGB24_t
{
    unsigned char B;
    unsigned char G;
    unsigned char R;
    unsigned char unused;
};
struct draw_bitmap_multiwindow_handle_t
{
    /* the data interface for communicating to the GUI thread */
    struct draw_bitmap_multiwindow_data_interface_t data_interface;

    /* Backing pixmap for drawing area */
    GdkPixmap *                                     pixmap;
    struct draw_bitmap_multiwindow_cairo_RGB24_t *  cairo_bitmap;
    cairo_format_t                                  cairo_format;
    int                                             cairo_stride;
    GtkWidget *                                     drawing_area;
    GtkWidget *                                     window;

    struct draw_bitmap_multiwindow_handle_t *       next;                       /* used by the timer event to find open windows to display */
    struct draw_bitmap_multiwindow_handle_t *       previous;                   /* used by the timer event to find open windows to display */

    pthread_mutex_t                                 window_operation_lock;      /* used for open/close window operations */
    pthread_cond_t                                  window_operation_cond;      /* used to indicate the completion of open/close window operations */
    bool                                            window_operation_complete;  /* true when the window open/close operation is complete */
};

int draw_bitmap_start(                                                /* start a background thread to handle the multiple windows, returns 0 on error */
    int                                       argc,                   /* the argc parameter to main */
    char **                                   argv );                 /* the argv parameter to main */

struct draw_bitmap_multiwindow_handle_t * draw_bitmap_create_window(  /* return the allocated handle or NULL if failed */
    unsigned int                              width,                  /* the width of the bitmap to display */
    unsigned int                              height );               /* the height of the bitmap to display */

void draw_bitmap_display(                                             /* update the displayed bitmap */
    struct draw_bitmap_multiwindow_handle_t * handle,                 /* the handle returned from draw_bitmap_create_window */
    struct pixel_format_RGB *                 bitmap );               /* the new bitmap contents */

bool draw_bitmap_window_closed(                                       /* returns true when the window was closed by the user (by clicking the X in the upper-right, for example) */
    struct draw_bitmap_multiwindow_handle_t * handle );               /* the handle returned from draw_bitmap_create_window */

void draw_bitmap_close_window(                                        /* closes the window and deallocates memory associated with the handle */
    struct draw_bitmap_multiwindow_handle_t * handle );               /* the handle returned from draw_bitmap_create_window */

void draw_bitmap_stop( void );                                        /* shut down the background thread and clean up */
