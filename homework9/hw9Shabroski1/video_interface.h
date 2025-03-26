/*
 * video_interface.h
 *
 *  Created on: Jun 4, 2022
 *      Author: steveb
 */

#ifndef VIDEO_INTERFACE_H_
#define VIDEO_INTERFACE_H_

/*
 * libswscale must be installed to use this library
 *
 * apt-cache search libswscale-dev
 * sudo apt-get install libswscale-dev
 */

///////////////////////////////////////////////////////////////////////////////
// image format
#if 1
#define IMAGE_WIDTH   640
#define IMAGE_HEIGHT  480
#else
#define IMAGE_WIDTH   1920
#define IMAGE_HEIGHT  1080
#endif

struct image_row_t
{
    struct pixel_format_RGB column[IMAGE_WIDTH];     // the pixels for a particular row of the image
};

struct image_t
{
    struct image_row_t row[IMAGE_HEIGHT]; // the rows of pixels for the image
};

///////////////////////////////////////////////////////////////////////////////
// handle definition
struct video_interface_handle_buffer_t
{
  void *  start;    // the start of the buffer range used in video capture
  size_t  length;   // the length of the buffer used in video capture
};

struct video_interface_handle_format_t
{
  struct v4l2_fmtdesc       description;        // the format description for this mode
  size_t                    frame_size_length;  // the number of frame sizes available for this pixel format
  struct v4l2_frmsizeenum * frame_size;         // the array of frame sizes
};

struct video_interface_handle_t
{
    int                                     fd;                 // the file descriptor for accessing the v4l2 device
    struct video_interface_handle_buffer_t  buffer[4];          // the buffers used to retrieve image data from the v4l2 device
    size_t                                  format_length;      // the number of supported image formats for this interface
    struct video_interface_handle_format_t *format;             // the array of supported image formats
    size_t                                  selected_mode;      // the pixel format and frame size selected for scaling
    struct v4l2_format                      capture_format;     // the capture format being used
    size_t                                  configured_width;   // the actual image width being used
    size_t                                  configured_height;  // the actual image height being used
    struct SwsContext *                     scaling;            // the configuration for libswscale
};

///////////////////////////////////////////////////////////////////////////////
// functions
struct video_interface_handle_t * video_interface_open(             const char *                      v4l2_device );
void                              video_interface_close(            struct video_interface_handle_t * handle );
void                              video_interface_print_modes(      struct video_interface_handle_t * handle );
bool                              video_interface_set_mode_auto(    struct video_interface_handle_t * handle );
bool                              video_interface_set_mode_manual(  struct video_interface_handle_t * handle, unsigned int      mode );
bool                              video_interface_get_image(        struct video_interface_handle_t * handle, struct image_t *  image );

#endif /* VIDEO_INTERFACE_H_ */
