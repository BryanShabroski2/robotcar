/*
 * video_interface.c
 *
 *  Created on: Jun 4, 2022
 *      Author: steveb
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <libswscale/swscale.h>
#include <time.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>
#include "pixel_format_RGB.h"
#include "video_interface.h"

/*
 * apt-cache search libswscale-dev
 * see also http://git.videolan.org/?p=ffmpeg.git;a=blob;f=libswscale/swscale.h;h=3ebf7c698d31a023fde0dd5ee7fa2b9d0a0d7997;hb=refs/heads/master
 * also, /usr/include/arm-linux-gnueabihf/libswscale/swscale.h
 * fmt.fmt.pix.bytesperline is the line stride (see start_capturing above)
 */

#define ARRAYSIZE(X) (sizeof(X)/sizeof((X)[0]))

static enum AVPixelFormat get_format_conversion(  // returns AV_PIX_FMT_NONE when the format is not supported
    __u32 pixelformat )                           // the v4l2 pixel format to look up
{
  enum AVPixelFormat return_value;

#define MAKE_CASE(FMT_V4L2, FMT_AVU)  case V4L2_PIX_FMT_##FMT_V4L2: return_value = AV_PIX_FMT_##FMT_AVU;  break
  switch (pixelformat)
  {
    MAKE_CASE(YUV420M, YUV420P);                ///< planar YUV 4:2:0, 12bpp, (1 Cr & Cb sample per 2x2 Y samples)
    MAKE_CASE(YUYV, YUYV422);                   ///< packed YUV 4:2:2, 16bpp, Y0 Cb Y1 Cr
    MAKE_CASE(RGB24, RGB24);                    ///< packed RGB 8:8:8, 24bpp, RGBRGB...
    MAKE_CASE(BGR24, BGR24);                    ///< packed RGB 8:8:8, 24bpp, BGRBGR...
    MAKE_CASE(YUV422P, YUV422P);                ///< planar YUV 4:2:2, 16bpp, (1 Cr & Cb sample per 2x1 Y samples)
    MAKE_CASE(YUV444M, YUV444P);                ///< planar YUV 4:4:4, 24bpp, (1 Cr & Cb sample per 1x1 Y samples)
    MAKE_CASE(YUV410, YUV410P);                 ///< planar YUV 4:1:0,  9bpp, (1 Cr & Cb sample per 4x4 Y samples)
    MAKE_CASE(YUV411P, YUV411P);                ///< planar YUV 4:1:1, 12bpp, (1 Cr & Cb sample per 4x1 Y samples)
    MAKE_CASE(PAL8, PAL8);                      ///< 8 bits with AV_PIX_FMT_RGB32 palette
    MAKE_CASE(UYVY, UYVY422);                   ///< packed YUV 4:2:2, 16bpp, Cb Y0 Cr Y1
    MAKE_CASE(Y41P, UYYVYY411);                 ///< packed YUV 4:1:1, 12bpp, Cb Y0 Y1 Cr Y2 Y3
    MAKE_CASE(RGB332, RGB8);                    ///< packed RGB 3:3:2,  8bpp, (msb)2R 3G 3B(lsb)
    MAKE_CASE(NV12, NV12);                      ///< planar YUV 4:2:0, 12bpp, 1 plane for Y and 1 plane for the UV components, which are interleaved (first byte U and the following byte V)
    MAKE_CASE(NV21, NV21);                      ///< as above, but U and V bytes are swapped
    MAKE_CASE(ARGB32, ARGB);                    ///< packed ARGB 8:8:8:8, 32bpp, ARGBARGB...
    MAKE_CASE(RGBA32, RGBA);                    ///< packed RGBA 8:8:8:8, 32bpp, RGBARGBA...
    MAKE_CASE(ABGR32, ABGR);                    ///< packed ABGR 8:8:8:8, 32bpp, ABGRABGR...
    MAKE_CASE(BGRA32, BGRA);                    ///< packed BGRA 8:8:8:8, 32bpp, BGRABGRA...
    MAKE_CASE(RGB565, RGB565LE);                ///< packed RGB 5:6:5, 16bpp, (msb)   5R 6G 5B(lsb), little-endian
    MAKE_CASE(RGB565X, RGB565BE);               ///< packed RGB 5:6:5, 16bpp, (msb)   5R 6G 5B(lsb), big-endian
    MAKE_CASE(RGB555, RGB555LE);                ///< packed RGB 5:5:5, 16bpp, (msb)1X 5R 5G 5B(lsb), little-endian, X=unused/undefined
    MAKE_CASE(RGB555X, RGB555BE);               ///< packed RGB 5:5:5, 16bpp, (msb)1X 5R 5G 5B(lsb), big-endian   , X=unused/undefined
    MAKE_CASE(XBGR555, BGR555LE);               ///< packed BGR 5:5:5, 16bpp, (msb)1X 5B 5G 5R(lsb), little-endian, X=unused/undefined
    MAKE_CASE(RGB444, RGB444LE);                ///< packed RGB 4:4:4, 16bpp, (msb)4X 4R 4G 4B(lsb), little-endian, X=unused/undefined
    MAKE_CASE(XBGR444, BGR444LE);               ///< packed BGR 4:4:4, 16bpp, (msb)4X 4B 4G 4R(lsb), little-endian, X=unused/undefined
    MAKE_CASE(NV16, NV16);                      ///< interleaved chroma YUV 4:2:2, 16bpp, (1 Cr & Cb sample per 2x1 Y samples)
    MAKE_CASE(YVYU, YVYU422);                   ///< packed YUV 4:2:2, 16bpp, Y0 Cr Y1 Cb
    MAKE_CASE(XRGB32, 0RGB);                    ///< packed RGB 8:8:8, 32bpp, XRGBXRGB...   X=unused/undefined
    MAKE_CASE(RGBX32, RGB0);                    ///< packed RGB 8:8:8, 32bpp, RGBXRGBX...   X=unused/undefined
    MAKE_CASE(XBGR32, 0BGR);                    ///< packed BGR 8:8:8, 32bpp, XBGRXBGR...   X=unused/undefined
    MAKE_CASE(BGRX32, BGR0);                    ///< packed BGR 8:8:8, 32bpp, BGRXBGRX...   X=unused/undefined
    MAKE_CASE(SBGGR8, BAYER_BGGR8);             ///< bayer, BGBG..(odd line), GRGR..(even line), 8-bit samples */
    MAKE_CASE(SRGGB8, BAYER_RGGB8);             ///< bayer, RGRG..(odd line), GBGB..(even line), 8-bit samples */
    MAKE_CASE(SGBRG8, BAYER_GBRG8);             ///< bayer, GBGB..(odd line), RGRG..(even line), 8-bit samples */
    MAKE_CASE(SGRBG8, BAYER_GRBG8);             ///< bayer, GRGR..(odd line), BGBG..(even line), 8-bit samples */
    MAKE_CASE(SBGGR16, BAYER_BGGR16LE);         ///< bayer, BGBG..(odd line), GRGR..(even line), 16-bit samples, little-endian */
    MAKE_CASE(SRGGB16, BAYER_RGGB16LE);         ///< bayer, RGRG..(odd line), GBGB..(even line), 16-bit samples, little-endian */
    MAKE_CASE(SGBRG16, BAYER_GBRG16LE);         ///< bayer, GBGB..(odd line), RGRG..(even line), 16-bit samples, little-endian */
    MAKE_CASE(SGRBG16, BAYER_GRBG16LE);         ///< bayer, GRGR..(odd line), BGBG..(even line), 16-bit samples, little-endian */
    MAKE_CASE(NV24, NV24);                      ///< planar YUV 4:4:4, 24bpp, 1 plane for Y and 1 plane for the UV components, which are interleaved (first byte U and the following byte V)
    MAKE_CASE(NV42, NV42);                      ///< as above, but U and V bytes are swapped
    default:  return_value = AV_PIX_FMT_NONE; break;
  }
#undef MAKE_CASE

  return return_value;
}

static void enumerate_formats_and_frame_sizes(                      // populate the handle's format field with the device's supported image formats
    struct video_interface_handle_t * handle )                      // the interface to interrogate
{
  int                     ioctl_result;
  uint32_t                index_fmt;
  struct v4l2_fmtdesc     fmtdesc;
  uint32_t                index_frmsizeenum;
  struct v4l2_frmsizeenum frmsizeenum;

  // deallocate any existing format array
  for (index_fmt = 0; index_fmt < handle->format_length; index_fmt++)
  {
    free( handle->format[index_fmt].frame_size );
  }
  if (handle->format_length != 0)
  {
    free( handle->format );
  }
  else
  {
    ; // no formats enumerated
  }
  handle->format_length = 0;

  // determine the number of image formats so that the array can be allocated
  index_fmt = 0;
  fmtdesc.index = index_fmt;
  fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  for (ioctl_result = ioctl( handle->fd, VIDIOC_ENUM_FMT, &fmtdesc );
      ioctl_result >= 0;
      ioctl_result = ioctl( handle->fd, VIDIOC_ENUM_FMT, &fmtdesc ))
  {
    index_fmt++;
    fmtdesc.index = index_fmt;
  }

  // allocate the format descriptions
  handle->format_length = index_fmt;
  handle->format = (struct video_interface_handle_format_t *)calloc( handle->format_length, sizeof(*(handle->format)) );
  if (handle->format != NULL)
  {
    for (index_fmt = 0; index_fmt < handle->format_length; index_fmt++)
    {
      handle->format[index_fmt].description.index = index_fmt;
      handle->format[index_fmt].description.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ioctl_result = ioctl( handle->fd, VIDIOC_ENUM_FMT, &(handle->format[index_fmt].description) );
      if (ioctl_result >= 0)
      {
        // determine the number of frame sizes so that the array can be allocated
        index_frmsizeenum = 0;
        frmsizeenum.index = index_frmsizeenum;
        frmsizeenum.pixel_format = handle->format[index_fmt].description.pixelformat;
        for (ioctl_result = ioctl( handle->fd, VIDIOC_ENUM_FRAMESIZES, &frmsizeenum );
            ioctl_result >= 0;
            ioctl_result = ioctl( handle->fd, VIDIOC_ENUM_FRAMESIZES, &frmsizeenum ))
        {
          index_frmsizeenum++;
          frmsizeenum.index = index_frmsizeenum;
        }

        // allocate the frame size
        handle->format[index_fmt].frame_size_length = index_frmsizeenum;
        handle->format[index_fmt].frame_size = (struct v4l2_frmsizeenum *)calloc( handle->format[index_fmt].frame_size_length, sizeof(*(handle->format[index_fmt].frame_size)) );
        if (handle->format[index_fmt].frame_size != NULL)
        {
          for (index_frmsizeenum = 0; index_frmsizeenum < handle->format[index_fmt].frame_size_length; index_frmsizeenum++)
          {
            handle->format[index_fmt].frame_size[index_frmsizeenum].index = index_frmsizeenum;
            handle->format[index_fmt].frame_size[index_frmsizeenum].pixel_format = handle->format[index_fmt].description.pixelformat;
            ioctl_result = ioctl( handle->fd, VIDIOC_ENUM_FRAMESIZES, &(handle->format[index_fmt].frame_size[index_frmsizeenum]) );
            if (ioctl_result >= 0)
            {
              ; // frame size array entry successfully populated
            }
            else
            {
              printf( "unable to get previously retrieved frame size\n" );
            }
          }
        }
        else
        {
          printf( "unable to allocate frame size array during format enumeration\n" );
          handle->format[index_fmt].frame_size_length = 0;
        }
      }
      else
      {
        printf( "unable to get previously retrieved format description\n" );
      }
    }
  }
  else
  {
    printf( "unable to allocate format array during format enumeration\n" );
    handle->format_length = 0;
  }

  return;
}

static size_t count_modes(                      // return the total number of modes
    struct video_interface_handle_t * handle )  // device whose mode is to be looked up
{
    size_t  return_value;
    size_t  i;
    size_t  j;

    return_value = 0;
    for (i = 0; i < handle->format_length; i++)
    {
      for (j = 0; j < handle->format[i].frame_size_length; j++)
      {
        return_value++;
      }
    }

    return return_value;
}

static size_t format_index_frame_size_index_to_mode(      // return the mode index based on a format and frame size index
    struct video_interface_handle_t * handle,             // device whose mode is to be looked up
    size_t                            format_index,       // the format index to look up in the handle
    size_t                            frame_size_index )  // the frame size index within the format in the handle to look up
{
  size_t  return_value;
  size_t  i;
  size_t  j;

  return_value = 0;
  for (i = 0; (i < handle->format_length) && (i < format_index); i++)
  {
    for (j = 0; j < handle->format[i].frame_size_length; j++)
    {
      return_value++;
    }
  }
  if (i < handle->format_length)
  {
    for (j = 0; (j < handle->format[i].frame_size_length) && (j < frame_size_index); j++)
    {
      return_value++;
    }
  }
  else
  {
    ; // return_value is equal to the sum of all frame_size_lenght's (an invalid mode)
  }

  return return_value;
}

static bool mode_to_format_index_frame_size_index(        // return the format and frame size index based on a mode
    struct video_interface_handle_t * handle,             // device whose mode is to be looked up
    size_t                            mode,               // the mode to be looked up
    size_t *                          format_index,       // the format index returned
    size_t *                          frame_size_index )  // the frame size index returned
{
  size_t  mode_count;
  bool    mode_found;
  size_t  i;
  size_t  j;

  if (mode < count_modes( handle ))
  {
    mode_count = 0;
    mode_found = false;
    i = 0;
    while ((i < handle->format_length) &&
           !mode_found)
    {
      j = 0;
      while ((j < handle->format[i].frame_size_length) &&
             !mode_found)
      {
        if (mode == mode_count)
        {
          if (format_index != NULL)
          {
            *format_index = i;
          }
          else
          {
            ; // format index not requested
          }
          if (frame_size_index != NULL)
          {
            *frame_size_index = j;
          }
          else
          {
            ; // frame size index not requested
          }

          mode_found = true;
        }
        else
        {
          mode_count++;
          j++;
        }
      }

      if (mode_found)
      {
        ; // done
      }
      else
      {
        i++;
      }
    }
  }
  else
  {
    ; // invalid mode, do not set the values
  }

  return mode_found;
}

struct video_interface_handle_t * video_interface_open(             // return a handle to the specified device or NULL upon failure
    const char *                      v4l2_device )                 // the device to open (such as "/dev/video0"
{
    struct video_interface_handle_t * return_value; // the return value of this function
    int                               ioctl_result; // the success/failure return value of ioctl()
    struct v4l2_capability            capability;   // the v4l2 capabilities returned by the specified device

    return_value = (struct video_interface_handle_t *)calloc( 1, sizeof(*return_value) );
    if (return_value != NULL)
    {
      return_value->fd = open( v4l2_device, O_RDWR, 0 );
      if (return_value->fd >= 0)
      {
        ioctl_result = ioctl( return_value->fd, VIDIOC_QUERYCAP, &capability );
        if (ioctl_result >= 0)
        {
          if ((capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) != 0)
          {
            ; // device is available for use
          }
          else
          {
            printf( "device is unable to capture images: %s, %s, %s [0x%8.8X]\n",
                capability.driver,
                capability.card,
                capability.bus_info,
                capability.capabilities );

            free( return_value );
            return_value = NULL;
          }
        }
        else
        {
          printf( "unable to query device capabilities\n" );

          free( return_value );
          return_value = NULL;
        }
      }
      else
      {
        printf( "unable to open device %s\n", v4l2_device );

        free( return_value );
        return_value = NULL;
      }
    }
    else
    {
      printf( "memory allocation failed\n" );
    }

    return return_value;
}

void                              video_interface_close(            // close the video interface, invalidating the provided handle
    struct video_interface_handle_t * handle )                      // the interface to close
{
  enum v4l2_buf_type  type; // the buffer type to manipulate
  size_t              i;    // used to index through the allocated video buffers and formats

  if (handle != NULL)
  {
    // turn off video streaming so that the buffers are not in use
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl( handle->fd, VIDIOC_STREAMOFF, &type );

    // deallocate the streaming buffers
    for (i = 0; i < ARRAYSIZE( handle->buffer ); i++)
    {
      if (handle->buffer[i].start != NULL)
      {
        free( handle->buffer[i].start );
      }
      else
      {
        ; // nothing to deallocate
      }
    }

    // deallocate the array of formats
    for (i = 0; i < handle->format_length; i++)
    {
      free( handle->format[i].frame_size );
    }
    if (handle->format_length != 0)
    {
      free( handle->format );
    }
    else
    {
      ; // no formats enumerated
    }

    // shut down libswscale
    sws_freeContext( handle->scaling );

    // close the device and deallocate the handle
    close( handle->fd );
    free( handle );
  }
  else
  {
    ; // nothing to close
  }

  return;
}

void                              video_interface_print_modes(      // print the available modes for the video interface (formats and resolutions)
    struct video_interface_handle_t * handle )                      // the interface to interrogate
{
  size_t i;
  size_t j;
  size_t mode_index;

  if (handle != NULL)
  {
    // populate the format and frame size data in the handle for easy reference
    enumerate_formats_and_frame_sizes( handle );

    // print the enumerated data
    for (i = 0; i < handle->format_length; i++)
    {
      printf( "fmt[%2.1d] desc: %s\n",          handle->format[i].description.index, handle->format[i].description.description );
      printf( "        flags: 0x%8.8X\n",       handle->format[i].description.flags );
      printf( "        pixelformat: 0x%8.8X (%c-%c-%c-%c)\n", handle->format[i].description.pixelformat,
          (handle->format[i].description.pixelformat>> 0)&0xFF,
          (handle->format[i].description.pixelformat>> 8)&0xFF,
          (handle->format[i].description.pixelformat>>16)&0xFF,
          (handle->format[i].description.pixelformat>>24)&0xFF );
      printf( "        compatible with video interface: %s\n", (get_format_conversion( handle->format[i].description.pixelformat ) != AV_PIX_FMT_NONE) ? "TRUE" : "FALSE" );

      for (j = 0; j < handle->format[i].frame_size_length; j++)
      {
        mode_index = format_index_frame_size_index_to_mode( handle, i, j );
        printf( "        size[%2.1d] video interface mode: %zu\n", handle->format[i].frame_size[j].index, mode_index );
        printf( "                 pixel_format: 0x%8.8X\n", handle->format[i].frame_size[j].pixel_format );
        switch (handle->format[i].frame_size[j].type)
        {
          case V4L2_FRMSIZE_TYPE_DISCRETE:
            printf( "                 type: DISCRETE\n" );
            printf( "                 width:  %d\n", handle->format[i].frame_size[j].discrete.width );
            printf( "                 height: %d\n", handle->format[i].frame_size[j].discrete.height );
            break;
          case V4L2_FRMSIZE_TYPE_CONTINUOUS:
            printf( "                 type: CONTINUOUS\n" );
            break;
          case V4L2_FRMSIZE_TYPE_STEPWISE:
            printf( "                 type: STEPWISE\n" );
            printf( "                 min_width:   %d\n", handle->format[i].frame_size[j].stepwise.min_width   );
            printf( "                 max_width:   %d\n", handle->format[i].frame_size[j].stepwise.max_width   );
            printf( "                 step_width:  %d\n", handle->format[i].frame_size[j].stepwise.step_width  );
            printf( "                 min_height:  %d\n", handle->format[i].frame_size[j].stepwise.min_height  );
            printf( "                 max_height:  %d\n", handle->format[i].frame_size[j].stepwise.max_height  );
            printf( "                 step_height: %d\n", handle->format[i].frame_size[j].stepwise.step_height );
            break;
          default:
            printf( "                 type: UNKNOWN\n" );
            break;
        }
      }
    }
    printf( "\n" );
  }
  else
  {
    ; // handle not open
  }

  return;
}

bool                              video_interface_set_mode_auto(    // set up the video interface to grab images, taking a best guess at which to use (leaning towards native resolution discrete modes or 1920x1080 and an RGB or YUV format), returns true when successful
    struct video_interface_handle_t * handle )                      // the interface to configure
{
  size_t    i;                            // used to index through the formats
  size_t    j;                            // used to index through the frame sizes for each format
  uint32_t  max_width;                    // if no native resolution is found, this is the maximum resolution's width found
  uint32_t  max_height;                   // if no native resolution is found, this is the maximum resolution's height found
  size_t    max_resolution_i;             // if no native resolution is found, this is the maximum resolution's format index found
  size_t    max_resolution_j;             // if no native resolution is found, this is the maximum resolution's frame size index found
  size_t    mode;                         // used to set the mode, once a compatible/preferred mode is found
  bool      found_native_resolution;      // used to short-circuit the search for a compatible native resolution mode
  bool      found_compatible_resolution;  // used to indicate that the maximum resolution should be used
  bool      return_value;                 // the return value of this function

  if (handle != NULL)
  {
    // populate the format and frame size data in the handle for easy reference
    enumerate_formats_and_frame_sizes( handle );

    // see if we are in luck and there is a native resolution that requires NO SCALING or use the max resolution found, otherwise
    found_native_resolution     = false;
    found_compatible_resolution = false;
    max_width                   = 0;
    max_height                  = 0;
    for (i = 0; !found_native_resolution && (i < handle->format_length); i++)
    {
      if (get_format_conversion( handle->format[i].description.pixelformat ) != AV_PIX_FMT_NONE)
      {
        for (j = 0; !found_native_resolution && (j < handle->format[i].frame_size_length); j++)
        {
          switch (handle->format[i].frame_size[j].type)
          {
            case V4L2_FRMSIZE_TYPE_DISCRETE:
              if ((handle->format[i].frame_size[j].discrete.width  == IMAGE_WIDTH) &&
                  (handle->format[i].frame_size[j].discrete.height == IMAGE_HEIGHT))
              {
                mode = format_index_frame_size_index_to_mode( handle, i, j );
                found_native_resolution = true;
              }
              else
              {
                // some other size
                if ((handle->format[i].frame_size[j].discrete.width > max_width) &&
                    (handle->format[i].frame_size[j].discrete.height > max_height))
                {
                  max_resolution_i  = i;
                  max_resolution_j  = j;
                  max_width         = handle->format[i].frame_size[j].discrete.width;
                  max_height        = handle->format[i].frame_size[j].discrete.height;
                  found_compatible_resolution = true;
                }
                else
                {
                  ; // lower resolution than the maximum found so far
                }
              }
              break;
            case V4L2_FRMSIZE_TYPE_CONTINUOUS:
              mode = format_index_frame_size_index_to_mode( handle, i, j );
              found_native_resolution = true;
              break;
            case V4L2_FRMSIZE_TYPE_STEPWISE:
              if ((handle->format[i].frame_size[j].stepwise.min_width <= IMAGE_WIDTH) &&
                  (handle->format[i].frame_size[j].stepwise.max_width >= IMAGE_WIDTH) &&
                  ((IMAGE_WIDTH - handle->format[i].frame_size[j].stepwise.min_width) % handle->format[i].frame_size[j].stepwise.step_width == 0) &&
                  (handle->format[i].frame_size[j].stepwise.min_height <= IMAGE_HEIGHT) &&
                  (handle->format[i].frame_size[j].stepwise.max_height >= IMAGE_HEIGHT) &&
                  ((IMAGE_HEIGHT - handle->format[i].frame_size[j].stepwise.min_height) % handle->format[i].frame_size[j].stepwise.step_height == 0))
              {
                mode = format_index_frame_size_index_to_mode( handle, i, j );
                found_native_resolution = true;
              }
              else
              {
                // it was not a perfect fit... do what we can with it
                if ((handle->format[i].frame_size[j].stepwise.max_width > max_width) &&
                    (handle->format[i].frame_size[j].stepwise.max_height > max_height))
                {
                  max_resolution_i  = i;
                  max_resolution_j  = j;
                  max_width         = handle->format[i].frame_size[j].stepwise.max_width;
                  max_height        = handle->format[i].frame_size[j].stepwise.max_height;
                  found_compatible_resolution = true;
                }
                else
                {
                  ; // lower resolution than the maximum found so far
                }
              }
              break;
            default:
              ; // no idea what format is
              break;
          }
        }
      }
      else
      {
        ; // the format is not compatible with a simple format conversion (it might be JPEG or MPEG encoded)
      }
    }

    if (!found_native_resolution &&
        found_compatible_resolution)
    {
      mode = format_index_frame_size_index_to_mode( handle, max_resolution_i, max_resolution_j );
    }
    else
    {
      ; // not using the maximum resolution or possibly could not find a compatible format
    }

    if (found_native_resolution ||
        found_compatible_resolution)
    {
      // configure the device to use the discovered mode
      return_value = video_interface_set_mode_manual( handle, mode );
    }
    else
    {
      printf( "could not find a compatible mode during auto-selection\n" );
      return_value = false;
    }
  }
  else
  {
    return_value = false; // no handle to operate on
  }

  return return_value;
}

bool                              video_interface_set_mode_manual(  // set up the video interface to grab images, using the specified mode (see video_interface_print_modes) for the image format, returns true when successful
    struct video_interface_handle_t * handle,                       // the interface to configure
    unsigned int                      mode )                        // the mode, as displayed by video_interface_print_modes
{
  size_t                      format_index;
  size_t                      frame_size_index;
  bool                        mode_lookup_successful;
  unsigned int                min;
  struct v4l2_requestbuffers  req;
  size_t                      i;
  enum v4l2_buf_type          type;
  bool                        return_value;

  if (handle != NULL)
  {
    // populate the format and frame size data in the handle for easy reference
    enumerate_formats_and_frame_sizes( handle );

    // find the format that was chosen
    mode_lookup_successful = mode_to_format_index_frame_size_index( handle, mode, &format_index, &frame_size_index );
    if (mode_lookup_successful)
    {
      handle->selected_mode = mode;

      // configure v4l2 to use a reasonable format
      handle->capture_format.type                 = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      switch (handle->format[format_index].frame_size[frame_size_index].type)
      {
        case V4L2_FRMSIZE_TYPE_DISCRETE:
          handle->configured_width                = handle->format[format_index].frame_size[frame_size_index].discrete.width;
          handle->configured_height               = handle->format[format_index].frame_size[frame_size_index].discrete.height;
          break;
        case V4L2_FRMSIZE_TYPE_STEPWISE:
          // pick a frame size as close to 1920x1080 as possible (with rounding)
          if (handle->format[format_index].frame_size[frame_size_index].stepwise.max_width > 1920)
          {
            if (handle->format[format_index].frame_size[frame_size_index].stepwise.min_width < 1920)
            {
              handle->configured_width            = ((1920 + handle->format[format_index].frame_size[frame_size_index].stepwise.step_width/2) /
                                                      handle->format[format_index].frame_size[frame_size_index].stepwise.step_width) *
                                                     handle->format[format_index].frame_size[frame_size_index].stepwise.step_width;
            }
            else
            {
              handle->configured_width            = handle->format[format_index].frame_size[frame_size_index].stepwise.min_width;
            }
          }
          else
          {
            handle->configured_width              = handle->format[format_index].frame_size[frame_size_index].stepwise.max_width;
          }
          if (handle->format[format_index].frame_size[frame_size_index].stepwise.max_height > 1080)
          {
            if (handle->format[format_index].frame_size[frame_size_index].stepwise.min_height < 1080)
            {
              handle->configured_height           = ((1080 + handle->format[format_index].frame_size[frame_size_index].stepwise.step_height/2) /
                                                      handle->format[format_index].frame_size[frame_size_index].stepwise.step_height) *
                                                     handle->format[format_index].frame_size[frame_size_index].stepwise.step_height;
            }
            else
            {
              handle->configured_height           = handle->format[format_index].frame_size[frame_size_index].stepwise.min_height;
            }
          }
          else
          {
            handle->configured_height             = handle->format[format_index].frame_size[frame_size_index].stepwise.max_height;
          }
          break;
        case V4L2_FRMSIZE_TYPE_CONTINUOUS:
        default:
          handle->capture_format.fmt.pix.width    = 1920;
          handle->capture_format.fmt.pix.height   = 1080;
          break;
      }
      handle->capture_format.fmt.pix.width        = handle->configured_width;
      handle->capture_format.fmt.pix.height       = handle->configured_height;
      handle->capture_format.fmt.pix.pixelformat  = handle->format[format_index].description.pixelformat;
      handle->capture_format.fmt.pix.field        = V4L2_FIELD_ANY;
      handle->capture_format.fmt.pix.bytesperline = 0;
      handle->capture_format.fmt.pix.sizeimage    = 0;
      handle->capture_format.fmt.pix.colorspace   = V4L2_COLORSPACE_DEFAULT;
      handle->capture_format.fmt.pix.priv         = 0;
      handle->capture_format.fmt.pix.flags        = 0;
      handle->capture_format.fmt.pix.ycbcr_enc    = 0;
      handle->capture_format.fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
      handle->capture_format.fmt.pix.xfer_func    = V4L2_XFER_FUNC_DEFAULT;
      if (ioctl( handle->fd, VIDIOC_S_FMT, &(handle->capture_format) ) == 0)
      {
        // get the format configuration from the driver to determine the buffer sizes
        if (ioctl( handle->fd, VIDIOC_G_FMT, &(handle->capture_format) ) == 0)
        {
          /* Buggy driver paranoia. */
          min = handle->capture_format.fmt.pix.width * 2;
          if (handle->capture_format.fmt.pix.bytesperline < min)
          {
            handle->capture_format.fmt.pix.bytesperline = min;
          }
          else
          {
            ; // the line width is not absurd
          }
          min = handle->capture_format.fmt.pix.bytesperline * handle->capture_format.fmt.pix.height;
          if (handle->capture_format.fmt.pix.sizeimage < min)
          {
            handle->capture_format.fmt.pix.sizeimage = min;
          }
          else
          {
            ; // the size is not too small
          }

          // allocate image buffers
          memset( &req, 0, sizeof(req) );
          req.count  = 4;
          req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          req.memory = V4L2_MEMORY_USERPTR;
          if (ioctl( handle->fd, VIDIOC_REQBUFS, &req ) == 0)
          {
            return_value = true;
            for (i = 0; (i < ARRAYSIZE(handle->buffer)) && return_value; i++)
            {
              handle->buffer[i].length = handle->capture_format.fmt.pix.sizeimage;
              handle->buffer[i].start  = malloc(handle->capture_format.fmt.pix.sizeimage);

              if (handle->buffer[i].start != NULL)
              {
                ; // all is still well
              }
              else
              {
                fprintf( stderr, "Out of memory\n" );
                return_value = false;
              }
            }

            // set up buffers to receive the images
            for (i = 0; (i < ARRAYSIZE(handle->buffer)) && return_value; i++)
            {
              struct v4l2_buffer buf;

              memset( &buf, 0, sizeof(buf) );
              buf.type      = V4L2_BUF_TYPE_VIDEO_CAPTURE;
              buf.memory    = V4L2_MEMORY_USERPTR;
              buf.index     = i;
              buf.m.userptr = (unsigned long)(handle->buffer[i].start);
              buf.length    = handle->buffer[i].length;

              if (ioctl( handle->fd, VIDIOC_QBUF, &buf ) == 0)
              {
                ; // all is still well
              }
              else
              {
                printf( "VIDIOC_QBUF error %d, %s\n", errno, strerror( errno ) );
                return_value = false;
              }
            }

            // start capturing
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (return_value &&
                (ioctl( handle->fd, VIDIOC_STREAMON, &type ) == 0))
            {
              // configure libswscale to convert the selected image format to something more convenient for simple image analysis
              handle->scaling = sws_getContext(
                  handle->configured_width,                       // srcW
                  handle->configured_height,                      // srcH
                  get_format_conversion(                          // srcFormat
                      handle->format[format_index].description.pixelformat ),
                  IMAGE_WIDTH,                                    // dstW
                  IMAGE_HEIGHT,                                   // dstH
                  AV_PIX_FMT_RGB24,                               // dstFormat
                  SWS_FAST_BILINEAR,                              // flags
                  NULL,                                           // srcFilter
                  NULL,                                           // dstFilter
                  0 );                                            // param
            }
            else
            {
              printf( "VIDIOC_STREAMON error %d, %s\n", errno, strerror( errno ) );
              return_value = false;
            }
          }
          else
          {
            printf("VIDIOC_REQBUFS error %d, %s\n", errno, strerror(errno));
            return_value = false;
          }
        }
        else
        {
          printf( "VIDIOC_G_FMT error %d, %s\n", errno, strerror(errno));
          return_value = false;
        }
      }
      else
      {
        printf( "VIDIOC_S_FMT error %d, %s\n", errno, strerror(errno));
        return_value = false;
      }
    }
    else
    {
      printf( "could not find the specified mode\n" );
      return_value = false;
    }
  }
  else
  {
    printf( "no handle to operate on\n" );
    return_value = false;
  }

  return return_value;
}

bool                              video_interface_get_image(        // get an image from the video interface, returns true when an image was retrieved
    struct video_interface_handle_t * handle,                       // the interface to capture from
    struct image_t *                  image )                       // the image structure to populate with the image data
{
  bool                return_value;
  struct v4l2_buffer  buf;
  size_t              i;
  const uint8_t *     srcSlice;
  int                 srcStride;
  int                 dstStride;
  uint8_t *           dstData;

  // grab an image off the queue
  memset( &buf, 0, sizeof(buf) );
  buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory  = V4L2_MEMORY_USERPTR;
  if (ioctl( handle->fd, VIDIOC_DQBUF, &buf ) == 0)
  {
    // find the read image
    i = 0;
    while ((i < ARRAYSIZE(handle->buffer)) &&
           ((buf.m.userptr != (unsigned long)handle->buffer[i].start) ||
            (buf.length != handle->buffer[i].length)))
    {
      i++;
    }

    // do something with the image
    if (i < ARRAYSIZE(handle->buffer))
    {
      // enqueue the buffer to receive another image
      if (ioctl( handle->fd, VIDIOC_QBUF, &buf ) == 0)
      {
        // the following was helpful https://newbedev.com/how-to-convert-rgb-from-yuv420p-for-ffmpeg-encoder
        dstData   = (void *)image;
        srcSlice  = handle->buffer[i].start;
        srcStride = handle->capture_format.fmt.pix.bytesperline;
        dstStride = IMAGE_WIDTH * sizeof(struct pixel_format_RGB);

        sws_scale(
            handle->scaling,
            &srcSlice,
            &srcStride,
            0,
            handle->configured_height,
            &dstData,
            &dstStride );

        return_value = true;
      }
      else
      {
        fprintf( stderr, "VIDIOC_QBUF error %d, %s\n", errno, strerror( errno ) );
        return_value = false;
      }
    }
    else
    {
      fprintf( stderr, "could not find available image\n" );
      return_value = false;
    }
  }
  else
  {
    fprintf( stderr, "VIDIOC_DQBUF error %d, %s\n", errno, strerror( errno ) );
    return_value = false;
  }

  return return_value;
}
