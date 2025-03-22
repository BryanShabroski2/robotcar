/*
 * scale_image_data.c
 *
 *  Created on: Feb 24, 2018
 *      Author: steveb
 */

#include "pixel_format_RGB.h"

void scale_image_data(
    const struct pixel_format_RGB * unscaled_data,        /* the data to be scaled */
    unsigned int                    unscaled_height,      /* the height of the original image */
    unsigned int                    unscaled_width,       /* the width of the original image */
    struct pixel_format_RGB *       scaled_data,          /* the result of the rescaling */
    unsigned int                    horizontal_reduction, /* the reduction in horizontal scale */
    unsigned int                    vertical_reduction )  /* the reduction in vertical scale */
{
  unsigned int scaled_height;
  unsigned int scaled_width;
  unsigned int height_index;
  unsigned int width_index;

  scaled_height = unscaled_height/vertical_reduction;
  scaled_width  = unscaled_width/horizontal_reduction;
  for (height_index = 0; height_index < scaled_height; height_index++)
  {
    for (width_index = 0; width_index < scaled_width; width_index++)
    {
      scaled_data[height_index*scaled_width + width_index] = unscaled_data[height_index*scaled_width*vertical_reduction*horizontal_reduction + width_index*horizontal_reduction];
    }
  }

  return;
}
