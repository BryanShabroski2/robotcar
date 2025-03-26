/*
 * scale_image_data.h
 *
 *  Created on: Feb 24, 2018
 *      Author: steveb
 */

#ifndef SCALE_IMAGE_DATA_H_
#define SCALE_IMAGE_DATA_H_

void scale_image_data( const struct pixel_format_RGB * unscaled_data, unsigned int unscaled_height, unsigned int unscaled_width, struct pixel_format_RGB *scaled_data, unsigned int horizontal_reduction, unsigned int vertical_reduction );

#endif /* SCALE_IMAGE_DATA_H_ */
