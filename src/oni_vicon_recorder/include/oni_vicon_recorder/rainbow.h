/**
 * Copyright (C) 2010, California Institute of Technology.
 * All Rights Reserved. U.S. Government Sponsorship Acknowledged.
 * Any commercial use must be negotiated with the Office of
 * Technology Transfer at the California Institute of Technology.
 * 
 * This software may be subject to U.S. export control laws. By
 * accepting this software, the user agrees to comply with all
 * applicable U.S. export laws and regulations. User has the
 * responsibility to obtain export licenses, or other export authority
 * as may be required before exporting such information to foreign
 * countries or providing access to foreign persons.
 *

 @file  rainbow.h
 @brief simple rainbow color scaling utility
 
 @date  06/20/2012
 
 @author Max Bajracharya (maxb@jpl.nasa.gov) 
 Mobility and Robotic Systems (347), JPL
*/

#ifndef RAINBOW_H
#define RAINBOW_H

#include <stdint.h>

#if defined __cplusplus
extern "C"
{
#endif
  
/** @brief Intialize rainbow scale
*/
void rainbow_init();

/** @brief Set a pixel using then rainbow scale.

    The input @c value is normalized to the interval 0 to 1, then
    converted to the corresponding false-color.  The @c src field must
    point to an RGB pixel (i.e., @c src[0..2] will be set.
 
    @param[out] dst Destination RGB pixel.
    @param[in] value Scalar value.
    @param[in] min,max Minimum and maximum scalar values.
*/
void rainbow_set(uint8_t *dst, float value, float min, float max);
    
#if defined __cplusplus
}
#endif

#endif
