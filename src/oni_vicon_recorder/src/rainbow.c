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

 @file  rainbow.c
 @brief simple rainbow color scaling utility
 
 @date  06/20/2012
 
 @author Max Bajracharya (maxb@jpl.nasa.gov)
 Mobility and Robotic Systems (347), JPL
*/


#include <arm_rgbd/rainbow.h>
    
// Rainbow color scale
static uint8_t rainbow[0x10000][3];


// Initialize
void rainbow_init()
{
  int i;
  float d;

  // This is the non-linearized rainbow scale.
  for (i = 0; i < 0x10000; i++)
  {
    d = 4 * ((double) i / 0x10000);

    if (d >= 0 && d < 1.0)
    {
      rainbow[i][0] = 0x00;
      rainbow[i][1] = (int) (d * 0xFF);
      rainbow[i][2] = 0xFF;
    }
    else if (d < 2.0)
    {
      d -= 1.0;
      rainbow[i][0] = 0x00;
      rainbow[i][1] = 0xFF;
      rainbow[i][2] = (int) ((1 - d) * 0xFF);
    }
    else if (d < 3.0)
    {
      d -= 2.0;
      rainbow[i][0] = (int) (d * 0xFF);
      rainbow[i][1] = 0xFF;
      rainbow[i][2] = 0x00;
    }
    else if (d < 4.0)
    {
      d -= 3.0;
      rainbow[i][0] = 0xFF;
      rainbow[i][1] = (int) ((1 - d) * 0xFF);
      rainbow[i][2] = 0x00;
    }
    else
    {
      rainbow[i][0] = 0xFF;
      rainbow[i][1] = 0x00;
      rainbow[i][2] = 0x00;
    }
  }
  
  return;
}

// Set a pixel using then rainbow scale.
void rainbow_set(uint8_t *dst, float value, float min, float max)
{
  int k;
  
  if (value < min)
    value = min;
  if (value > max)
    value = max;
  k = (int) ((value - min) / (max - min) * 0xFFFF);

  if (k < 0 || k > 0xFFFF)
  {
    dst[0] = 0x80;
    dst[1] = 0x80;
    dst[2] = 0x80;
    return;
  }

  dst[0] = rainbow[k][0];
  dst[1] = rainbow[k][1];
  dst[2] = rainbow[k][2];
  
  return;
}

