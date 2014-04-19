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

 @file  kinect.h
 @brief Wrapper around ASUS/kinect device driver
 
 @date  06/20/2012
 
 @author Jeremy Ma (jeremy.c.ma@jpl.nasa.gov)
 Mobility and Robotic Systems (347), JPL
*/

// --------------------------------------------------------------------
// General Notes:
// --------------------------------------------------------------------
// The kinect sensor provides a 640x480 rgb depth map with color image
// at ~30Hz -- the two buffers are not synchronized however. There is
// also an intensity image which is used for perceiving the dot
// patters for structured light range calculations (640x480 -- though
// documentation seems to indicate it's 1280x1024, I cannot get that
// resolution output at the time of the writing). The depth buffer is
// always sent, but you can toggle the image stream that is sent with
// it; i.e. you can have the color image or the intensity image but
// not both.
// --------------------------------------------------------------------
// This driver assumes the following pakcages have been installed:
// - OpenNI (version 1.3.3.6)
// - PrimeSense (Sensor-Bin-Linux64-v5.0.2.3)
// --------------------------------------------------------------------

#ifndef HH_KINECT_HH
#define HH_KINECT_HH

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <ni/XnCppWrapper.h>

using namespace xn;

#define KINECT_IMAGE_COLS       640
#define KINECT_IMAGE_ROWS       480

// default parameters as taken from Kurt Konolige's detailed description of how the kinect works: 
// http://www.ros.org/wiki/kinect_calibration/technical
/*
// Comment: They are actually quite off! Instead we use OpenNI Registration
#define KINECT_RGB_FOCAL_LENGTH_DEFAULT     ((536.203514 + 534.836569)/2.0f)
#define KINECT_RGB_CENTER_COL_DEFAULT       311.986210
#define KINECT_RGB_CENTER_ROW_DEFAULT       245.607368
#define KINECT_IR_FOCAL_LENGTH_DEFAULT      ((573.921320 + 572.701134)/2.0f)
//#define KINECT_IR_CENTER_COL_DEFAULT        320
//#define KINECT_IR_CENTER_ROW_DEFAULT        240
#define KINECT_IR_CENTER_COL_DEFAULT        (310.488808) 
#define KINECT_IR_CENTER_ROW_DEFAULT        (247.529406) 
#define KINECT_RGB_TO_IR_X_DEFAULT          (-0.0184)
#define KINECT_RGB_TO_IR_Y_DEFAULT          (-0.0017)
#define KINECT_RGB_TO_IR_Z_DEFAULT          (-0.0019)
#define KINECT_RGB_TO_IR_ROLL_DEFAULT       (-0.4681 * M_PI/180.0f) // rad
#define KINECT_RGB_TO_IR_PITCH_DEFAULT      (-0.5396 * M_PI/180.0f) // rad
#define KINECT_RGB_TO_IR_YAW_DEFAULT        (0.1095 * M_PI/180.0f) // rad
*/
// Comment: They are actually quite off! 
#define KINECT_RGB_FOCAL_LENGTH_DEFAULT     525
#define KINECT_RGB_CENTER_COL_DEFAULT       320
#define KINECT_RGB_CENTER_ROW_DEFAULT       240
#define KINECT_IR_FOCAL_LENGTH_DEFAULT      580
#define KINECT_IR_CENTER_COL_DEFAULT        320
#define KINECT_IR_CENTER_ROW_DEFAULT        240
#define KINECT_RGB_TO_IR_X_DEFAULT          (-0.0254)
#define KINECT_RGB_TO_IR_Y_DEFAULT          (-0.00013)
#define KINECT_RGB_TO_IR_Z_DEFAULT          (-0.00218)
#define KINECT_RGB_TO_IR_ROLL_DEFAULT       0.0 // rad
#define KINECT_RGB_TO_IR_PITCH_DEFAULT      0.0 // rad
#define KINECT_RGB_TO_IR_YAW_DEFAULT        0.0 // rad

#define KINECT_VENDOR_ID                    0x45e
#define XTION_VENDOR_ID                     0x1d27

#ifdef __cplusplus
extern "C" {
#endif

  // a simple vector type
  typedef struct
  {
    double x;
    double y;
    double z;
  } vec3_t;
    
  
  // kinect params
  typedef struct
  {
    // focal length of the RGB camera
    float rgb_focal_length; // in pixels

    // focal length of the IR camera
    float ir_focal_length;  // in pixels
    XnUInt64 ir_focal_length_device;

    // ir camera center pixel
    vec3_t ir_camera_center;

    // rgb camera center pixel
    vec3_t rgb_camera_center;

    // baseline between ir and rgb
    XnDouble baseline;

    // Pixel size
    XnDouble pixel_size;
  
    // transform between IR and RGB camera
    double rgb_to_ir[4][4]; // ir camera as seen in the rgb frame
  
  } kinect_params_t;
  
  // kinect object
  typedef struct
  {
    // ----------------
    // context
    // ----------------
    Context *g_context;  

  
    // ----------------
    // generators
    // ----------------
  
    // generator for depth buffer
    DepthGenerator *g_depth;

    // generator for image buffer (this cannot be on the same time as the IR)
    ImageGenerator *g_image;

    // generator for IR buffer (this cannot be on the same time as the Color Image)
    IRGenerator *g_ir;

    // generator for audio (not tested)
    AudioGenerator *g_audio;


    // -----------------
    // metadata
    // -----------------
  
    // meta data to hold depth buffer
    DepthMetaData *md_depth;

    // meta data to hold image buffer
    ImageMetaData *md_image;

    // meta data to hold IR buffer
    IRMetaData *md_ir;

    // meta data to hold audio buffer
    AudioMetaData *md_audio;


    // ----------------
    // booleans
    // ----------------
  
    // booleans to signify whether the kinect component is on
    bool depth_on;
    bool image_on;
    bool image_present;
    bool ir_on;
    bool audio_on;

    // booleans to signify whether the kinect component is dirty or not
    bool depth_dirty;
    bool image_dirty;
    bool ir_dirty;
    bool audio_dirty;

    bool debayer_on;

    // the focal lengths and transforms in the params
    kinect_params_t params;

    // device info
    char device_name[1024];
    char vendor_info[1024];  
    unsigned short vendor_id;  
    uint32_t serial;

    XnUInt64 no_sample_value_;
    XnUInt64 shadow_value_;
  
  } kinect_t;

  // allocate memory
  kinect_t *kinect_alloc();

  // free memory
  void kinect_free(kinect_t *self);

  // initialize and start (can specify to use IR image instead of color image)
  int kinect_init_and_start(kinect_t *self, bool openni_reg = false, bool debayer = false, bool use_ir = false);

  // set 4x4 transform from xyzrpy
  void kinect_set_transform(double x, double y, double z, double roll, double pitch, double yaw, double G[4][4]);
  
  // set the params
  int kinect_set_params(kinect_t* self, const kinect_params_t params);
    
  // close the sensor
  int kinect_close(kinect_t *self);

  // capture
  int kinect_capture(kinect_t *self);

  // get the depth buffer
  int kinect_get_depth_buffer(kinect_t *self, uint16_t *depth_buffer, uint32_t depth_buffer_len);
  int kinect_get_f_depth_buffer(kinect_t *self, float *depth_buffer, uint32_t depth_buffer_len);

  // get the disparity buffer
  int kinect_get_disparity_buffer(kinect_t *self, float *disp_buffer, uint32_t disp_buffer_len, uint32_t line_step);

  // get the image buffer
  int kinect_get_image_buffer(kinect_t *self, uint8_t *img_buffer, uint32_t img_buffer_len);

  // debayer the image buffer
  int kinect_debayer_image_buffer(kinect_t *self, uint8_t *img_buffer, unsigned img_line_step, unsigned width, unsigned height);

  // get the gray buffer
  int kinect_get_gray_buffer(kinect_t *self, uint8_t *gray_buffer, uint32_t gray_buffer_len, unsigned gray_line_step);

  // get the image buffer for ASUS Xtion (YUV coding)
  int xtion_get_image_buffer(kinect_t *self, unsigned char *img_buffer, unsigned img_line_step, 
			     unsigned width, unsigned height);
  
  // get the gray buffer for ASUS Xtion (YUV coding)
  int xtion_get_gray_buffer(kinect_t *self, unsigned char *gray_buffer, unsigned gray_line_step, 
			    unsigned width, unsigned height);
  
  // get the ir buffer (16-bit image)
  int kinect_get_ir_buffer16(kinect_t *self, uint16_t *ir_buffer, uint32_t ir_buffer_len);

  // get the ir buffer (8-bit image, normalized)
  int kinect_get_ir_buffer8(kinect_t *self, uint8_t *ir_buffer, uint32_t ir_buffer_len);

  // get the xyz buffer (with matching color buffer if given)
  int kinect_depth_to_xyz_buffer(kinect_t *self,
                                 uint16_t *depth_buffer,   uint32_t depth_buffer_len,    // input
                                 uint8_t  *color_buffer,   uint32_t color_buffer_len,    // input
                                 float    *xyz_buffer,     uint32_t xyz_buffer_len,      // output
                                 uint8_t  *xyz_rgb_buffer, uint32_t xyz_rgb_buffer_len); // output                          

  int kinect_f_depth_to_xyz_buffer(kinect_t *self,
				   float *depth_buffer,   uint32_t depth_buffer_len,   // input
				   uint8_t  *color_buffer,   uint32_t color_buffer_len,   // input
				   float    *xyz_buffer,     uint32_t xyz_buffer_len,     // output
				   uint8_t  *xyz_rgb_buffer, uint32_t xyz_rgb_buffer_len); // output

  // check whether depth stream or image stream are valid
  bool kinect_depth_stream_running(kinect_t *self);
  bool kinect_image_stream_running(kinect_t *self);

  // check whether openni depth registration is turned on
  bool kinect_depth_registration_running(kinect_t *self);
  
#ifdef __cplusplus
}
#endif


#endif
