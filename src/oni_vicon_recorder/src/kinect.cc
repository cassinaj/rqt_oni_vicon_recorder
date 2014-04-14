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

 @file  kinect.cc
 @brief Wrapper around ASUS/kinect device driver
 
 @date  07/06/2012
 
 @author Jeremy Ma (jeremy.c.ma@jpl.nasa.gov)
 Mobility and Robotic Systems (347), JPL
 @author Jeannette Bohg (jbohg@tuebingen.mpg.de)
 Autonomous Motion Lab, MPI for Intelligent Systems, Tuebingen, Germany
*/

/* Debayering function based on openni_image_bayer_grgb.cpp in OpenNI ROS package*/

#include <oni_vicon_recorder/kinect.h>

#include <string>
#include <limits>

#include <opencv/cv.h>

#define CLIP_CHAR(c) ((c)>255?255:(c)<0?0:(c))
#define AVG(a,b) (((int)(a) + (int)(b)) >> 1)
#define AVG3(a,b,c) (((int)(a) + (int)(b) + (int)(c)) / 3)
#define AVG4(a,b,c,d) (((int)(a) + (int)(b) + (int)(c) + (int)(d)) >> 2)
#define WAVG4(a,b,c,d,x,y)  ( ( ((int)(a) + (int)(b)) * (int)(x) + ((int)(c) + (int)(d)) * (int)(y) ) / ( 2 * ((int)(x) + (int(y))) ) )

namespace kinect
{
  static cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  static cv::Mat T = cv::Mat::zeros(3, KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS, CV_32F);
  static std::vector<cv::Point3f> points(KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS, cv::Point3f(0.0, 0.0, 0.0));
  // points.resize(KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS);

}

vec3_t vec3_transform(double G[4][4], vec3_t pt)
{
  vec3_t pt_new;

  pt_new.x = G[0][0] * pt.x + G[0][1] * pt.y + G[0][2] * pt.z + G[0][3];
  pt_new.y = G[1][0] * pt.x + G[1][1] * pt.y + G[1][2] * pt.z + G[1][3];
  pt_new.z = G[2][0] * pt.x + G[2][1] * pt.y + G[2][2] * pt.z + G[2][3];

  return pt_new;
}

// allocate memory
kinect_t *kinect_alloc()
{
  kinect_t *self = (kinect_t*)calloc(1, sizeof(kinect_t));  
  
  self->depth_on = false;
  self->image_on = false;
  self->ir_on    = false;
  self->audio_on = false;

  self->depth_dirty = false;
  self->image_dirty = false;
  self->ir_dirty    = false;
  self->audio_dirty = false;

  self->serial = 0;

  memset(&self->params, 0, sizeof(kinect_params_t));
  
  // set params to default values
  self->params.rgb_focal_length     = KINECT_RGB_FOCAL_LENGTH_DEFAULT;
  self->params.ir_focal_length      = KINECT_IR_FOCAL_LENGTH_DEFAULT;
  self->params.rgb_camera_center.x  = KINECT_RGB_CENTER_COL_DEFAULT;
  self->params.rgb_camera_center.y  = KINECT_RGB_CENTER_ROW_DEFAULT;
  self->params.rgb_camera_center.z  = 0;
  self->params.ir_camera_center.x   = KINECT_IR_CENTER_COL_DEFAULT;
  self->params.ir_camera_center.y   = KINECT_IR_CENTER_ROW_DEFAULT;
  self->params.ir_camera_center.z   = 0;

  kinect_set_transform(KINECT_RGB_TO_IR_X_DEFAULT,
                       KINECT_RGB_TO_IR_Y_DEFAULT,
                       KINECT_RGB_TO_IR_Z_DEFAULT,
                       KINECT_RGB_TO_IR_ROLL_DEFAULT,
                       KINECT_RGB_TO_IR_PITCH_DEFAULT,
                       KINECT_RGB_TO_IR_YAW_DEFAULT,
                       self->params.rgb_to_ir);

  return self;
}

// free memory
void kinect_free(kinect_t *self)
{
  if(self)
    free(self);
    
  return;
}

// initialize and start (can specify to use IR image instead of color image)
int kinect_init_and_start(kinect_t *self, bool use_openni_reg, bool debayer, bool use_ir)
{
  if(!self)
    {
      fprintf(stderr, "kinect device must be allocated first!\n");
      return -1;
    }
  
  self->g_context = new Context();
  
  self->g_depth = new DepthGenerator();
  self->g_image = new ImageGenerator();
  self->g_ir    = new IRGenerator();
  self->g_audio = new AudioGenerator();
  
  self->md_depth = new DepthMetaData();
  self->md_image = new ImageMetaData();
  self->md_ir    = new IRMetaData();
  self->md_audio = new AudioMetaData();

  self->depth_on = false;
  self->image_on = false;
  self->ir_on    = false;
  self->audio_on = false;
 
  self->debayer_on = debayer;
 
  // open device
  fprintf(stderr, "opening kinect device ...");
  
  XnStatus ret = XN_STATUS_OK;
  EnumerationErrors errors;
  XnMapOutputMode Mode;
  ret = self->g_context->Init();
  if(ret!=XN_STATUS_OK)
    {
      char buffer[1024];
      fprintf(stderr, "failed to initialize kinect!\n");    
      errors.ToString((XnChar*)buffer, (XnUInt32)(sizeof(buffer)));
      fprintf(stderr, "error: %s\n", buffer);
    } else
    fprintf(stderr, "initialized kinect!\n");    
  
  
  // -------------------------
  // initialize depth stream
  // -------------------------
  fprintf(stderr, "starting depth stream!\n");    
  ret = self->g_context->CreateAnyProductionTree(XN_NODE_TYPE_DEPTH, NULL, *(self->g_depth), &errors);
  if (ret == XN_STATUS_NO_NODE_PRESENT)
    {
      XnChar strError[1024];
      errors.ToString(strError, 1024);
      fprintf(stderr, "No Node Present:\n%s \n", (char*)strError);
      return -1;
    }
  else if (ret != XN_STATUS_OK)
    {
      XnChar strError[1024];
      errors.ToString(strError, 1024);
      fprintf(stderr, "XN_STATUS not ok:\n%s \n", (char*)strError);
      return -1;
    }

  ret = self->g_depth->GetIntProperty ((XnChar*)"NoSampleValue", self->no_sample_value_);
  if (ret != XN_STATUS_OK)
    {
      XnChar strError[1024];
      errors.ToString(strError, 1024);
      fprintf(stderr, "Could not get NoSampleValue:\n%s \n", (char*)strError);
      return -1;
    }
  
  ret = self->g_depth->GetIntProperty ((XnChar*)"ShadowValue", self->shadow_value_);
  if (ret != XN_STATUS_OK)
    {
      XnChar strError[1024];
      errors.ToString(strError, 1024);
      fprintf(stderr, "Could no get ShadowValue:\n%s \n", (char*)strError);
      return -1;
    }

  

  ret =  self->g_depth->GetIntProperty ((XnChar*)"ZPD", self->params.ir_focal_length_device);
  if (ret != XN_STATUS_OK)
    {
      XnChar strError[1024];
      errors.ToString(strError, 1024);
      fprintf(stderr, "Could not get ZPD:\n%s \n", (char*)strError);
      return -1;
    } /*
	else {
	printf("Focal length of IR camera: %f\n", (double)self->params.ir_focal_length_device);
	}
      */
  
  
  
  ret = self->g_depth->GetRealProperty ("ZPPS", self->params.pixel_size);
  if (ret != XN_STATUS_OK)
    {
      XnChar strError[1024];
      errors.ToString(strError, 1024);
      fprintf(stderr, "%s \n", (char*)strError);
      return -1;
    }/* else {
	printf("Pixel Size of IR camera: %f\n", (double)self->params.pixel_size);
	}
     */
 

  ret = self->g_depth->GetRealProperty ((XnChar*)"LDDIS", self->params.baseline);
  if (ret != XN_STATUS_OK)
    {
      XnChar strError[1024];
      errors.ToString(strError, 1024);
      fprintf(stderr, "%s \n", (char*)strError);
      return -1;
    } else {
    // convert to meters
    self->params.baseline*=0.01f;
    printf("Baseline (in metres) %f\n", (double)self->params.baseline);
  }

  // start generating
  self->g_depth->StartGenerating();
  if(self->g_depth->IsGenerating())
    self->depth_on = true;

  // set the resolution
  fprintf(stderr, "Setting resolution for depth image!\n");
  self->g_depth->GetMapOutputMode(Mode);
  Mode.nXRes = Resolution((XnResolution)XN_RES_VGA).GetXResolution();
  Mode.nYRes = Resolution((XnResolution)XN_RES_VGA).GetYResolution();
  Mode.nFPS = 30;
  printf("X and Y Res: %d %d\n", Mode.nXRes, Mode.nYRes);
  ret = self->g_depth->SetMapOutputMode(Mode);
  if(ret != XN_STATUS_OK)
    {
      XnChar strError[1024];
      errors.ToString(strError, 1024);
      fprintf(stderr, "%s \n", (char*)strError);
      return -1;
    }
  
  
  // scale factor needed because actual size of bayer encoded ir image (XN_SXGA_X_RES)
  // might be different from output resolution -> focal length has to be scaled to get focal length in pixels
  float scale = Mode.nXRes / (float)XN_SXGA_X_RES;
  self->params.ir_focal_length 
    = (float)(self->params.ir_focal_length_device / self->params.pixel_size * scale);
  

  if(use_ir)
    {
      // -------------------------
      // initialize IR stream
      // -------------------------    
      fprintf(stderr, "starting IR stream!\n");
    
      ret = self->g_context->CreateAnyProductionTree(XN_NODE_TYPE_IR, NULL, (*self->g_ir), &errors);
      if (ret == XN_STATUS_NO_NODE_PRESENT)
	{
	  XnChar strError[1024];
	  errors.ToString(strError, 1024);
	  fprintf(stderr, "%s \n", (char*)strError);
	  return -1;
	}
      else if (ret != XN_STATUS_OK)
	{
	  XnChar strError[1024];
	  errors.ToString(strError, 1024);
	  fprintf(stderr, "%s \n", (char*)strError);
	  return -1;
	}

      // start generating
      self->g_ir->StartGenerating();
    
      // set the resolution
      fprintf(stderr, "Setting resolution for IR Image!\n");
      self->g_ir->GetMapOutputMode(Mode);
      Mode.nXRes = Resolution((XnResolution)XN_RES_VGA).GetXResolution();
      Mode.nYRes = Resolution((XnResolution)XN_RES_VGA).GetYResolution();
      ret = self->g_ir->SetMapOutputMode(Mode);
      if(ret != XN_STATUS_OK)
	{
	  XnChar strError[1024];
	  errors.ToString(strError, 1024);
	  fprintf(stderr, "%s \n", (char*)strError);
	  return -1;
	}

      // set the framerate
      fprintf(stderr, "setting framerate!\n");
      self->g_ir->GetMapOutputMode(Mode);
      Mode.nFPS = 30;
      ret = self->g_ir->SetMapOutputMode(Mode);
      if (ret != XN_STATUS_OK)
	{
	  XnChar strError[1024];
	  errors.ToString(strError, 1024);
	  fprintf(stderr, "%s \n", (char*)strError);
	  return -1;
	}

      // start generating
      self->g_ir->StartGenerating();
      if(self->g_ir->IsGenerating())
	self->ir_on = true;
      
    } else  {
    // ------------------------------------------------
    // initialize image stream if device has rgb camera
    // ------------------------------------------------
    
    fprintf(stderr, "starting image stream!\n");   
    ret = self->g_context->CreateAnyProductionTree(XN_NODE_TYPE_IMAGE, NULL, *(self->g_image), &errors);
    if (ret == XN_STATUS_NO_NODE_PRESENT)
      {
	std::cout << "Device doesn't have RGB camera." << std::endl;
	
	XnChar strError[1024];
	errors.ToString(strError, 1024);
	fprintf(stderr, "%s \n", (char*)strError);
	self->image_present = false;
	self->image_on = false;
      }
    else if (ret != XN_STATUS_OK)
      {
	XnChar strError[1024];
	errors.ToString(strError, 1024);
	fprintf(stderr, "%s \n", (char*)strError);
	return -1;
	
      } else {
      
      
      // set flag to indicate that rgb camera is running
      self->image_present = true;
       
      // set the resolution
      fprintf(stderr, "Setting resolution for RGB Image!\n");
      self->g_image->GetMapOutputMode(Mode);
      Mode.nXRes = Resolution((XnResolution)XN_RES_VGA).GetXResolution();
      Mode.nYRes = Resolution((XnResolution)XN_RES_VGA).GetYResolution();
      Mode.nFPS = 30;
      printf("X and Y Res: %d %d\n", Mode.nXRes, Mode.nYRes);
      ret = self->g_image->SetMapOutputMode(Mode);
      if(ret != XN_STATUS_OK)
	{
	  XnChar strError[1024];
	  errors.ToString(strError, 1024);
	  fprintf(stderr, "%s \n", (char*)strError);
	  return -1;
	}
    }
    
    // ---------------------------
    // get device info
    // ---------------------------
    char buff[1024];
    uint32_t buff_len = 1024;
    ProductionNode prd_node;

    unsigned short vendorID;
    unsigned short productID;
    unsigned char bus;
    unsigned char address;

    ret = self->g_context->FindExistingNode(XN_NODE_TYPE_DEVICE, prd_node);
    ret = xnGetDeviceName(prd_node.GetHandle(), (XnChar*)buff, (XnUInt32*)(&buff_len));
    snprintf(self->device_name, sizeof(self->device_name), "%s", buff);

    ret = xnGetVendorSpecificData(prd_node.GetHandle(), (XnChar*)buff, (XnUInt32*)(&buff_len));
    snprintf(self->vendor_info, sizeof(self->vendor_info), "%s", buff);
    
    ret = xnGetSerialNumber(prd_node.GetHandle(), (XnChar*)buff, (XnUInt32*)(&buff_len));
    self->serial = atoi(buff);
    
    std::string connectionString = prd_node.GetInfo().GetCreationInfo();
    sscanf (connectionString.c_str(), "%hx/%hx@%hhu/%hhu", &vendorID, &productID, &bus, &address);
    self->vendor_id = vendorID;
    
    fprintf(stderr, "device name: %s\n", self->device_name);
    fprintf(stderr, "vendor info: %s\n", self->vendor_info);
    fprintf(stderr, "vendor id: %d\n", self->vendor_id);
    fprintf(stderr, "serial: %u\n", self->serial);
  
    /*
      ret = xnGetDeviceName(self->g_context->);
      ret != XN_STATUS_OK)  
    */
  
    // ---------------------------
    // set the pixel format based on VendorID
    // Asus Xtion uses YUV422
    // MS Kinect uses RGB24 or Bayer pattern
    // ---------------------------
    
    // ASUS Xtion
    if (self->vendor_id == XTION_VENDOR_ID && self->image_present)
      { 
	std::cout << "Treating device as XTION device" << std::endl;

	ret = self->g_image->SetIntProperty ("InputFormat", 5);
	if(ret != XN_STATUS_OK)
	  {
	    XnChar strError[1024];
	    errors.ToString(strError, 1024);
	    fprintf(stderr, "%s \n", (char*)strError);
	    return -1;
	  }
	// Depth Registration (Hardware)
	self->g_depth->SetIntProperty ("RegistrationType", 1);
	// RGB Pixel Format
        ret = self->g_image->SetPixelFormat (XN_PIXEL_FORMAT_YUV422);
	printf("Xtion Pixel Format %d\n", self->g_image->GetPixelFormat ());
	if(ret != XN_STATUS_OK)
	  {
	    XnChar strError[1024];
	    errors.ToString(strError, 1024);
	    fprintf(stderr, "%s \n", (char*)strError);
	    return -1;
	  }
	
      } else if(self->vendor_id == KINECT_VENDOR_ID) {
      // MS Kinect 
      // Depth Registration (Software)
      self->g_depth->SetIntProperty ("RegistrationType", 2);
      // RGB Pixel Format -> kinect specific format -> set to grayscale format to get bayer pattern directly
      if(self->debayer_on){
	ret = self->g_image->SetIntProperty ("InputFormat", 6);
	ret = self->g_image->SetPixelFormat (XN_PIXEL_FORMAT_GRAYSCALE_8_BIT);
      } else {
	// RGB24 has strong interlace pattern (only use in conjunction with kinect_get_image_buffer)
	ret = self->g_image->SetIntProperty ("InputFormat", 6);
	ret = self->g_image->SetPixelFormat (XN_PIXEL_FORMAT_RGB24);
      }
      
      
      printf("Kinect Pixel Format %d\n", self->g_image->GetPixelFormat ());
      if(ret != XN_STATUS_OK)
	{
	  XnChar strError[1024];
	  errors.ToString(strError, 1024);
	  fprintf(stderr, "%s \n", (char*)strError);
	  return -1;
	}
      
    }
    
    // start generating
    if(self->image_present) {
      self->g_image->StartGenerating();
      if(self->g_image->IsGenerating()){
	self->image_on = true;
	std::cout << "ImageGenerator generating. " << std::endl;
      } else {
	std::cout << "ImageGenerator NOT generating. " << std::endl;
      }
    }
  }

  // USC/MPI Fix
  // Turn on the depth2rgb registration if depth and image generators are running
  if(use_openni_reg){
    if(self->g_depth->IsValid() && self->g_image->IsValid())
      {
	if (!self->g_depth->GetAlternativeViewPointCap ().IsViewPointAs (*(self->g_image)))
	  {
	    if (self->g_depth->GetAlternativeViewPointCap ().IsViewPointSupported (*(self->g_image)))
	      {
		XnStatus status = self->g_depth->GetAlternativeViewPointCap ().SetViewPoint (*(self->g_image));
		if (status != XN_STATUS_OK)
		  printf("turning registration on failed. Reason: %s\n", xnGetStatusString (status));
	      } else {
	      printf("turning registration on failed. Reason: unsopported viewpoint\n");
	    }
	  } else {
	  printf("turning registration on failed. Reason: Is already turned on\n");
	}
      } else {
      printf("Device does not provide image + depth stream\n");
    }
  } else printf("Using manual registration\n");
  
  self->depth_dirty = false;
  self->image_dirty = false;
  self->ir_dirty    = false;
  self->audio_dirty = false;
  
  return 0;
}

// assuming fixed angles (NOT euler)
void kinect_set_transform(double x,
                          double y,
                          double z,
                          double roll,
                          double pitch,
                          double yaw,
                          double G[4][4])
{
  
  double a, b, g;
  a = roll;
  b = pitch;
  g = yaw;

  G[0][0] = cos(b)*cos(g);
  G[0][1] = -cos(b)*sin(g);
  G[0][2] = sin(b);
  G[0][3] = x;

  G[1][0] = sin(a)*sin(b)*cos(g) + cos(a)*sin(g);
  G[1][1] = -sin(a)*sin(b)*sin(g) + cos(a)*cos(g);
  G[1][2] = -sin(a)*cos(b);
  G[1][3] = y;

  G[2][0] = -cos(a)*sin(b)*cos(g) + sin(a)*sin(g);
  G[2][1] = cos(a)*sin(b)*sin(g) + sin(a)*cos(g);
  G[2][2] = cos(a)*cos(b);
  G[2][3] = z;

  G[3][0] = 0;
  G[3][1] = 0;
  G[3][2] = 0;
  G[3][3] = 1;
    
  return;
}

// set the params
int kinect_set_params(kinect_t* self, const kinect_params_t params)
{
  self->params = params;
  return 0;
}


// close the sensor
int kinect_close(kinect_t *self)
{
  fprintf(stderr, "closing kinect device ...\n");
  
  self->g_depth->Release();
  self->g_image->Release();
  self->g_ir->Release();
  self->g_audio->Release();
  self->g_context->Release();

  delete self->g_depth;
  delete self->g_image;
  delete self->g_ir;
  delete self->g_audio;
  delete self->g_context;
  
  return 0;
}

// capture
int kinect_capture(kinect_t *self)
{
  // read frame    
  XnStatus rc = XN_STATUS_OK;
  
  // you can specify which sensor to wait for an update for here:
  /*
    rc = g_Context.WaitOneUpdateAll(g_Depth);
    rc = g_Context.WaitOneUpdateAll(g_Image);
    rc = g_Context.WaitOneUpdateAll(g_IR);
    rc = g_Context.WaitOneUpdateAll(g_Audio);
  */
  
  self->depth_dirty = false;
  self->image_dirty = false;
  self->ir_dirty    = false;
  self->audio_dirty = false;
    
  // or just wait for any update on any sensor
  rc = self->g_context->WaitAnyUpdateAll();    
  if (rc != XN_STATUS_OK)
    {
      fprintf(stderr, "error: %s\n", xnGetStatusString(rc));
      return -1;
    }

  // check depth sensor
  if(self->depth_on)
    {
      if(self->g_depth->IsValid())
	{
	  self->g_depth->GetMetaData((*self->md_depth));
	  self->depth_dirty = true;
	}
    }

  // check image sensor
  if(self->image_on)
    {
      if(self->g_image->IsValid())
	{
	  self->g_image->GetMetaData((*self->md_image));
	  self->image_dirty = true;
	}
    } 
  
  // check ir sensor
  if(self->ir_on)
    {
      if(self->g_ir->IsValid())
	{
	  self->g_ir->GetMetaData((*self->md_ir));
	  self->ir_dirty = true;
	}
    }
  
  
  return 0;
}

// get the depth buffer
int kinect_get_depth_buffer(kinect_t *self, uint16_t *depth_buffer, uint32_t depth_buffer_len)
{
  if(depth_buffer_len < (KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS * sizeof(uint16_t)))
    {
      fprintf(stderr, "buffer len too small!\n");
      return -1;
    }
  
  if(!self->depth_on)
    {
      fprintf(stderr, "depth stream is not on!\n");
      return -1;
    }
  
  memcpy(depth_buffer, (char*)self->md_depth->Data(), sizeof(uint16_t) * KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS);
  
  return 0;
}

// get the depth buffer
int kinect_get_disparity_buffer(kinect_t *self, float *disp_buffer, uint32_t disp_buffer_len, uint32_t line_step)
{
  if(disp_buffer_len < (KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS * sizeof(uint16_t)))
    {
      fprintf(stderr, "buffer len too small!\n");
      return -1;
    }
  
  if(!self->depth_on)
    {
      fprintf(stderr, "depth stream is not on!\n");
      return -1;
    }
  
  unsigned xStep = self->md_depth->XRes () / KINECT_IMAGE_COLS;
  unsigned ySkip = (self->md_depth->YRes () / KINECT_IMAGE_ROWS - 1) * self->md_depth->XRes ();

  unsigned bufferSkip = line_step - KINECT_IMAGE_COLS * sizeof (float);

  // Fill in the depth image data
  // iterate over all elements and fill disparity matrix: disp[x,y] = f * b / z_distance[x,y];
  // focal length is for the native image resolution -> focal_length = focal_length_ / xStep;
  float constant = self->params.ir_focal_length * self->params.baseline * 1000.0f/ (float) xStep;
  
  //  printf("xStep %d and constant %f\n", xStep, constant);

  for (unsigned yIdx = 0, depthIdx = 0; yIdx < KINECT_IMAGE_ROWS; ++yIdx, depthIdx += ySkip)
    {
      for (unsigned xIdx = 0; xIdx < KINECT_IMAGE_COLS; ++xIdx, depthIdx += xStep, ++disp_buffer)
	{
	  if ((*self->md_depth)[depthIdx] == 0 ||
	      (*self->md_depth)[depthIdx] == self->no_sample_value_ ||
	      (*self->md_depth)[depthIdx] == self->shadow_value_)
	    *disp_buffer = 0.0;
	  else {
	    *disp_buffer = constant / (double) (*self->md_depth)[depthIdx];
	    //	printf("disp %f\n", constant / (double) (*self->md_depth)[depthIdx]);
	  }
	}

      // if we have padding
      if (bufferSkip > 0)
	{
	  char* cBuffer = reinterpret_cast<char*> (disp_buffer);
	  disp_buffer = reinterpret_cast<float*> (cBuffer + bufferSkip);
	}
    }
  
  return 0;
}

int kinect_get_f_depth_buffer(kinect_t *self, float *depth_buffer, uint32_t depth_buffer_len)
{
  if(depth_buffer_len < (KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS * sizeof(float)))
    {
      fprintf(stderr, "buffer len too small!\n");
      return -1;
    }

  if(!self->depth_on)
    {
      fprintf(stderr, "depth stream is not on!\n");
      return -1;
    }
  
  // step and padding skip for source image
  unsigned xStep = self->md_depth->XRes () / KINECT_IMAGE_COLS;
  unsigned ySkip = (self->md_depth->YRes () / KINECT_IMAGE_ROWS - 1) * self->md_depth->XRes();
  
  // Fill in the depth image data, converting mm to m
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  unsigned depthIdx = 0;

  for (unsigned yIdx = 0; yIdx < KINECT_IMAGE_ROWS; ++yIdx, depthIdx += ySkip)
    {
      for (unsigned xIdx = 0; xIdx < KINECT_IMAGE_COLS; ++xIdx, depthIdx += xStep, ++depth_buffer)
	{
	  /// @todo Different values for these cases
	  if ((*(self->md_depth))[depthIdx] == 0 ||
	      (*(self->md_depth))[depthIdx] == self->no_sample_value_ ||
	      (*(self->md_depth))[depthIdx] == self->shadow_value_)
	    *depth_buffer = bad_point;
	  else
	    {	
	      *depth_buffer = (float) (*(self->md_depth))[depthIdx];
	    }
	}
    }

  return 0;
}





// get the image buffer
int kinect_get_image_buffer(kinect_t *self, uint8_t *img_buffer, uint32_t img_buffer_len)
{
  if(img_buffer_len < (KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS * 3 * sizeof(uint8_t)))
    {
      fprintf(stderr, "buffer len too small!\n");
      return -1;
    }

  if(!self->image_on)
    {
      fprintf(stderr, "image stream is not on!\n");
      return -1;
    }
  
  memcpy(img_buffer, (uint8_t*)self->md_image->Data(), sizeof(uint8_t) * KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS * 3);
  
  return 0;
}

// debayer the image buffer
int kinect_debayer_image_buffer(kinect_t *self, uint8_t *img_buffer, unsigned img_line_step, unsigned width, unsigned height)
{
  if (self->md_image->XRes() != width || self->md_image->YRes() != height)
    {
      fprintf(stderr, "buffer len too small!\n");
      return -1;
    }

  if(!self->image_on)
    {
      fprintf(stderr, "image stream is not on!\n");
      return -1;
    }
  
  register const XnUInt8 *bayer_pixel = self->md_image->Data();
  register unsigned yIdx, xIdx;

  // padding skip for destination image
  unsigned img_line_skip = img_line_step - width * 3;
  
  int bayer_line_step = self->md_image->XRes ();
  int bayer_line_step2 = self->md_image->XRes () << 1;

  int dh, dv;

  // first two pixel values for first two lines
  // Bayer         0 1 2
  //         0     G r g
  // line_step     b g b
  // line_step2    g r g

  img_buffer[3] = img_buffer[0] = bayer_pixel[1]; // red pixel
  img_buffer[1] = bayer_pixel[0]; // green pixel
  img_buffer[img_line_step + 2] = img_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

  // Bayer         0 1 2
  //         0     g R g
  // line_step     b g b
  // line_step2    g r g
  //rgb_pixel[3] = bayer_pixel[1];
  img_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
  img_buffer[img_line_step + 5] = img_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

  // BGBG line
  // Bayer         0 1 2
  //         0     g r g
  // line_step     B g b
  // line_step2    g r g
  img_buffer[img_line_step + 3] = img_buffer[img_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
  img_buffer[img_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
  //rgb_pixel[img_line_step + 2] = bayer_pixel[line_step];

  // pixel (1, 1)  0 1 2
  //         0     g r g
  // line_step     b G b
  // line_step2    g r g
  //rgb_pixel[img_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
  img_buffer[img_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  //rgb_pixel[img_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

  img_buffer += 6;
  bayer_pixel += 2;
  // rest of the first two lines
  for (xIdx = 2; xIdx < width - 2; xIdx += 2, img_buffer += 6, bayer_pixel += 2)
    {
      // GRGR line
      // Bayer        -1 0 1 2
      //           0   r G r g
      //   line_step   g b g b
      // line_step2    r g r g
      img_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
      img_buffer[1] = bayer_pixel[0];
      img_buffer[2] = bayer_pixel[bayer_line_step + 1];

      // Bayer        -1 0 1 2
      //          0    r g R g
      //  line_step    g b g b
      // line_step2    r g r g
      img_buffer[3] = bayer_pixel[1];
      img_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
      img_buffer[img_line_step + 5] = img_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

      // BGBG line
      // Bayer         -1 0 1 2
      //         0      r g r g
      // line_step      g B g b
      // line_step2     r g r g
      img_buffer[img_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
      img_buffer[img_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
      img_buffer[img_line_step + 2] = bayer_pixel[bayer_line_step];

      // Bayer         -1 0 1 2
      //         0      r g r g
      // line_step      g b G b
      // line_step2     r g r g
      img_buffer[img_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
      img_buffer[img_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      //rgb_pixel[img_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
    }

  // last two pixel values for first two lines
  // GRGR line
  // Bayer        -1 0 1
  //           0   r G r
  //   line_step   g b g
  // line_step2    r g r
  img_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
  img_buffer[1] = bayer_pixel[0];
  img_buffer[img_line_step + 5] = img_buffer[img_line_step + 2] = img_buffer[5] = img_buffer[2] = bayer_pixel[bayer_line_step];

  // Bayer        -1 0 1
  //          0    r g R
  //  line_step    g b g
  // line_step2    r g r
  img_buffer[3] = bayer_pixel[1];
  img_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
  //rgb_pixel[5] = bayer_pixel[line_step];

  // BGBG line
  // Bayer        -1 0 1
  //          0    r g r
  //  line_step    g B g
  // line_step2    r g r
  img_buffer[img_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
  img_buffer[img_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
  //rgb_pixel[img_line_step + 2] = bayer_pixel[line_step];

  // Bayer         -1 0 1
  //         0      r g r
  // line_step      g b G
  // line_step2     r g r
  img_buffer[img_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
  img_buffer[img_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  //rgb_pixel[img_line_step + 5] = bayer_pixel[line_step];

  bayer_pixel += bayer_line_step + 2;
  img_buffer += img_line_step + 6 + img_line_skip;
  // main processing
  for (yIdx = 2; yIdx < height - 2; yIdx += 2)
    {
      // first two pixel values
      // Bayer         0 1 2
      //        -1     b g b
      //         0     G r g
      // line_step     b g b
      // line_step2    g r g

      img_buffer[3] = img_buffer[0] = bayer_pixel[1]; // red pixel
      img_buffer[1] = bayer_pixel[0]; // green pixel
      img_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]); // blue;

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g R g
      // line_step     b g b
      // line_step2    g r g
      //rgb_pixel[3] = bayer_pixel[1];
      img_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
      img_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

      // BGBG line
      // Bayer         0 1 2
      //         0     g r g
      // line_step     B g b
      // line_step2    g r g
      img_buffer[img_line_step + 3] = img_buffer[img_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
      img_buffer[img_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
      img_buffer[img_line_step + 2] = bayer_pixel[bayer_line_step];

      // pixel (1, 1)  0 1 2
      //         0     g r g
      // line_step     b G b
      // line_step2    g r g
      //rgb_pixel[img_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      img_buffer[img_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      img_buffer[img_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

      img_buffer += 6;
      bayer_pixel += 2;
      // continue with rest of the line
      for (xIdx = 2; xIdx < width - 2; xIdx += 2, img_buffer += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer        -1 0 1 2
          //          -1   g b g b
          //           0   r G r g
          //   line_step   g b g b
          // line_step2    r g r g
          img_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          img_buffer[1] = bayer_pixel[0];
          img_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

          // Bayer        -1 0 1 2
          //          -1   g b g b
          //          0    r g R g
          //  line_step    g b g b
          // line_step2    r g r g

          dh = abs (bayer_pixel[0] - bayer_pixel[2]);
          dv = abs (bayer_pixel[-bayer_line_step + 1] - bayer_pixel[bayer_line_step + 1]);

          if (dv == 0 && dh == 0)
            img_buffer[4] = AVG4 (bayer_pixel[1 - bayer_line_step], bayer_pixel[1 + bayer_line_step], bayer_pixel[0], bayer_pixel[2]);
          else
            img_buffer[4] = WAVG4 (bayer_pixel[1 - bayer_line_step], bayer_pixel[1 + bayer_line_step], bayer_pixel[0], bayer_pixel[2], dh, dv);
          img_buffer[3] = bayer_pixel[1];
          img_buffer[5] = AVG4 (bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step], bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

          // BGBG line
          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g B g b
          // line_step2     r g r g
          img_buffer[img_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
          img_buffer[img_line_step + 2] = bayer_pixel[bayer_line_step];

          dv = abs (bayer_pixel[0] - bayer_pixel[bayer_line_step2]);
          dh = abs (bayer_pixel[bayer_line_step - 1] - bayer_pixel[bayer_line_step + 1]);

          if (dv == 0 && dh == 0)
            img_buffer[img_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          else
            img_buffer[img_line_step + 1] = WAVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1], dh, dv);

          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g b G b
          // line_step2     r g r g
          img_buffer[img_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          img_buffer[img_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          img_buffer[img_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
        }

      // last two pixels of the line
      // last two pixel values for first two lines
      // GRGR line
      // Bayer        -1 0 1
      //           0   r G r
      //   line_step   g b g
      // line_step2    r g r
      img_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
      img_buffer[1] = bayer_pixel[0];
      img_buffer[img_line_step + 5] = img_buffer[img_line_step + 2] = img_buffer[5] = img_buffer[2] = bayer_pixel[bayer_line_step];

      // Bayer        -1 0 1
      //          0    r g R
      //  line_step    g b g
      // line_step2    r g r
      img_buffer[3] = bayer_pixel[1];
      img_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
      //rgb_pixel[5] = bayer_pixel[line_step];

      // BGBG line
      // Bayer        -1 0 1
      //          0    r g r
      //  line_step    g B g
      // line_step2    r g r
      img_buffer[img_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
      img_buffer[img_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
      //rgb_pixel[img_line_step + 2] = bayer_pixel[line_step];

      // Bayer         -1 0 1
      //         0      r g r
      // line_step      g b G
      // line_step2     r g r
      img_buffer[img_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
      img_buffer[img_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      //rgb_pixel[img_line_step + 5] = bayer_pixel[line_step];

      bayer_pixel += bayer_line_step + 2;
      img_buffer += img_line_step + 6 + img_line_skip;
    }

  //last two lines
  // Bayer         0 1 2
  //        -1     b g b
  //         0     G r g
  // line_step     b g b

  img_buffer[img_line_step + 3] = img_buffer[img_line_step ] = img_buffer[3] = img_buffer[0] = bayer_pixel[1]; // red pixel
  img_buffer[1] = bayer_pixel[0]; // green pixel
  img_buffer[img_line_step + 2] = img_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

  // Bayer         0 1 2
  //        -1     b g b
  //         0     g R g
  // line_step     b g b
  //rgb_pixel[3] = bayer_pixel[1];
  img_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
  img_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

  // BGBG line
  // Bayer         0 1 2
  //        -1     b g b
  //         0     g r g
  // line_step     B g b
  //rgb_pixel[img_line_step    ] = bayer_pixel[1];
  img_buffer[img_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
  img_buffer[img_line_step + 2] = bayer_pixel[bayer_line_step];

  // Bayer         0 1 2
  //        -1     b g b
  //         0     g r g
  // line_step     b G b
  //rgb_pixel[img_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
  img_buffer[img_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  img_buffer[img_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

  img_buffer += 6;
  bayer_pixel += 2;
  // rest of the last two lines
  for (xIdx = 2; xIdx < width - 2; xIdx += 2, img_buffer += 6, bayer_pixel += 2)
    {
      // GRGR line
      // Bayer       -1 0 1 2
      //        -1    g b g b
      //         0    r G r g
      // line_step    g b g b
      img_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
      img_buffer[1] = bayer_pixel[0];
      img_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

      // Bayer       -1 0 1 2
      //        -1    g b g b
      //         0    r g R g
      // line_step    g b g b
      img_buffer[img_line_step + 3] = img_buffer[3] = bayer_pixel[1];
      img_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
      img_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

      // BGBG line
      // Bayer       -1 0 1 2
      //        -1    g b g b
      //         0    r g r g
      // line_step    g B g b
      img_buffer[img_line_step ] = AVG (bayer_pixel[-1], bayer_pixel[1]);
      img_buffer[img_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
      img_buffer[img_line_step + 2] = bayer_pixel[bayer_line_step];


      // Bayer       -1 0 1 2
      //        -1    g b g b
      //         0    r g r g
      // line_step    g b G b
      //rgb_pixel[img_line_step + 3] = bayer_pixel[1];
      img_buffer[img_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      img_buffer[img_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
    }

  // last two pixel values for first two lines
  // GRGR line
  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r G r
  // line_step    g b g
  img_buffer[img_line_step ] = img_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
  img_buffer[1] = bayer_pixel[0];
  img_buffer[5] = img_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r g R
  // line_step    g b g
  img_buffer[img_line_step + 3] = img_buffer[3] = bayer_pixel[1];
  img_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[-bayer_line_step + 1]);
  //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

  // BGBG line
  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r g r
  // line_step    g B g
  //rgb_pixel[img_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
  img_buffer[img_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
  img_buffer[img_line_step + 5] = img_buffer[img_line_step + 2] = bayer_pixel[bayer_line_step];

  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r g r
  // line_step    g b G
  //rgb_pixel[img_line_step + 3] = bayer_pixel[1];
  img_buffer[img_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  //rgb_pixel[img_line_step + 5] = bayer_pixel[line_step];
      
  return 0;
}

// get the image buffer
int kinect_get_gray_buffer(kinect_t *self, uint8_t *gray_buffer, uint32_t gray_buffer_len, unsigned gray_line_step)
{
  if(gray_buffer_len < (KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS * sizeof(uint8_t)))
    {
      fprintf(stderr, "buffer len too small!\n");
      return -1;
    }

  if(!self->image_on)
    {
      fprintf(stderr, "image stream is not on!\n");
      return -1;
    }
  
  if (self->md_image->XRes () % KINECT_IMAGE_COLS == 0 && self->md_image->YRes () % KINECT_IMAGE_ROWS == 0)
    {
      unsigned src_step = self->md_image->XRes () / KINECT_IMAGE_COLS;
      unsigned src_skip = (self->md_image->YRes () / KINECT_IMAGE_ROWS - 1) * self->md_image->XRes ();
      
      if (gray_line_step == 0)
	gray_line_step = KINECT_IMAGE_COLS;
      
      unsigned dst_skip = gray_line_step - KINECT_IMAGE_COLS; // skip of padding values in bytes
      
      unsigned char* dst_line = gray_buffer;
      const XnRGB24Pixel* src_line = self->md_image->RGB24Data();
    
      for (unsigned yIdx = 0; yIdx < KINECT_IMAGE_ROWS; ++yIdx, src_line += src_skip, dst_line += dst_skip)
	{
	  for (unsigned xIdx = 0; xIdx < KINECT_IMAGE_COLS; ++xIdx, src_line += src_step, dst_line ++)
	    {
	      *dst_line = (unsigned char)(((int)src_line->nRed * 299 + (int)src_line->nGreen * 587 + (int)src_line->nBlue * 114) * 0.001);
	    }
	}
    }
  else
    {
      fprintf(stderr, "buffer len too small!\n");
      return (-1);
    }

  return 0;
}

// get the image buffer -> robust version
int xtion_get_image_buffer(kinect_t *self, unsigned char *img_buffer, unsigned img_line_step, 
			   unsigned width, unsigned height)
{
  // 0  1   2  3
  // u  y1  v  y2
  
  register const XnUInt8* yuv_buffer = self->md_image->Data();
  
  unsigned rgb_line_skip = 0;
  if (img_line_step != 0)
    rgb_line_skip = img_line_step - width * 3;
  

  if (self->md_image->XRes() == width && self->md_image->YRes() == height)
    {
      for( register unsigned yIdx = 0; yIdx < height; ++yIdx, img_buffer += rgb_line_skip )
	{
	  for( register unsigned xIdx = 0; xIdx < width; xIdx += 2, img_buffer += 6, yuv_buffer += 4 )
	    {

	      int v = yuv_buffer[2] - 128;
	      int u = yuv_buffer[0] - 128;
	      
	      img_buffer[0] =  CLIP_CHAR (yuv_buffer[1] + ((v * 18678 + 8192 ) >> 14));
	      img_buffer[1] =  CLIP_CHAR (yuv_buffer[1] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
	      img_buffer[2] =  CLIP_CHAR (yuv_buffer[1] + ((u * 33292 + 8192 ) >> 14));
	      
	      img_buffer[3] =  CLIP_CHAR (yuv_buffer[3] + ((v * 18678 + 8192 ) >> 14));
	      img_buffer[4] =  CLIP_CHAR (yuv_buffer[3] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
	      img_buffer[5] =  CLIP_CHAR (yuv_buffer[3] + ((u * 33292 + 8192 ) >> 14));
	    }
	}
      
    }
  else
    {
      fprintf(stderr, "buffer len too small!\n");
    }
  return 0;
}


// get the gray buffer -> robust version
int xtion_get_gray_buffer(kinect_t *self, unsigned char *gray_buffer, unsigned gray_line_step, 
			  unsigned width, unsigned height)
{
  if (width > self->md_image->XRes () || height > self->md_image->YRes ()){
    fprintf(stderr, "Upsampling not supported. Request was: %d x %d -> %d x %d", self->md_image->XRes (), self->md_image->YRes (), width, height);
    return (-1);
  }
  
  if (self->md_image->XRes () % width != 0 || self->md_image->YRes () % height != 0){
    fprintf (stderr, "Downsampling only possible for integer scales in both dimensions. Request was %d x %d -> %d x %d", 
	     self->md_image->XRes (), self->md_image->YRes (), width, height);
    return (-1);
  }
  
  unsigned gray_line_skip = 0;
  if (gray_line_step != 0)
    gray_line_skip = gray_line_step - width;
  
  register unsigned yuv_step = self->md_image->XRes() / width;
  register unsigned yuv_x_step = yuv_step << 1;
  register unsigned yuv_skip = (self->md_image->YRes() / height - 1) * ( self->md_image->XRes() << 1 );
  register const XnUInt8* yuv_buffer = (self->md_image->Data() + 1);

  for( register unsigned yIdx = 0; yIdx < self->md_image->YRes(); yIdx += yuv_step, yuv_buffer += yuv_skip, gray_buffer += gray_line_skip )
    {
      for( register unsigned xIdx = 0; xIdx < self->md_image->XRes(); xIdx += yuv_step, ++gray_buffer, yuv_buffer += yuv_x_step )
	{
	  *gray_buffer = *yuv_buffer;
	}
    }

  return 0;
}

// get the ir buffer (16-bit image)
int kinect_get_ir_buffer16(kinect_t *self, uint16_t *ir_buffer, uint32_t ir_buffer_len)
{
  if(ir_buffer_len < (KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS * sizeof(uint16_t)))
    {
      fprintf(stderr, "buffer len too small!\n");
      return -1;
    }

  if(!self->ir_on)
    {
      fprintf(stderr, "ir stream is not on!\n");
      return -1;
    }
  
  memcpy(ir_buffer, (char*)self->md_ir->Data(), sizeof(uint16_t) * KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS);
  
  return 0;
}

// get the ir buffer (8-bit image)
int kinect_get_ir_buffer8(kinect_t *self, uint8_t *ir_buffer, uint32_t ir_buffer_len)
{
  if(ir_buffer_len < (KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS * sizeof(uint8_t)))
    {
      fprintf(stderr, "buffer len too small!\n");
      return -1;
    }

  if(!self->ir_on)
    {
      fprintf(stderr, "ir stream is not on!\n");
      return -1;
    }

  uint16_t *ptr = (uint16_t*)self->md_ir->Data();
  uint16_t val, max_val = 0;
  uint8_t uval;
  float fval;

  
  for(int i=0; i<KINECT_IMAGE_ROWS; i++)
    {
      for(int j=0; j<KINECT_IMAGE_COLS; j++)
	{
	  val = ptr[i*KINECT_IMAGE_COLS + j];
	  if(val > max_val)
	    max_val = val;         
	}
  }
  
  for(int i=0; i<KINECT_IMAGE_ROWS; i++)
  {
    for(int j=0; j<KINECT_IMAGE_COLS; j++)
    {
      fval = (float)(ptr[i*KINECT_IMAGE_COLS + j]*1.0)/(max_val*1.0)*255.0;
      uval = (uint8_t)fval;
      ir_buffer[i*KINECT_IMAGE_COLS + j] = uval;
    }
  }


  
  return 0;
}


// get the xyz buffer (with matching color buffer if given)
int kinect_depth_to_xyz_buffer(kinect_t *self,
                               uint16_t *depth_buffer,   uint32_t depth_buffer_len,   // input
                               uint8_t  *color_buffer,   uint32_t color_buffer_len,   // input
                               float    *xyz_buffer,     uint32_t xyz_buffer_len,     // output
                               uint8_t  *xyz_rgb_buffer, uint32_t xyz_rgb_buffer_len) // output
{
  if(!depth_buffer || !xyz_buffer)
  {
    fprintf(stderr, "NULL buffer!\n");
    return -1;
  }
    
  // convert kinect disparity to normalized disparity
  float      depth;
  float      x,y,z;
  int        u, v;
  vec3_t     ir_pt;
  vec3_t     rgb_pt;
  vec3_t     img_pt;
  uint8_t    r, g, b;
  
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  for(int i=0; i<KINECT_IMAGE_ROWS; i++)
  {
    for(int j=0; j<KINECT_IMAGE_COLS; j++)
    {      
      // get the 3d location of the pt first
      if (depth_buffer[i*KINECT_IMAGE_COLS + j] == 0 ||
          depth_buffer[i*KINECT_IMAGE_COLS + j] == self->no_sample_value_ ||
          depth_buffer[i*KINECT_IMAGE_COLS + j] == self->shadow_value_) 
	{
	  ir_pt.x = bad_point;
	  ir_pt.y = bad_point;
	  ir_pt.z = bad_point;        
	}
      else
	{
	  depth = ((float)depth_buffer[i*KINECT_IMAGE_COLS + j])/1000.0;
      
	  // z = baseline * focal_length / disparity
	  // x = image_pt.x * z / focal_length
	  // y = image_pt.y * z / focal_length
	  
	  z = depth;
	  x = 1.0*(j-self->params.ir_camera_center.x) * z / self->params.ir_focal_length;
	  y = 1.0*(i-self->params.ir_camera_center.y) * z / self->params.ir_focal_length;
	  
	  // as seen in ir frame
	  ir_pt.x = x;
	  ir_pt.y = y;
	  ir_pt.z = z;        
	}
      
      xyz_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 0] = ir_pt.x;
      xyz_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 1] = ir_pt.y;
      xyz_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 2] = ir_pt.z;

      
      // if no color buffers provided, continue
      if(!color_buffer)
        continue;
      if(!xyz_rgb_buffer)
        continue;

      // now get the color      
      if(kinect_depth_registration_running(self))
	// assume that depth map is already relative to rgb frame
	rgb_pt =ir_pt;
      else 
	// Instead of using harware registration, do conversion by hand 
	rgb_pt = vec3_transform(self->params.rgb_to_ir, ir_pt);
      
      img_pt.x = self->params.rgb_focal_length * rgb_pt.x / rgb_pt.z + self->params.rgb_camera_center.x;
      img_pt.y = self->params.rgb_focal_length * rgb_pt.y / rgb_pt.z + self->params.rgb_camera_center.y;
      if( (img_pt.x < 0) || (img_pt.x >= KINECT_IMAGE_COLS ) ||
          (img_pt.y < 0) || (img_pt.y >= KINECT_IMAGE_ROWS ) )
      {
        r = 0xFF;
        g = 0xFF;
        b = 0xFF;
      }
      else
      {
        v = (int)img_pt.x;
        u = (int)img_pt.y;
        
        r = color_buffer[u*KINECT_IMAGE_COLS*3  + v*3 + 0];
        g = color_buffer[u*KINECT_IMAGE_COLS*3  + v*3 + 1];
        b = color_buffer[u*KINECT_IMAGE_COLS*3  + v*3 + 2];        
      }
      
      xyz_rgb_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 0] = r;
      xyz_rgb_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 1] = g;
      xyz_rgb_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 2] = b;
      
    }
  }

  
  
  return 0;
}

// get the xyz buffer (with matching color buffer if given)
int kinect_f_depth_to_xyz_buffer(kinect_t *self,
				 float    *depth_buffer,   uint32_t depth_buffer_len,   // input
				 uint8_t  *color_buffer,   uint32_t color_buffer_len,   // input
				 float    *xyz_buffer,     uint32_t xyz_buffer_len,     // output
				 uint8_t  *xyz_rgb_buffer, uint32_t xyz_rgb_buffer_len) // output
{
  if(!depth_buffer || !xyz_buffer)
    {
      fprintf(stderr, "NULL buffer!\n");
      return -1;
    }
  
  // convert kinect disparity to normalized disparity
  float      depth;
  float      x,y,z;
  int        u, v;
  vec3_t     ir_pt;
  vec3_t     rgb_pt;
  vec3_t     img_pt;
  uint8_t    r, g, b;

  // Set transformation from IR 2 RGB camera 
  // In case depth map is already in RGB frame, transformation is identity matrix
  // cv::Mat R = cv::Mat::eye(3,3,CV_32F);
  // cv::Mat T = cv::Mat::zeros(3,KINECT_IMAGE_COLS*KINECT_IMAGE_ROWS,CV_32F);
  if(!kinect_depth_registration_running(self)){
    // In case depth map is in IR frame, transformation is taken from calibration file 
    for(int i=0; i<3; ++i)
      for(int j=0; j<3; ++j)
        kinect::R.at<float>(i,j) = self->params.rgb_to_ir[i][j];


    
    for(int i=0; i<KINECT_IMAGE_COLS*KINECT_IMAGE_ROWS; ++i)
      for(int j=0; j<3; ++j){
        kinect::T.at<float>(j,i) = self->params.rgb_to_ir[j][3];
      }
  }
  
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

//  std::vector<cv::Point3f> points;
//  points.resize(KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS);
  
  for(int i=0; i<KINECT_IMAGE_ROWS; i++)
    {
      for(int j=0; j<KINECT_IMAGE_COLS; j++)
	{      
	  // get the 3d location of the pt first
	  if (depth_buffer[i*KINECT_IMAGE_COLS + j] == 0 ||
	      depth_buffer[i*KINECT_IMAGE_COLS + j] == self->no_sample_value_ ||
	      depth_buffer[i*KINECT_IMAGE_COLS + j] == self->shadow_value_ ||
	      depth_buffer[i*KINECT_IMAGE_COLS + j] != depth_buffer[i*KINECT_IMAGE_COLS + j]) 
	    {
	      ir_pt.x = bad_point;
	      ir_pt.y = bad_point;
	      ir_pt.z = bad_point;        
	    }
	  else
	    {
	      // from millimeters to meters
	      depth = depth_buffer[i*KINECT_IMAGE_COLS + j]*0.001;
	      
	      // z = baseline * focal_length / disparity
	      // x = image_pt.x * z / focal_length
	      // y = image_pt.y * z / focal_length
	      
	      z = depth;
	      x = 1.0*(j-self->params.ir_camera_center.x) * z / self->params.ir_focal_length;
	      y = 1.0*(i-self->params.ir_camera_center.y) * z / self->params.ir_focal_length;
	  
	      /*
	      std::cout << "ir focal length " << self->params.ir_focal_length << std::endl;
	      
	      std::cout << "depth " << z << " x " << x << " y " << y << std::endl;
	      */

	      // as seen in ir frame
	      ir_pt.x = x;
	      ir_pt.y = y;
	      ir_pt.z = z;        
	    }
      
	  xyz_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 0] = ir_pt.x;
	  xyz_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 1] = ir_pt.y;
	  xyz_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 2] = ir_pt.z;

      
	  // if no color buffers provided, continue
	  if(!color_buffer)
	    continue;
	  if(!xyz_rgb_buffer)
	    continue;

	  
	  cv::Point3f pt(ir_pt.x, ir_pt.y, ir_pt.z);
	  kinect::points.at(i*KINECT_IMAGE_COLS+j) = pt;
	  
	}
    }
  
  if(!color_buffer)
    return 0;
  if(!xyz_rgb_buffer)
    return 0;
  
  // now get the color      
  cv::Mat point_mat = cv::Mat(kinect::points).reshape(1).t();
  cv::Mat rgb_mat = kinect::R * point_mat;
  rgb_mat+=kinect::T;
  
  for(int i=0; i<KINECT_IMAGE_ROWS; i++)
    {
      for(int j=0; j<KINECT_IMAGE_COLS; j++)
	{   
	  
	  rgb_pt.x = rgb_mat.at<float>(0, i*KINECT_IMAGE_COLS+j);
	  rgb_pt.y = rgb_mat.at<float>(1, i*KINECT_IMAGE_COLS+j);
	  rgb_pt.z = rgb_mat.at<float>(2, i*KINECT_IMAGE_COLS+j);
	  
	  img_pt.x = self->params.rgb_focal_length * rgb_pt.x / rgb_pt.z + self->params.rgb_camera_center.x;
	  img_pt.y = self->params.rgb_focal_length * rgb_pt.y / rgb_pt.z + self->params.rgb_camera_center.y;

	  if( (img_pt.x < 0) || (img_pt.x >= KINECT_IMAGE_COLS ) ||
	      (img_pt.y < 0) || (img_pt.y >= KINECT_IMAGE_ROWS ) ||
	      img_pt.x != img_pt.x || img_pt.y != img_pt.y)
	    {
	      r = 0xFF;
	      g = 0xFF;
	      b = 0xFF;
	    }
	  else
	    {

	      v = (int)img_pt.x;
	      u = (int)img_pt.y;
	      
	      r = color_buffer[u*KINECT_IMAGE_COLS*3  + v*3 + 0];
	      g = color_buffer[u*KINECT_IMAGE_COLS*3  + v*3 + 1];
	      b = color_buffer[u*KINECT_IMAGE_COLS*3  + v*3 + 2];        

	    }
	  
	  xyz_rgb_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 0] = r;
	  xyz_rgb_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 1] = g;
	  xyz_rgb_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 2] = b;
	  
	}
    }

  return 0;
}


bool kinect_depth_stream_running(kinect_t *self)
{
  return ( self->g_depth->IsValid () && self->g_depth->IsGenerating ());
}


bool kinect_image_stream_running(kinect_t *self)
{
  return ( self->g_image->IsValid () && self->g_image->IsGenerating ());
}

bool kinect_depth_registration_running(kinect_t *self)
{
  return (self->g_depth->GetAlternativeViewPointCap ().IsViewPointAs (*(self->g_image)));
}
