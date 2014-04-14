/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012 Max-Planck-Institute for Intelligent Systems
 *    Jeannette Bohg (jbohg@tuebingen.mpg.de)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 *  arm_rgbd_node.cpp
 *  ROS_Wrapper around slim rgbd driver. Based on 
 *  Willow Garage openni_nodelet and Jon Kelley's honeybee node.
 *
 *  Created on: Jul 7, 2012
 */

#ifndef ARM_RGBD_NODE_H_
#define ARM_RGBD_NODE_H_

#include <signal.h>
#include <boost/format.hpp>

// ROS
#include <ros/ros.h>
#include <arm_rgbd/kinect.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

// Local
//#include "dev_bumblebee2.h"
//#include "arm_honeybee/honeybeeConfig.h"

/** @file
 *
 *  @brief ARM driver node for RGB-D cameras (ASUS Xtion and MS Kinect).
 *
 *  This node provides a driver for the ASUS Xtion and MS KInect cameras.  The node relies on OpenNI.
 *
 *  @par Advertises
 *
 *   - @b arm_rgbd/rgb/image (sensor_msgs/Image) RGB Image.
 * 
 *   - @b arm_rgbd/gray/image (sensor_msgs/Image) Grayscale Image.
 *
 *   - @b arm_rgbd/depth/image (sensor_msgs/Image) Depth Image.
 *
 *   - @b arm_rgbd/disparity/image (sensor_msgs/Image) Disparity Image.
 *
 *   - @b arm_rgbd/rgb/camera_info (sensor_msgs/CameraInfo) Calibration
 *     information for rgb camera.
 *
 *   - @b arm_rgbd/depth/camera_info (sensor_msgs/CameraInfo) Calibration
 *     information for rgb camera. 
 *
 *   - @b arm_rgbd/depth/points (sensor_msgs/PointCloud2) Point cloud
 *
 *   - @b arm_rgbd/rgb/points (sensor_msgs/PointCloud2) Point cloud with RGB info
 */

namespace enc = sensor_msgs::image_encodings;

// Prototypes
void sigsegv_handler(int sig);

class RGBDNode
{
private:

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  
  ros::NodeHandle priv_nh_;
  image_transport::ImageTransport* it_;

  std::string camera_name_;
  std::string frame_id_;

  sensor_msgs::Image rgb_image_;
  sensor_msgs::Image gray_image_;

  sensor_msgs::CameraInfo rgb_cam_info_;
  sensor_msgs::CameraInfo depth_cam_info_;

  std::string calib_url_;

  // Standard parameters
  bool bus_reset_;
  std::string bayer_pattern_;
  std::string encoding_;
  std::string guid_;
  int iso_speed_;
  std::string video_mode_;

  // helper methods
  inline bool isImageStreamRequired() const;
  inline bool isDepthStreamRequired() const;
  sensor_msgs::CameraInfoPtr fillCameraInfo (ros::Time time, bool is_rgb);

  void subscriberChangedEvent ();

  bool calibration_valid_;
  bool calibration_loaded_;

  bool initDevice();
  void initParameters();

  // Approximate synchronization for XYZRGB point clouds.
  boost::shared_ptr<Synchronizer> depth_rgb_sync_;
  
  // Publishers Camera Info
  ros::Publisher pub_rgb_info_;
  ros::Publisher pub_depth_info_;
  // Publishers Images
  image_transport::Publisher pub_rgb_image_;
  image_transport::Publisher pub_gray_image_; 
  image_transport::Publisher pub_depth_image_;
  // Publishers Point Clouds
  ros::Publisher pub_disp_image_;
  ros::Publisher pub_point_cloud_;
  ros::Publisher pub_point_cloud_rgb_;

  // publish methods
  void publishRgbImage (ros::Time time);
  void publishGrayImage (ros::Time time);
  void publishDepthImage (ros::Time time);
  void publishDisparity (ros::Time time);
  void publishXYZRGBPointCloud (ros::Time time);
  void publishXYZPointCloud (ros::Time time);

  /** \brief the actual rgbd device*/
  kinect_t* device_;

  std::string rgb_frame_id_;
  std::string depth_frame_id_;
  unsigned image_width_;
  unsigned image_height_;
  unsigned depth_width_;
  unsigned depth_height_;

public:

  RGBDNode();
  ~RGBDNode();
  
  void closeCamera();
  
  void spin();
  
};

bool RGBDNode::isImageStreamRequired() const
{
  return (pub_rgb_image_.getNumSubscribers()       > 0 ||
	  pub_gray_image_.getNumSubscribers()      > 0 ||
	  pub_point_cloud_rgb_.getNumSubscribers() > 0 );
}

bool RGBDNode::isDepthStreamRequired() const
{
  return (pub_depth_image_.getNumSubscribers()     > 0 ||
	  pub_disp_image_.getNumSubscribers()       > 0 ||
	  pub_point_cloud_.getNumSubscribers()     > 0 ||
	  pub_point_cloud_rgb_.getNumSubscribers() > 0 );
}

#endif //ARM_RGBD_NODE_H_
