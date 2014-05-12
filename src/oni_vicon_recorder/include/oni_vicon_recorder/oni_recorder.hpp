/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Max-Planck-Institute for Intelligent Systems,
 *                     University of Southern California,
 *                     Karlsruhe Institute of Technology
 *    Jan Issac (jan.issac@gmail.com)
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

/**
 * @date 04/14/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */

#ifndef ONI_VICON_RECORDER_ONI_RECORDER_HPP
#define ONI_VICON_RECORDER_ONI_RECORDER_HPP

#include <string>
#include <map>
#include <signal.h>

#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>

#include <oni_vicon_recorder/RunDepthSensorAction.h>
#include <oni_vicon_recorder/ChangeDepthSensorModeAction.h>

#include <rgbd_sensor/kinect.h>
#include <rgbd_sensor/rgbd_sensor.h>

#include "oni_vicon_recorder/frame_time_tracker.hpp"

class OniRecorder
{
public:
    OniRecorder(ros::NodeHandle& node_handle,
                FrameTimeTracker::Ptr frame_time_tracker);
    ~OniRecorder();

    void changeDeptSensorModeCB(const oni_vicon_recorder::ChangeDepthSensorModeGoalConstPtr &goal);
    void runDepthSensorCB(const oni_vicon_recorder::RunDepthSensorGoalConstPtr& goal);
    bool closeDevice();

    bool startRecording(std::string file);
    bool stopRecording();
    u_int64_t countFrames() const;
    bool isRecording() const;

    std::map<std::string, XnMapOutputMode> getSupportedModes(const DepthGenerator *generator) const;
    std::vector<std::string> getSupportedModeList(
            const std::map<std::string, XnMapOutputMode>& mode_map) const;
    XnMapOutputMode getCurrentMode(const DepthGenerator *generator) const;
    std::string getModeName(const XnMapOutputMode& mode) const;

private:
    ros::NodeHandle node_handle_;
    RgbdCapture rgbd_capture_;
    Recorder recorder_;
    bool recording_;
    bool running_;
    u_int64_t frames_;
    boost::shared_mutex frameLock_;
    std::map<std::string, XnMapOutputMode> modes_;

    actionlib::SimpleActionServer<
        oni_vicon_recorder::RunDepthSensorAction> run_depth_sensor_as_;
    actionlib::SimpleActionServer<
        oni_vicon_recorder::ChangeDepthSensorModeAction> change_depth_sensor_mode_as_;

    FrameTimeTracker::Ptr frame_time_tracker_;
};

#endif
