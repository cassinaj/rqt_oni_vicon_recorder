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

#include <string>
#include <ros/ros.h>

#include <depth_sensor_vicon_calibration/calibration.hpp>

#include "oni_vicon_recorder/frame_time_tracker.hpp"
#include "oni_vicon_recorder/oni_recorder.hpp"
#include "oni_vicon_recorder/vicon_recorder.hpp"
#include "oni_vicon_recorder/oni_vicon_recorder.hpp"

using namespace depth_sensor_vicon_calibration;
using namespace oni_vicon_recorder;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oni_vicon_recorder");
    ros::NodeHandle nh;

    /* Parameters */

    // calibration parameters with defaults
    int global_calib_iterations = 100;
    int local_calib_iterations = 100;
    std::string path = "package://depth_sensor_vicon_calibration/object";
    std::string calib_object_name = "calib_ob";
    std::string calib_object = path + "/calib_obj_downsampled.obj";
    std::string calib_object_display = path + "/calib_obj.obj";
    std::string camera_info_topic = "/XTION/depth/camera_info";

    // load set parameters otherwise maintain defautls
    nh.param("/global_calibration/iterations", global_calib_iterations, global_calib_iterations);
    nh.param("/local_calibration/iterations", local_calib_iterations, local_calib_iterations);
    nh.param("/global_calibration/object_name", calib_object_name, calib_object_name);
    nh.param("/global_calibration/object", calib_object, calib_object);
    nh.param("/global_calibration/object_display", calib_object_display, calib_object_display);
    nh.param("/global_calibration/camera_info_topic", camera_info_topic, camera_info_topic);

    /* build dependencies and inject them */
    FrameTimeTracker::Ptr frame_time_tracker(new FrameTimeTracker());

    OniRecorder oni_recorder(nh, frame_time_tracker);

    ViconRecorder vicon_recorder(nh, frame_time_tracker);

    Calibration calibration(nh,
                            global_calib_iterations,
                            local_calib_iterations,
                            calib_object_name,
                            calib_object,
                            calib_object_display,
                            camera_info_topic);

    OniViconRecorder oni_vicon_recorder(nh,
                                        frame_time_tracker,
                                        oni_recorder,
                                        vicon_recorder,
                                        calibration);
    oni_vicon_recorder.run();

    return 0;
}
