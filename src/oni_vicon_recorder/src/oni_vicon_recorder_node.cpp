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
 * Karlsruhe Institute of Technology (KIT), University of Southern California (USC)
 */

#include <string>
#include <ros/ros.h>

#include <depth_sensor_vicon_calibration/depth_sensor_vicon_calibration.hpp>

#include "oni_vicon_recorder/kinect.h"
#include "oni_vicon_recorder/frame_time_tracker.hpp"
#include "oni_vicon_recorder/oni_recorder.hpp"
#include "oni_vicon_recorder/vicon_recorder.hpp"
#include "oni_vicon_recorder/oni_vicon_recorder.hpp"
#include "oni_vicon_recorder/namespaces.hpp"


using namespace depth_sensor_vicon_calibration;
using namespace oni_vicon_recorder;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oni_vicon_recorder");
    ros::NodeHandle nh;

    /* Parameters */

    // calibration parameters
    int global_calib_iterations;
    std::string global_calib_object_name;
    std::string global_calib_object;
    std::string global_calib_object_display;
    nh.param("/global_calibration/iterations", global_calib_iterations, 100);
    nh.param("/global_calibration/object_name", global_calib_object_name, std::string("calib_ob"));
    nh.param("/global_calibration/object", global_calib_object, std::string("package://depth_sensor_vicon_calibration/object/calib_obj_downsampled.obj"));
    nh.param("/global_calibration/object_display", global_calib_object_display, std::string("package://depth_sensor_vicon_calibration/object/calib_obj.obj"));

    // OniRecorder parameters

    // OniViconRecorder parameters

    // ViconRecorder parameters

    /* build dependencies and inject them */
    FrameTimeTracker::Ptr frame_time_tracker(new FrameTimeTracker());

    OniRecorder oni_recorder(nh,
                             frame_time_tracker,
                             ACTION_NS_RUN_DEPTH_SENSOR,
                             ACTION_NS_CHANGE_DEPTH_SENSOR_MODE);

    ViconRecorder vicon_recorder(nh,
                                 frame_time_tracker,
                                 SERVICE_NS_VICON_OBJECTS,
                                 SERVICE_NS_VERIFY_OBJECT_EXISTS,
                                 SERVICE_NS_VICON_OBJECT_POSE,
                                 ACTION_NS_CONNECT_TO_VICON);

    Calibration calibration(nh,
                            global_calib_iterations,
                            global_calib_object_name,
                            global_calib_object,
                            global_calib_object_display,
                            ACTION_NS_GLOBAL_CALIBRATION,
                            ACTION_NS_CONTINUE_GLOBAL_CALIBRATION,
                            SERVICE_NS_VICON_OBJECT_POSE);

    OniViconRecorder oni_vicon_recorder(nh,
                                        ACTION_NS_RECORD,
                                        frame_time_tracker,
                                        oni_recorder,
                                        vicon_recorder,
                                        calibration);
    oni_vicon_recorder.run();
    return 0;
}
