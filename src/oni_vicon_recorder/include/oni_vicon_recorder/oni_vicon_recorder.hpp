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

#ifndef ONI_VICON_RECORDER_ONI_VICON_RECORDER_HPP
#define ONI_VICON_RECORDER_ONI_VICON_RECORDER_HPP

#include <string>
#include <cstdlib>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <oni_vicon_recorder/RecordAction.h>

#include <oni_vicon_calibration/calibrator.hpp>

#include "oni_vicon_recorder/oni_recorder.hpp"
#include "oni_vicon_recorder/vicon_recorder.hpp"
#include "oni_vicon_recorder/frame_time_tracker.hpp"

namespace oni_vicon_recorder
{
    /**
     * The OniViconRecorder implements an Action to start recording of depth images sensor
     * (Kinect/XTION) device and Vicon stream data at the same time. It uses a FrameTimeTracker
     * to to keep track of correspoding frames between these two systems including their time of
     * capture in milliseconds. Note however, the timestamps are provided merely a reference
     * provided by the machine running this process.
     *
     * OniViconRecorder uses constructor dependency injection taking instances of
     *  - oni_vicon_recorder::FrameTimeTracker
     *  - oni_vicon_recorder::OniRecorder
     *  - oni_vicon_recorder::ViconRecorder
     *  - oni_vicon_calibration::Calibration
     */
    class OniViconRecorder
    {
    public:
        OniViconRecorder(ros::NodeHandle& node_handle,
                         FrameTimeTracker::Ptr frame_time_tracker,
                         OniRecorder& oni_recorder,
                         ViconRecorder& vicon_recorder,
                         oni_vicon_calibration::Calibrator& global_calibration);
        virtual ~OniViconRecorder();
        void run();
        void recordCB(const oni_vicon_recorder::RecordGoalConstPtr& goal);

    private:
        FrameTimeTracker::Ptr frame_time_tracker_;
        OniRecorder& oni_recorder_;
        ViconRecorder& vicon_recorder_;
        oni_vicon_calibration::Calibrator& global_calibration_;

        actionlib::SimpleActionServer<oni_vicon_recorder::RecordAction> record_as_;
    };
}

#endif

