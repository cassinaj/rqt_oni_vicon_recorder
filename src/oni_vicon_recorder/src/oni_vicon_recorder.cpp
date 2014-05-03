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

#include "oni_vicon_recorder/oni_vicon_recorder.hpp"

#include <boost/filesystem.hpp>

using namespace oni_vicon_recorder;
using namespace depth_sensor_vicon_calibration;

OniViconRecorder::OniViconRecorder(ros::NodeHandle& nh,
                                   std::string record_as_name,
                                   FrameTimeTracker::Ptr frame_time_tracker,
                                   OniRecorder& oni_recorder,
                                   ViconRecorder& vicon_recorder,
                                   Calibration& global_calibration):
    frame_time_tracker_(frame_time_tracker),
    oni_recorder_(oni_recorder),
    vicon_recorder_(vicon_recorder),
    global_calibration_(global_calibration),
    record_as_(nh, record_as_name, boost::bind(&OniViconRecorder::recordCB, this, _1), false)
{
}

OniViconRecorder::~OniViconRecorder()
{

}

void OniViconRecorder::run()
{
    record_as_.start();

    ROS_INFO("OniViconRecorder is running ");
    ros::spin();
    ROS_INFO("Shutting down OniViconRecorder");
}

void OniViconRecorder::recordCB(const RecordGoalConstPtr& goal)
{
    ROS_INFO("Start ONI Vicon recording");
    ROS_INFO(" - Recording Name: %s", goal->name.c_str());
    ROS_INFO(" - Recording Destination %s/%s", goal->destination.c_str(), goal->name.c_str());

    RecordFeedback feedback;
    RecordResult result;

    result.vicon_frames = 0;
    result.kinect_frames = 0;

    // create directory before recording
    boost::filesystem::path destination_dir(goal->destination);
    destination_dir /= goal->name;
    if(!boost::filesystem::create_directory(destination_dir))
    {
        ROS_ERROR("ONI Vicon recording aborted. Could not create destination directory %s",
                  destination_dir.c_str());
        record_as_.setAborted(result);
        return;
    }

    frame_time_tracker_->reset();

    // start ONI recording
    boost::filesystem::path oni_file = destination_dir / (goal->name + ".oni");   
    if(!oni_recorder_.startRecording(oni_file.string()))
    {
        ROS_WARN("ONI Vicon recording aborted.");
        record_as_.setAborted(result);
        return;
    }

    // start Vicon data stream recording
    boost::filesystem::path vicon_file = destination_dir / (goal->name + ".txt");
    if(!vicon_recorder_.startRecording(vicon_file.string(), goal->object_name))
    {
        oni_recorder_.stopRecording();
        ROS_WARN("ONI Vicon recording aborted.");
        record_as_.setAborted(result);
        return;
    }

    feedback.duration = 0;
    feedback.vicon_frames = 0;
    feedback.kinect_frames = 0;

    // report feedback 30 times a second until stopped
    ros::Rate r(30);
    while (true)
    {
        if (record_as_.isPreemptRequested())
        {
            break;
        }
        else if (!oni_recorder_.isRecording()
                 || !vicon_recorder_.isRecording()
                 || !ros::ok())
        {
            ROS_WARN("ONI Vicon recording aborted");
            result.vicon_frames = vicon_recorder_.countFrames();
            result.kinect_frames = oni_recorder_.countFrames();
            record_as_.setAborted(result);
            oni_recorder_.stopRecording();
            vicon_recorder_.stopRecording();
            return;
        }

        feedback.duration = frame_time_tracker_->timeInMilliseconds();
        feedback.vicon_frames = vicon_recorder_.countFrames();
        feedback.kinect_frames = oni_recorder_.countFrames();
        record_as_.publishFeedback(feedback);

        r.sleep();
    }

    ROS_INFO("Stopping ONI Vicon recording");
    vicon_recorder_.stopRecording();
    oni_recorder_.stopRecording();

    result.vicon_frames = vicon_recorder_.countFrames();;
    result.kinect_frames = oni_recorder_.countFrames();
    record_as_.setSucceeded(result);    
}
