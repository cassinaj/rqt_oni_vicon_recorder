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
 * @date 05/03/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Karlsruhe Institute of Technology (KIT), University of Southern California (USC)
 */

#include <sys/time.h>
#include <unistd.h>

#include "oni_vicon_recorder/frame_time_tracker.hpp"

FrameTimeTracker::FrameTimeTracker():
    starting_time_(0.),
    vicon_frame_(0),
    depth_sensor_frame_(0)
{
}

FrameTimeTracker::~FrameTimeTracker()
{
}

void FrameTimeTracker::reset()
{
    vicon_frame_ = 0;
    depth_sensor_frame_ = 0;

    starting_time_ = timeInMilliseconds();
    vicon_frame_time_ = 0.;
    depth_senso_frame_time_ = 0.;
}

unsigned long FrameTimeTracker::viconFrame()
{
    return vicon_frame_ ;
}

unsigned long FrameTimeTracker::depthSensorFrame()
{
    return depth_sensor_frame_;
}

u_int64_t FrameTimeTracker::viconFrameTime()
{
    return vicon_frame_time_;
}

u_int64_t FrameTimeTracker::depthSensorFrameTime()
{
    return depth_senso_frame_time_;
}

void FrameTimeTracker::viconFrame(unsigned long vicon_frame)
{\
    vicon_frame_ = vicon_frame;
    vicon_frame_time_ = timeInMilliseconds();
}

void FrameTimeTracker::depthSensorFrame(unsigned long depth_sensor_frame)
{
    depth_sensor_frame_ = depth_sensor_frame;
    depth_senso_frame_time_ = timeInMilliseconds();
}

u_int64_t FrameTimeTracker::timeInSeconds()
{
    return timeInMilliseconds() / 1000;
}

u_int64_t FrameTimeTracker::timeInMicroseconds()
{
    return timeInMilliseconds() * 1000;
}

u_int64_t FrameTimeTracker::timeInMilliseconds()
{
    timeval current_time;
    gettimeofday(&current_time, NULL);

    return (current_time.tv_sec * 1000  + current_time.tv_usec / 1000) - starting_time_;
}
