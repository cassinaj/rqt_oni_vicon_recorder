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

#ifndef ONI_VICON_RECORDER_FRAME_TIME_TRACKER_HPP
#define ONI_VICON_RECORDER_FRAME_TIME_TRACKER_HPP

#include <boost/shared_ptr.hpp>

class FrameTimeTracker
{
public:
    typedef boost::shared_ptr<FrameTimeTracker> Ptr;

public:
    FrameTimeTracker();
    virtual ~FrameTimeTracker();

    void reset();

    long unsigned int viconFrame();
    long unsigned int depthSensorFrame();

    u_int64_t viconFrameTime();
    u_int64_t depthSensorFrameTime();

    void viconFrame(long unsigned int vicon_frame);
    void depthSensorFrame(long unsigned int depth_sensor_frame);

    u_int64_t timeInSeconds();
    u_int64_t timeInMilliseconds();
    u_int64_t timeInMicroseconds();

private:
    u_int64_t starting_time_;
    u_int64_t vicon_frame_time_;
    u_int64_t depth_senso_frame_time_;
    long unsigned int vicon_frame_;
    long unsigned int depth_sensor_frame_;
};

#endif

