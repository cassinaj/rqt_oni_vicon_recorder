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

#include "oni_vicon_recorder/vicon_recorder.hpp"

#include <algorithm>
#include <iterator>

using namespace oni_vicon_recorder;
using namespace ViconDataStreamSDK::CPP;

ViconRecorder::ViconRecorder(ros::NodeHandle& node_handle,
                             FrameTimeTracker::Ptr frame_time_tracker,
                             std::string vicon_objects_srv_name,
                             std::string object_verification_srv_name,
                             std::string vicon_object_pose_srv_name,
                             std::string connect_to_vicon_as_name,
                             int float_precision):
    float_precision_(float_precision),
    connected_(false),
    recording_(false),
    frames_(0),
    hostname_("localhost:801"),
    multicast_address_("244.0.0.0:44801"),
    connect_to_multicast_(false),
    multicast_enabled_(false),
    connect_to_vicon_as_(node_handle,
                         connect_to_vicon_as_name,
                         boost::bind(&ViconRecorder::connectCB, this, _1),
                         false),
    frame_time_tracker_(frame_time_tracker)
{
    connect_to_vicon_as_.start();

    vicon_objects_srv_ = node_handle.advertiseService(vicon_objects_srv_name,
                                                      &ViconRecorder::viconObjectsCB,
                                                      this);

    object_verification_srv_ = node_handle.advertiseService(object_verification_srv_name,
                                                            &ViconRecorder::objectExistsCB,
                                                            this);

    vicon_object_pose_srv_ = node_handle.advertiseService(vicon_object_pose_srv_name,
                                                          &ViconRecorder::viconObjectPose,
                                                          this);
}

ViconRecorder::~ViconRecorder()
{

}

bool ViconRecorder::startRecording(const std::string& file, const std::string& object_name)
{
    boost::upgrade_lock<boost::shared_mutex> lock(iteration_mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);

    if (!connected_)
    {
        ROS_WARN("Cannot start recording. Not connected to Vicon system.");
        return false;
    }

    if(!file.empty())
    {
        ofs_.open(file.c_str());
        if(!ofs_.is_open())
        {
            ROS_WARN("Could not create or open file %s", file.c_str());
            return false;
        }
    }

    object_name_ = object_name;

    recording_ = true;
    frames_ = 0;

    ROS_INFO("Vicon data recording started");

    return true;
}

bool ViconRecorder::stopRecording()
{
    boost::upgrade_lock<boost::shared_mutex> lock(iteration_mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);

    if(ofs_.is_open())
    {
        ofs_.close();
    }

    if (!recording_)
    {
        ROS_WARN("No Vicon data recording to stop.");
        return false;
    }

    recording_ = false;
}

u_int64_t ViconRecorder::countFrames()
{
    // boost::upgrade_lock<boost::shared_mutex> lock(iteration_mutex_);
    // boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);

    return frames_;
}

bool ViconRecorder::isRecording()
{
    // boost::upgrade_lock<boost::shared_mutex> lock(iteration_mutex_);
    // boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);

    return recording_;
}

void ViconRecorder::closeConnection()
{
    boost::shared_lock<boost::shared_mutex> lock(iteration_mutex_);

    // disconnect from vicon
    if(multicast_enabled_)
    {
        vicon_client_.StopTransmittingMulticast();
    }

    vicon_client_.DisableSegmentData();
    vicon_client_.DisableMarkerData();
    vicon_client_.DisableUnlabeledMarkerData();
    vicon_client_.DisableDeviceData();
    vicon_client_.Disconnect();

    connected_ = false;
    recording_ = false;
}

void ViconRecorder::connectCB(const ConnectToViconGoalConstPtr& goal)
{
    ConnectToViconFeedback feedback;
    ConnectToViconResult result;

    ROS_INFO("Connecting to Vicon");

    if (vicon_client_.IsConnected().Connected)
    {
        ROS_INFO("Already connected to Vicon System.");
        connect_to_vicon_as_.setAborted(result);
        return;
    }

    hostname_ = goal->host;
    multicast_enabled_ = goal->enable_multicast;
    multicast_address_ = goal->multicast_address;
    connect_to_multicast_ = goal->connect_to_multicast;
    int retries = goal->retry;

    // connect to vicon
    ros::Duration wait_time(1.);
    while (retries >= 0 && !vicon_client_.IsConnected().Connected)
    {
        if(connect_to_multicast_)
        {
            ROS_INFO("Connecting to Vicon %s, multicast %s ... ",
                     hostname_.c_str(), goal->multicast_address.c_str());
            connected_ = (vicon_client_.ConnectToMulticast(
                      hostname_, multicast_address_).Result == Result::Success);

        }
        else
        {
            ROS_INFO("Connecting to Vicon %s ... ", hostname_.c_str());
            connected_ = (vicon_client_.Connect( hostname_ ).Result == Result::Success);
        }

        if (connected_)
        {
            break;
        }

        retries--;

        if (retries >= 0)
        {
            ROS_INFO("No connecting to Vicon system. Retry %d ...", (goal->retry - retries));
        }

        if (connect_to_vicon_as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Connecting to Vicon System aborted.");
            connect_to_vicon_as_.setAborted(result);
            return;
        }

        wait_time.sleep();
    }

    result.connected = connected_;
    feedback.connected = connected_;
    connect_to_vicon_as_.publishFeedback(feedback);

    if (!connected_)
    {
        connect_to_vicon_as_.setAborted(result);
        ROS_INFO("Connecting to Vicon System failed.");
        return;
    }

    // Enable some different data types
    vicon_client_.EnableSegmentData();
    vicon_client_.EnableMarkerData();
    vicon_client_.EnableUnlabeledMarkerData();
    vicon_client_.EnableDeviceData();
    vicon_client_.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);
    vicon_client_.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up); // Z-up

    if(multicast_enabled_)
    {
        vicon_client_.StartTransmittingMulticast(hostname_, multicast_address_);
    }

    ROS_INFO("Connected to Vicon System.");

    ros::Rate check_rate(1000);
    while (!connect_to_vicon_as_.isPreemptRequested() && ros::ok())
    {
        boost::shared_lock<boost::shared_mutex> lock(iteration_mutex_);

        if (recording_ && vicon_client_.GetFrame().Result == Result::Success)
        {
            if (!recordFrame())
            {
                connected_ = false;
                connect_to_vicon_as_.setAborted(result);
                closeConnection();
                ROS_ERROR("Recording vicon data frame failed. CLosing connection.");
                return;
            }
        }
        else
        {
            check_rate.sleep();
        }
    }

    closeConnection();
    ROS_INFO("Vicon connection closed.");
    connect_to_vicon_as_.setSucceeded(result);
}

bool ViconRecorder::viconObjectsCB(ViconObjects::Request& request, ViconObjects::Response& response)
{
    std::set<std::string> objects = getViconObjects();
    std::copy(objects.begin(), objects.end(), std::back_inserter(response.object_names));

    ROS_INFO("Vicon objects requested. Found %lu objects.", objects.size());

    return true;
}

bool ViconRecorder::objectExistsCB(VerifyObjectExists::Request& request,
                                   VerifyObjectExists::Response& response)
{
    std::set<std::string> objects = getViconObjects();
    response.exists = objects.find(request.object_name) != objects.end();

    return true;
}

bool ViconRecorder::viconObjectPose(oni_vicon_recorder::ViconObjectPose::Request& request,
                                    oni_vicon_recorder::ViconObjectPose::Response& response)
{
    // ROS_INFO_STREAM("Requesting object pose of " << request.object_name);

    if (!connected_ || !waitForFrame())
    {
        ROS_INFO("Not connected to Vicon or acquiring a frame failed!");
        return false;
    }

    bool objectExists = false;
    unsigned int SubjectCount = vicon_client_.GetSubjectCount().SubjectCount;
    for(unsigned int i = 0 ; i < SubjectCount ; ++i)
    {
        // Get the subject name
        std::string object_name = vicon_client_.GetSubjectName(i).SubjectName;
        if (object_name.compare(request.object_name) != 0)
        {
            continue;
        }

        // Count the number of segments
        unsigned int segment_count = vicon_client_.GetSegmentCount(object_name).SegmentCount;
        for(unsigned int j = 0 ; j < segment_count ; ++j)
        {
            // Get the segment name
            std::string SegmentName = vicon_client_.GetSegmentName(object_name, j).SegmentName;
            if (SegmentName.compare(request.object_name) != 0)
            {
                continue;
            }
            objectExists = true;

            // Get the global segment translation
            Output_GetSegmentGlobalTranslation globalTranslation =
                    vicon_client_.GetSegmentGlobalTranslation( object_name, SegmentName );
            response.object_pose.position.x = globalTranslation.Translation[0] / 1000.;
            response.object_pose.position.y = globalTranslation.Translation[1] / 1000.;
            response.object_pose.position.z = globalTranslation.Translation[2] / 1000.;

            // Get the global segment rotation in quaternion co-ordinates
            Output_GetSegmentGlobalRotationQuaternion globalRotationQuaternion =
                    vicon_client_.GetSegmentGlobalRotationQuaternion( object_name, SegmentName );

            response.object_pose.orientation.w = globalRotationQuaternion.Rotation[3];
            response.object_pose.orientation.x = globalRotationQuaternion.Rotation[0];
            response.object_pose.orientation.y = globalRotationQuaternion.Rotation[1];
            response.object_pose.orientation.z = globalRotationQuaternion.Rotation[2];

            break;
        }
    }

    // ROS_INFO_STREAM_COND(!objectExists, "Object " << request.object_name << " does not exist");
    // ROS_INFO_STREAM_COND(objectExists, "Object " << request.object_name << " found");

    return objectExists;
}

bool ViconRecorder::waitForFrame(double wait_time_in_sec)
{
    // acquire a frame
    ros::Rate check_rate(100);
    int wait_time = wait_time_in_sec * 100;
    while(vicon_client_.GetFrame().Result != Result::Success && wait_time > 0)
    {
        if (--wait_time == 0)
        {
            //time out and no frame has been acquired
            return false;
        }

        check_rate.sleep();
    }

    return true;
}

std::set<std::string> ViconRecorder::getViconObjects()
{
    std::set<std::string> objects;

    boost::upgrade_lock<boost::shared_mutex> lock(iteration_mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);

    if(connected_ && waitForFrame())
    {
        // acquire a frame
        int wait_time = 1; // sec
        ros::Rate check_rate(100);
        wait_time *= 100;
        while(vicon_client_.GetFrame().Result != Result::Success && wait_time > 0)
        {
            if (--wait_time == 0)
            {
                //time out and no frame has been acquired
                return objects;
            }

            check_rate.sleep();
        }
        unsigned int subject_count = vicon_client_.GetSubjectCount().SubjectCount;

        for(unsigned int i = 0; i < subject_count; ++i)
        {
            std::string subject_name = vicon_client_.GetSubjectName(i).SubjectName;
            objects.insert(subject_name);
        }
    }

    return objects;
}

bool ViconRecorder::recordFrame()
{
    frame_time_tracker_->viconFrame(++frames_);
    beginRecord(ofs_) << frame_time_tracker_->viconFrameTime();
    record(ofs_) << frames_;

    record(ofs_) << frame_time_tracker_->depthSensorFrameTime();
    record(ofs_) << frame_time_tracker_->depthSensorFrame();

    // Get the frame number
    Output_GetFrameNumber _Output_GetFrameNumber = vicon_client_.GetFrameNumber();
    record(ofs_) << _Output_GetFrameNumber.FrameNumber;

    // Get the timecode
    /*
    Output_GetTimecode _Output_GetTimecode  = vicon_client_.GetTimecode();
    record(ofs_) << _Output_GetTimecode.Hours;
    record(ofs_) << _Output_GetTimecode.Minutes;
    record(ofs_) << _Output_GetTimecode.Seconds;
    record(ofs_) << _Output_GetTimecode.Frames;
    record(ofs_) << _Output_GetTimecode.SubFrame;
    record(ofs_) << _Output_GetTimecode.Standard;
    record(ofs_) << _Output_GetTimecode.SubFramesPerFrame;
    record(ofs_) << _Output_GetTimecode.UserBits;

    // Get the latency
    record(ofs_) << vicon_client_.GetLatencyTotal().Total;
    */

    bool objectExists = false;
    unsigned int SubjectCount = vicon_client_.GetSubjectCount().SubjectCount;
    for(unsigned int i = 0 ; i < SubjectCount ; ++i)
    {
        // Get the subject name
        std::string object_name = vicon_client_.GetSubjectName(i).SubjectName;
        if (object_name.compare(object_name_) != 0)
        {
            continue;
        }

        // Count the number of segments
        unsigned int segment_count = vicon_client_.GetSegmentCount(object_name).SegmentCount;
        for(unsigned int j = 0 ; j < segment_count ; ++j)
        {
            // Get the segment name
            std::string SegmentName = vicon_client_.GetSegmentName(object_name, j).SegmentName;
            if (SegmentName.compare(object_name_) != 0)
            {
                continue;
            }
            objectExists = true;

            // Get the global segment translation
            Output_GetSegmentGlobalTranslation global_translation =
                    vicon_client_.GetSegmentGlobalTranslation( object_name, SegmentName );
            record(ofs_) << global_translation.Translation[0] / 1000.;
            record(ofs_) << global_translation.Translation[1] / 1000.;
            record(ofs_) << global_translation.Translation[2] / 1000.;

            // Get the global segment rotation as a matrix
            Output_GetSegmentGlobalRotationMatrix gloabl_rotation_matrix =
                    vicon_client_.GetSegmentGlobalRotationMatrix( object_name, SegmentName );
            record(ofs_) << gloabl_rotation_matrix.Rotation[ 0 ];
            record(ofs_) << gloabl_rotation_matrix.Rotation[ 1 ];
            record(ofs_) << gloabl_rotation_matrix.Rotation[ 2 ];
            record(ofs_) << gloabl_rotation_matrix.Rotation[ 3 ];
            record(ofs_) << gloabl_rotation_matrix.Rotation[ 4 ];
            record(ofs_) << gloabl_rotation_matrix.Rotation[ 5 ];
            record(ofs_) << gloabl_rotation_matrix.Rotation[ 6 ];
            record(ofs_) << gloabl_rotation_matrix.Rotation[ 7 ];
            endRecord(ofs_) << gloabl_rotation_matrix.Rotation[ 8 ];

            // Get the global segment rotation in quaternion co-ordinates
            Output_GetSegmentGlobalRotationQuaternion global_rotation_quaternion =
                    vicon_client_.GetSegmentGlobalRotationQuaternion( object_name, SegmentName );
            record(ofs_) << global_rotation_quaternion.Rotation[3]; // w
            record(ofs_) << global_rotation_quaternion.Rotation[0]; // x
            record(ofs_) << global_rotation_quaternion.Rotation[1]; // y
            record(ofs_) << global_rotation_quaternion.Rotation[2]; // z

            break;
        }
    }

    if (!objectExists)
    {
        ROS_WARN("Error: Object <%s> does not exist.", object_name_.c_str());
        return false;
    }

    return true;
}

std::ofstream& ViconRecorder::beginRecord(std::ofstream& ofs)
{
    static bool first_record = true;

    if (!first_record)
    {
        ofs << "\n";
    }

    if (first_record)
    {
       first_record = false;
    }

    ofs << std::setprecision(float_precision_);
    return ofs;
}

std::ofstream& ViconRecorder::record(std::ofstream& ofs)
{
    ofs << " ";
    ofs << std::setprecision(float_precision_);
    return ofs;
}

std::ofstream& ViconRecorder::endRecord(std::ofstream& ofs)
{
    return record(ofs);
}

// ============================================================================================== //
// == STUB ====================================================================================== //
// ============================================================================================== //

ViconRecorderStub::ViconRecorderStub(ros::NodeHandle& node_handle,
                                     std::string vicon_objects_srv_name,
                                     std::string object_verification_srv_name,
                                     std::string vicon_frame_srv_name,
                                     int float_precision):
    float_precision_(float_precision),
    connected_(false),
    hostname_("localhost:801"),
    multicast_address_("244.0.0.0:44801"),
    connect_to_multicast_(false),
    multicast_enabled_(false),
    connect_to_vicon_as_(node_handle,
                         "connect_to_vicon",
                         boost::bind(&ViconRecorderStub::connectCB, this, _1),
                         false)
{
    connect_to_vicon_as_.start();

    vicon_objects_srv_ = node_handle.advertiseService("detect_vicon_objects",
                                                      &ViconRecorderStub::viconObjectsCB,
                                                      this);

    object_verification_srv_ = node_handle.advertiseService("object_exists_verification",
                                                            &ViconRecorderStub::objectExistsCB,
                                                            this);
}

ViconRecorderStub::~ViconRecorderStub()
{

}

void ViconRecorderStub::connectCB(const ConnectToViconGoalConstPtr& goal)
{
    connected_ = true;

    ROS_INFO("Connecting to Vicon");

    ConnectToViconFeedback feedback;
    ConnectToViconResult result;


    ROS_INFO("Connecting to Vicon host %s", goal->host.c_str());
    ROS_INFO("Multicast %s (enabled: %s)",
             goal->multicast_address.c_str(),
             goal->enable_multicast ? "yes" : "no");
    feedback.connected = connected_;
    connect_to_vicon_as_.publishFeedback(feedback);

    while (true)
    {
        boost::shared_lock<boost::shared_mutex> lock(iteration_mutex_);

        if (connect_to_vicon_as_.isPreemptRequested())
        {
            break;
        }
        else if (!ros::ok())
        {
            connect_to_vicon_as_.setAborted(result);
            ROS_WARN("Running depth sensor aborted. ROS shutting down.");
            connected_ = false;
            return;
        }
    }
    connected_ = false;
    connect_to_vicon_as_.setSucceeded(result);
    ROS_INFO("Vicon connection closed.");
}

bool ViconRecorderStub::viconObjectsCB(ViconObjects::Request& request, ViconObjects::Response& response)
{
    boost::upgrade_lock<boost::shared_mutex> lock(iteration_mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);

    if(connected_)
    {
        response.object_names.push_back("object_1");
        response.object_names.push_back("object_2");
        response.object_names.push_back("object_3");
        response.object_names.push_back("object_4");
    }

    return true;
}

bool ViconRecorderStub::objectExistsCB(VerifyObjectExists::Request& request,
                                   VerifyObjectExists::Response& response)
{
    boost::upgrade_lock<boost::shared_mutex> lock(iteration_mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);

    response.exists = false;

    if (connected_)
    {
        if (request.object_name.compare("object_1") == 0)
        {
            response.exists = true;
        }
    }

    return true;
}
