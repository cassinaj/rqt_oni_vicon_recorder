
#include "oni_vicon_recorder/vicon_recorder.hpp"

using namespace oni_vicon_recorder;

ViconRecorder::ViconRecorder(ros::NodeHandle& node_handle, int float_precision):
    float_precision_(float_precision),
    hostname_("localhost:801"),
    multicast_address_("244.0.0.0:44801"),
    connect_to_multicast_(false),
    multicast_enabled_(false),
    connect_to_vicon_as_(node_handle,
                         "connect_to_vicon",
                         boost::bind(&ViconRecorder::connectCB, this, _1),
                         false)
{
    connect_to_vicon_as_.start();
}

ViconRecorder::~ViconRecorder()
{

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

void ViconRecorder::connectCB(const ConnectToViconGoalConstPtr &goal)
{
    ROS_INFO("Connecting to Vicon");

    ConnectToViconFeedback feedback;
    ConnectToViconResult result;

    feedback.connected = true;
    connect_to_vicon_as_.publishFeedback(feedback);

    while (true)
    {
        if (connect_to_vicon_as_.isPreemptRequested())
        {
            break;
        }
        else if (!ros::ok())
        {
            connect_to_vicon_as_.setAborted(result);
            ROS_WARN("Running depth sensor aborted. ROS shutting down.");
            return;
        }
    }

    connect_to_vicon_as_.setSucceeded(result);
    ROS_INFO("Vicon connection closed.");
}
