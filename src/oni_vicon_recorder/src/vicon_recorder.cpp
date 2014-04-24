
#include "oni_vicon_recorder/vicon_recorder.hpp"

using namespace oni_vicon_recorder;

ViconRecorder::ViconRecorder(ros::NodeHandle& node_handle, int float_precision):
    float_precision_(float_precision),
    connected_(false),
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

    vicon_objects_srv_ = node_handle.advertiseService("detect_vicon_objects",
                                                      &ViconRecorder::viconObjectsCB,
                                                      this);

    object_verification_srv_ = node_handle.advertiseService("object_exists_verification",
                                                            &ViconRecorder::objectExistsCB,
                                                            this);
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

void ViconRecorder::connectCB(const ConnectToViconGoalConstPtr& goal)
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

bool ViconRecorder::viconObjectsCB(ViconObjects::Request& request, ViconObjects::Response& response)
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

bool ViconRecorder::objectExistsCB(VerifyObjectExists::Request& request,
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
