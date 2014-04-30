
#include "oni_vicon_recorder/vicon_recorder.hpp"

using namespace oni_vicon_recorder;
using namespace ViconDataStreamSDK::CPP;

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

void ViconRecorder::connectCB(const ConnectToViconGoalConstPtr& goal)
{
    ConnectToViconFeedback feedback;
    ConnectToViconResult result;

    ROS_INFO("Connecting to Vicon");

    hostname_ = goal->host;
    multicast_enabled_ = goal->enable_multicast;
    multicast_address_ = goal->multicast_address;
    connect_to_multicast_ = goal->connect_to_multicast;
    int retries = goal->retry;

    ros::Duration wait_time(1.);
    while (retries > 0 && !vicon_client_.IsConnected().Connected)
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

        if (retries > 0)
        {
            ROS_INFO("No connecting to Vicon system. Retry %d ...", (goal->retry - retries));
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

    ROS_INFO("Connected to Vicon System.");

    ros::Rate check_rate(100);
    while (true)
    {
        boost::shared_lock<boost::shared_mutex> lock(iteration_mutex_);

        if (connect_to_vicon_as_.isPreemptRequested() || !ros::ok())
        {
            break;
        }

        check_rate.sleep();
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

void ViconRecorder::record_stuff()
{

}

std::string ViconRecorder::Adapt( const bool i_Value )
{
    return i_Value ? "True" : "False";
}

std::string ViconRecorder::Adapt( const Direction::Enum i_Direction )
{
    switch( i_Direction )
    {
    case Direction::Forward:
        return "Forward";
    case Direction::Backward:
        return "Backward";
    case Direction::Left:
        return "Left";
    case Direction::Right:
        return "Right";
    case Direction::Up:
        return "Up";
    case Direction::Down:
        return "Down";
    default:
        return "Unknown";
    }
}

std::string ViconRecorder::Adapt( const DeviceType::Enum i_DeviceType )
{
    switch( i_DeviceType )
    {
    case DeviceType::ForcePlate:
        return "ForcePlate";
    case DeviceType::Unknown:
    default:
        return "Unknown";
    }
}

std::string ViconRecorder::Adapt( const Unit::Enum i_Unit )
{
    switch( i_Unit )
    {
    case Unit::Meter:
        return "Meter";
    case Unit::Volt:
        return "Volt";
    case Unit::NewtonMeter:
        return "NewtonMeter";
    case Unit::Newton:
        return "Newton";
    case Unit::Kilogram:
        return "Kilogram";
    case Unit::Second:
        return "Second";
    case Unit::Ampere:
        return "Ampere";
    case Unit::Kelvin:
        return "Kelvin";
    case Unit::Mole:
        return "Mole";
    case Unit::Candela:
        return "Candela";
    case Unit::Radian:
        return "Radian";
    case Unit::Steradian:
        return "Steradian";
    case Unit::MeterSquared:
        return "MeterSquared";
    case Unit::MeterCubed:
        return "MeterCubed";
    case Unit::MeterPerSecond:
        return "MeterPerSecond";
    case Unit::MeterPerSecondSquared:
        return "MeterPerSecondSquared";
    case Unit::RadianPerSecond:
        return "RadianPerSecond";
    case Unit::RadianPerSecondSquared:
        return "RadianPerSecondSquared";
    case Unit::Hertz:
        return "Hertz";
    case Unit::Joule:
        return "Joule";
    case Unit::Watt:
        return "Watt";
    case Unit::Pascal:
        return "Pascal";
    case Unit::Lumen:
        return "Lumen";
    case Unit::Lux:
        return "Lux";
    case Unit::Coulomb:
        return "Coulomb";
    case Unit::Ohm:
        return "Ohm";
    case Unit::Farad:
        return "Farad";
    case Unit::Weber:
        return "Weber";
    case Unit::Tesla:
        return "Tesla";
    case Unit::Henry:
        return "Henry";
    case Unit::Siemens:
        return "Siemens";
    case Unit::Becquerel:
        return "Becquerel";
    case Unit::Gray:
        return "Gray";
    case Unit::Sievert:
        return "Sievert";
    case Unit::Katal:
        return "Katal";

    case Unit::Unknown:
    default:
        return "Unknown";
    }
}


// ============================================================================================== //
// == STUB ====================================================================================== //
// ============================================================================================== //

ViconRecorderStub::ViconRecorderStub(ros::NodeHandle& node_handle, int float_precision):
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
