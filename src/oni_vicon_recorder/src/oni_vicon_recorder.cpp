

#include "oni_vicon_recorder/oni_vicon_recorder.hpp"


OniViconRecorder::OniViconRecorder(ros::NodeHandle& node_handler):
    node_handler_(node_handler)
{

}

OniViconRecorder::~OniViconRecorder()
{

}

void OniViconRecorder::run()
{
    while(ros::ok())
    {



        ros::spinOnce();
    }
}
