/* Copyright 2019 The MathWorks, Inc. */

//This function is for internal use only. It may be removed in the future.
// EDIT CAREFULLY!
/**Note: 
 * code line from onWorldUpdateStart() and subscribe functions are auto 
 * generated based on user-defined custom message. The commented part 
 * can be removed and edit as per need. Further, the message initialization 
 * is needed in the onWorldUpdateStart(). Please take help of documentation.
 */

#include <iostream>
#include <math.h>
#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <controlCommand.pb.h>
#include <stoupento_mesurement.pb.h>
#include <vector3d.pb.h>


namespace gazebo
{

typedef const boost::shared_ptr<const stoupentoPlugin_msgs::msgs::ControlCommand> stoupentoPlugin_msgs_msgs_ControlCommandPtr;
typedef const boost::shared_ptr<const stoupentoPlugin_msgs::msgs::StoupentoMesurement> stoupentoPlugin_msgs_msgs_StoupentoMesurementPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Vector3d> gazebo_msgs_Vector3dPtr;


class customMessagePlugin : public WorldPlugin
{
  transport::NodePtr node;

  transport::SubscriberPtr commandSubscriberstoupentoplugin_msgs_msgs_controlcommand0;
transport::SubscriberPtr commandSubscriberstoupentoplugin_msgs_msgs_stoupentomesurement0;
transport::SubscriberPtr commandSubscribergazebo_msgs_vector3d0;


  gazebo::transport::PublisherPtr commandPublisherstoupentoplugin_msgs_msgs_controlcommand0;
gazebo::transport::PublisherPtr commandPublisherstoupentoplugin_msgs_msgs_stoupentomesurement0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_vector3d0;


  physics::WorldPtr world;
  gazebo::event::ConnectionPtr worldUpdateStartEventConnection;
  gazebo::event::ConnectionPtr m_worldUpdateEndEventConnection;
  std::mutex pubMutex;

  public:

  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    node = transport::NodePtr(new transport::Node());
    world = _parent;

    // Initialize the node with the world name
    node->Init(world->Name());

    commandSubscriberstoupentoplugin_msgs_msgs_controlcommand0 = node->Subscribe("gazebo/default/stoupentoPlugin_msgs_msgs_ControlCommand/test_subscriber0", &customMessagePlugin::subscribeCallbackstoupentoplugin_msgs_msgs_controlcommand0, this);
commandSubscriberstoupentoplugin_msgs_msgs_stoupentomesurement0 = node->Subscribe("gazebo/default/stoupentoPlugin_msgs_msgs_StoupentoMesurement/test_subscriber0", &customMessagePlugin::subscribeCallbackstoupentoplugin_msgs_msgs_stoupentomesurement0, this);
commandSubscribergazebo_msgs_vector3d0 = node->Subscribe("gazebo/default/gazebo_msgs_Vector3d/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_vector3d0, this);


    commandPublisherstoupentoplugin_msgs_msgs_controlcommand0 = node->Advertise<stoupentoPlugin_msgs::msgs::ControlCommand>("gazebo/default/stoupentoPlugin_msgs_msgs_ControlCommand/test_publisher");
commandPublisherstoupentoplugin_msgs_msgs_stoupentomesurement0 = node->Advertise<stoupentoPlugin_msgs::msgs::StoupentoMesurement>("gazebo/default/stoupentoPlugin_msgs_msgs_StoupentoMesurement/test_publisher");
commandPublishergazebo_msgs_vector3d0 = node->Advertise<gazebo::msgs::Vector3d>("gazebo/default/gazebo_msgs_Vector3d/test_publisher");


    this->worldUpdateStartEventConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&customMessagePlugin::onWorldUpdateStart, this));
    this->m_worldUpdateEndEventConnection = gazebo::event::Events::ConnectWorldUpdateEnd(
        std::bind(&customMessagePlugin::onWorldUpdateEnd, this));
  }

  void onWorldUpdateStart()
  {
    std::lock_guard<std::mutex> lock(pubMutex);
    //stoupentoPlugin_msgs::msgs::ControlCommand  stoupentoplugin_msgs_msgs_controlcommand_msg;
//commandPublisherstoupentoplugin_msgs_msgs_controlcommand0->Publish(stoupentoplugin_msgs_msgs_controlcommand_msg);

//stoupentoPlugin_msgs::msgs::StoupentoMesurement  stoupentoplugin_msgs_msgs_stoupentomesurement_msg;
//commandPublisherstoupentoplugin_msgs_msgs_stoupentomesurement0->Publish(stoupentoplugin_msgs_msgs_stoupentomesurement_msg);

//gazebo::msgs::Vector3d  gazebo_msgs_vector3d_msg;
//commandPublishergazebo_msgs_vector3d0->Publish(gazebo_msgs_vector3d_msg);


  }
  
  void onWorldUpdateEnd()
  {
  }

  void subscribeCallbackstoupentoplugin_msgs_msgs_controlcommand0(stoupentoPlugin_msgs_msgs_ControlCommandPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackstoupentoplugin_msgs_msgs_stoupentomesurement0(stoupentoPlugin_msgs_msgs_StoupentoMesurementPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_vector3d0(gazebo_msgs_Vector3dPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}



};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(customMessagePlugin)
}