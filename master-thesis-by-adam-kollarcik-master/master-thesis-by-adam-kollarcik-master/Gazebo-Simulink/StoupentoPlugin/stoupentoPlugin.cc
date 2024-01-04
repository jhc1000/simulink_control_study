#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include "stoupento_mesurement.pb.h"
#include "controlCommand.pb.h"
#include <sdf/sdf.hh>
#include <functional>
#include <iostream>
#include <string>
#include <algorithm>



namespace gazebo
{
   typedef const boost::shared_ptr<const stoupentoPlugin_msgs::msgs::ControlCommand> ControlCommandPtr;
class EncoderPlugin : public ModelPlugin
{
  private: transport::NodePtr node;
  private: transport::PublisherPtr publisher;
  private: physics::ModelPtr model;
  private: physics::JointPtr joint1L;
  private: physics::JointPtr joint2L;
  private: physics::JointPtr joint3L;
  private: physics::JointPtr joint4L;
  private: physics::JointPtr joint1R;
  private: physics::JointPtr joint2R;
  private: physics::JointPtr joint3R;
  private: physics::JointPtr joint4R;
  private: physics::LinkPtr wheelL;
  private: physics::LinkPtr wheelR;
  private: event::ConnectionPtr updateConnection;
  private: transport::SubscriberPtr commandSubscriber;
  private: transport::SubscriberPtr imuSubscriber;
  private: double u0L,u3L,u0R,u3R;
  private: ignition::math::Vector3d bodyLinAcc,bodyAngVel;

    public: EncoderPlugin() : ModelPlugin()
            {
              // printf("Hello World!\n");
            }
  public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  { 
    this->model = _model;
          // this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          // std::bind(&EncoderPlugin::OnUpdate, this));
    //  printf("Hello World!\n");
    this->model = _model;
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());
    this->publisher  = node->Advertise<stoupentoPlugin_msgs::msgs::StoupentoMesurement>("~/stoupentoMesurement");
    this->commandSubscriber = node->Subscribe("~/controlCommand",&EncoderPlugin::callback,this);
    this->imuSubscriber = node->Subscribe("~/Stoupento/Stoupento/body/body_imu/imu",&EncoderPlugin::IMUcallback,this);
    this->u0L =0;
    this->u0R = 0;
    this-> u3L =0;
    this->u3R =0;
    this->bodyLinAcc = ignition::math::Vector3d(0,0,0);
    this->bodyAngVel = ignition::math::Vector3d(0,0,0);

     if (!_sdf->HasElement("wheelL") || !_sdf->HasElement("wheelR") || !_sdf->HasElement("joint1L")  || !_sdf->HasElement("joint2L")  || !_sdf->HasElement("joint3L")  || !_sdf->HasElement("joint4L")
       || !_sdf->HasElement("joint1R")  || !_sdf->HasElement("joint2R") || !_sdf->HasElement("joint3R")  || !_sdf->HasElement("joint4R") ){
    gzerr << "Plugin missing wheel element\n";
     }
    this->joint1L = _model->GetJoint(_sdf->GetElement("joint1L")->Get<std::string>());
    this->joint2L = _model->GetJoint(_sdf->GetElement("joint2L")->Get<std::string>());
    this->joint3L = _model->GetJoint(_sdf->GetElement("joint3L")->Get<std::string>());
    this->joint4L = _model->GetJoint(_sdf->GetElement("joint4L")->Get<std::string>());
    this->joint1R = _model->GetJoint(_sdf->GetElement("joint1R")->Get<std::string>());
    this->joint2R = _model->GetJoint(_sdf->GetElement("joint2R")->Get<std::string>());
    this->joint3R = _model->GetJoint(_sdf->GetElement("joint3R")->Get<std::string>());
    this->joint4R = _model->GetJoint(_sdf->GetElement("joint4R")->Get<std::string>());
    this->wheelL = _model->GetLink(_sdf->GetElement("wheelL")->Get<std::string>());
    this->wheelR = _model->GetLink(_sdf->GetElement("wheelR")->Get<std::string>());

    if (!this->joint1L || !this->joint2L|| !this->joint3L || !this->joint4L|| !this->joint1R
    || !this->joint2R|| !this->joint3R|| !this->joint4R || !this->wheelL || !this->wheelR){
      gzerr << "Unable to find joint or wheel elenet\n";}
    
    // this->sendMes();


       this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&EncoderPlugin::OnUpdate, this));

      // this->updateConnection = event::Events::ConnectWorldUpdateEnd (
      //     std::bind(&EncoderPlugin::OnUpdate, this));
  }

      public: void OnUpdate()
    {
        this->sendMes();
    }

    public: void Init(){


    }



      public: void callback(ControlCommandPtr &msg)
  {



    this->u0L =msg->u0l();
    this->u0R = msg->u0r();
    this-> u3L =msg->u3l();
    this->u3R =msg->u3r();



  }



       public: void IMUcallback(ConstIMUPtr &msg)
  {


    
    this->bodyLinAcc =ConvertIgn(msg->linear_acceleration());
    this->bodyAngVel =ConvertIgn(msg->angular_velocity());


  }




      public: void Reset(){
    stoupentoPlugin_msgs::msgs::StoupentoMesurement msg;

    this->u0L =0;
    this->u0R = 0;
    this-> u3L =0;
    this->u3R =0;
    this->bodyLinAcc = ignition::math::Vector3d(0,0,0);
    this->bodyAngVel = ignition::math::Vector3d(0,0,0);

    msg.set_left_wheel_vel(0);

    msg.set_left_joint2_vel(0);
    msg.set_left_joint1_vel(0);
    msg.set_left_joint3_vel(0);
    msg.set_left_joint4_vel(0);

    //
    msg.set_left_wheel_ang(0);

    msg.set_left_joint1_ang(0);
    msg.set_left_joint2_ang(0);
    msg.set_left_joint3_ang(0);
    msg.set_left_joint4_ang(0);
    //

    msg.set_left_wheel_torque(this->u0L);
    msg.set_left_body_torque(this->u3L);
    //
    msg.set_right_wheel_vel(0);

    msg.set_right_joint1_vel(0);
    msg.set_right_joint2_vel(0);
    msg.set_right_joint3_vel(0);
    msg.set_right_joint4_vel(0);
    //

        msg.set_right_wheel_ang(0);

    msg.set_right_joint1_ang(0);
    msg.set_right_joint2_ang(0);
    msg.set_right_joint3_ang(0);
    msg.set_right_joint4_ang(0);

    //
    msg.set_right_wheel_torque(this->u0R);
    msg.set_right_body_torque(this->u3R);


   msgs::Set(msg.mutable_linear_acc(),this->bodyLinAcc);
   msgs::Set(msg.mutable_angular_vel(),this->bodyAngVel);
    
    this->publisher->Publish(msg);

    }

    private: void sendMes(){
    stoupentoPlugin_msgs::msgs::StoupentoMesurement msg;
    double v0L, v1L, v2L, v3L ,v4L, v0R, v1R, v2R, v3R ,v4R, a0L, a1L, a2L, a3L, a4L, a0R ,a1R, a2R, a3R, a4R;









    v0L = -this->wheelL->RelativeAngularVel().Z();
    v1L = -this->joint1L->GetParent()->WorldAngularVel().Y();
    v2L = this->joint2L->GetVelocity(0);
    v3L = this->joint3L->GetVelocity(0) ;
    v4L = this->joint4L->GetVelocity(0) ; 

    a0L = (this->wheelL->WorldPose().Rot()*this->joint1L->GetParent()->WorldPose().Rot().Inverse()).Inverse().Yaw();
    a1L = -this->joint1L->GetParent()->WorldPose().Rot().Pitch() -IGN_PI*0.25;
    a2L = this->joint2L->Position();
    a3L = this->joint3L->Position();
    a4L = this->joint4L->Position();

    v0R = -this->wheelR->RelativeAngularVel().Z();
    v1R = -this->joint1R->GetParent()->WorldAngularVel().Y();
    v2R = this->joint2R->GetVelocity(0) ;
    v3R = this->joint3R->GetVelocity(0) ;
    v4R = this->joint4R->GetVelocity(0) ; 
    a0R = (this->wheelR->WorldPose().Rot()*this->joint1R->GetParent()->WorldPose().Rot().Inverse()).Inverse().Yaw();
    a1R = -this->joint1R->GetParent()->WorldPose().Rot().Pitch() -IGN_PI*0.25;
    a2R = this->joint2R->Position();
    a3R = this->joint3R->Position();
    a4R = this->joint4R->Position();


    msg.set_left_wheel_vel(v0L);

    msg.set_left_joint1_vel(v1L);
    msg.set_left_joint2_vel(v2L);
    msg.set_left_joint3_vel(v3L);
    msg.set_left_joint4_vel(v4L);

    //
    msg.set_left_wheel_ang(a0L);

    msg.set_left_joint1_ang(a1L);
    msg.set_left_joint2_ang(a2L);
    msg.set_left_joint3_ang(a3L);
    msg.set_left_joint4_ang(a4L);

   msg.set_left_wheel_torque(this->u0L);
   msg.set_left_body_torque(this->u3L);
    //
    msg.set_right_wheel_vel(v0R);

    msg.set_right_joint1_vel(v1R);
    msg.set_right_joint2_vel(v2R);
    msg.set_right_joint3_vel(v3R);
    msg.set_right_joint4_vel(v4R);
    // 
    msg.set_right_wheel_ang(a0R);

    msg.set_right_joint1_ang(a1R);
    msg.set_right_joint2_ang(a2R);
    msg.set_right_joint3_ang(a3R);
    msg.set_right_joint4_ang(a4R);
    
    

       msg.set_right_wheel_torque(this->u0R);
       msg.set_right_body_torque(this->u3R);
    //

   msgs::Set(msg.mutable_linear_acc(),this->bodyLinAcc);
   msgs::Set(msg.mutable_angular_vel(),this->bodyAngVel);
          this->joint1L->SetForce(0,-this->u0L);
      this->joint3L->SetForce(0,this->u3L);
            this->joint1R->SetForce(0,-this->u0R);
      this->joint3R->SetForce(0,this->u3R);
    this->publisher->Publish(msg);


    }

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(EncoderPlugin)
}