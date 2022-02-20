#ifndef __Quadrotor_plugin__
#define __Quadrotor_plugin__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo {
	class Quadrotor_plugin : public ModelPlugin {

		public : Quadrotor_plugin(){}
		public : virtual ~Quadrotor_plugin(){}
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
					this->model = _model;
					this->updateConnection = event::Events::ConnectWorldUpdateBegin( std::bind(&Quadrotor_plugin::Update, this, std::placeholders::_1));
					std::cout << "\nThe Quadrotor plugin is attached to model[" << _model->GetName() << "]"<<std::endl;
					// Create the node
					this->node = transport::NodePtr(new transport::Node());
					this->node->Init(this->model->GetWorld()->Name());
					// Create a topic name
					std::string pose_topic_str = "~/" + this->model->GetName() + "/pose";
					std::string angle_topic_str = "~/" + this->model->GetName() + "/angle";
					std::string lin_vel_topic = "~/" + this->model->GetName() + "/lin_vel_cmd";
					std::string ang_vel_topic = "~/" + this->model->GetName() + "/ang_vel_cmd";
					std::string set_pos_topic = "~/" + this->model->GetName() + "/set_pose_cmd";
					// Subscribe to the topic, and register a callback
					this->sub_lin = this->node->Subscribe(lin_vel_topic, &Quadrotor_plugin::LinVelMsg, this);
					this->sub_ang = this->node->Subscribe(ang_vel_topic, &Quadrotor_plugin::AngVelMsg, this);
					this->sub_pose = this->node->Subscribe(set_pos_topic, &Quadrotor_plugin::NewPoseMsg, this);
					// Create publishers
					this->pose_topic = this->node->Advertise<msgs::Vector3d>(pose_topic_str);
					this->angle_topic = this->node->Advertise<msgs::Vector3d>(angle_topic_str);
				}

		public: void Update(const common::UpdateInfo &_info) {
					//Link velocity instantaneously without applying forces
					model->GetLink("Dummy")->SetLinearVel(this->vel);
					model->GetLink("Dummy")->SetAngularVel(this->angVel);
					// Publish position ground truth
					gazebo::msgs::Vector3d msg_rot, msg_pos;
					ignition::math::v4::Vector3<double> pos = this->model->WorldPose().Pos();
					ignition::math::v4::Vector3<double> rot = this->model->WorldPose().Rot().Euler();
					gazebo::msgs::Set(&msg_rot, ignition::math::Vector3d(rot.X(),rot.Y(),rot.Z()));
					gazebo::msgs::Set(&msg_pos, ignition::math::Vector3d(pos.X(),pos.Y(),pos.Z()));
					this->angle_topic->Publish(msg_rot);
					this->pose_topic->Publish(msg_pos);
				} 

		private: void NewPoseMsg(ConstPosePtr &_msg) {
					 std::cout << "\nQuadrotor plugin set_pose"  << std::endl;
					 std::cout << "\t Old pose:     " << this->model->WorldPose() << std::endl;
					 model->SetWorldPose(ignition::math::Pose3d(
								 _msg->position().x(),
								 _msg->position().y(),
								 _msg->position().z(),
								 _msg->orientation().w(),
								 _msg->orientation().x(),
								 _msg->orientation().y(),
								 _msg->orientation().z()));
					 std::cout << "\t New pose:     " << this->model->WorldPose() << std::endl;
				 }

		private: void AngVelMsg(ConstVector3dPtr &_msg) {
					 std::cout << "\nQuadrotor plugin set_ang_velocity"  << std::endl;
					 std::cout << "\t Old velocity: " << this->model->RelativeAngularVel() <<std::endl;
					 std::cout << "\t Old velocity: " << this->angVel <<std::endl;
					 this->setAngVelocity(_msg);
					 std::cout << "\t New velocity: " << this->angVel <<std::endl;
				 }
		private: void LinVelMsg(ConstVector3dPtr &_msg) {
					 std::cout << "\nQuadrotor plugin set_lin_velocity"  << std::endl;
					 std::cout << "\t Old velocity: " << this->model->RelativeLinearVel() <<std::endl;
					 std::cout << "\t Old velocity: " << this->vel <<std::endl;
					 this->setLinVelocity(_msg);
					 std::cout << "\t New velocity: " << this->vel <<std::endl;
				 }
		public : void setAngVelocity(ConstVector3dPtr &angVel){
					 this->angVel = {angVel->x(), angVel->y(), angVel->z()};
				 }
		public : void setLinVelocity(ConstVector3dPtr &vel){
					 this->vel = {vel->x(), vel->y(), vel->z()};
				 }
		public: physics::ModelPtr model;
				/// \brief object for callback connection
		public: event::ConnectionPtr updateConnection;
				//
				/// \brief A node used for transport
		private: transport::NodePtr node;
				 transport::PublisherPtr pose_topic;
				 transport::PublisherPtr angle_topic;
				 transport::SubscriberPtr sub_lin;
				 transport::SubscriberPtr sub_ang;
				 transport::SubscriberPtr sub_pose;
				 int update_num = 0;
				 ignition::math::Vector3d vel = {0,0,0};
				 ignition::math::Vector3d angVel = {0,0,0};
	};
	GZ_REGISTER_MODEL_PLUGIN(Quadrotor_plugin)
}

#endif
