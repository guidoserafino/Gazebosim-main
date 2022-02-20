#ifndef __Building_plugin__
#define __Building_plugin__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo {
	class Building_plugin : public ModelPlugin {

		public : Building_plugin(){}
		public : virtual ~Building_plugin(){}
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
					this->model = _model;
					this->updateConnection = event::Events::ConnectWorldUpdateBegin( std::bind(&Building_plugin::Update, this, std::placeholders::_1));
					std::cout << "\nThe Building plugin is attached to model[" << _model->GetName() << "]"<<std::endl;
					// Create the node
					this->node = transport::NodePtr(new transport::Node());
					this->node->Init(this->model->GetWorld()->Name());
					// Create a topic name
					std::string bounding_box_topic_str1 = "~/" + this->model->GetName() + "/box1";
					std::string bounding_box_topic_str2 = "~/" + this->model->GetName() + "/box2";
					this->bounding_box_topic1 = this->node->Advertise<msgs::Vector3d>(bounding_box_topic_str1);
					this->bounding_box_topic2 = this->node->Advertise<msgs::Vector3d>(bounding_box_topic_str2);
				}

		public: void Update(const common::UpdateInfo &_info) {
					if(common::Time::GetWallTime() - this->prevUpdateTime < this->updateRate){
						return;
					}
					ignition::math::Box boundingBox = this->model->BoundingBox();
					msgs::Vector3d max_corner, min_corner;
					msgs::Set(&max_corner, boundingBox.Max());
					msgs::Set(&min_corner, boundingBox.Min());
					this->bounding_box_topic1->Publish(max_corner);
					this->bounding_box_topic2->Publish(min_corner);
					this->prevUpdateTime = common:: Time::GetWallTime();
				} 

		public: physics::ModelPtr model;
				/// \brief object for callback connection
		public: event::ConnectionPtr updateConnection;
				//
				/// \brief A node used for transport
		private: transport::NodePtr node;
				 transport::PublisherPtr bounding_box_topic1;
				 transport::PublisherPtr bounding_box_topic2;
				 common::Time updateRate = common::Time(0, common::Time::SecToNano(0.125));
				 common::Time prevUpdateTime = common::Time::GetWallTime();
	};
	GZ_REGISTER_MODEL_PLUGIN(Building_plugin)
}

#endif
