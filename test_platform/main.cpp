#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdint>
#include <iostream>
#include <cmath>
#include <random>
#include <stdlib.h>

int waiting = 1;
ignition::math::Vector3d lower_corner, upper_corner;

void set_first_vertex(ConstVector3dPtr &_msg){
	upper_corner = {_msg->x(), _msg->y(), _msg->z()};
	waiting = 0;
}

void set_second_vertex(ConstVector3dPtr &_msg){
	lower_corner = {_msg->x(), _msg->y(), _msg->z()};
	waiting = 0;
}

ignition::math::Box get_bounding_box(std::string model){
	/*
	 * Given a model name returns its bounding box
	 */
	gazebo::client::setup(0, NULL);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	//std::cout << "Created node" << std::endl;

	gazebo::transport::SubscriberPtr sub_box;
	// Subscribes to Max Corner topic.
	waiting = 1;
	sub_box = node->Subscribe("/gazebo/default/"+model+"/box1", set_first_vertex);
	//std::cout << "Subbed first corner" << std::endl;
	// Waits for update
	while(waiting){ }
	//Unsubscribes
	sub_box->Unsubscribe();
	waiting = 1;
	// Subscribes to Min Corner topic.
	sub_box = node->Subscribe("/gazebo/default/"+model+"/box2", set_second_vertex);
	//std::cout << "Subbed second corner" << std::endl;
	// Waits for update
	while(waiting){}
	//Unsubscribes
	sub_box->Unsubscribe();

	return ignition::math::Box( lower_corner, upper_corner);
}

int main(int argc, char **argv) {
	double padding = 1;	// Minimum distance from wall
	std::vector<ignition::math::Box> bounding_boxes; 	// List of all boxes
	std::vector<double> sizes;							// Size of each box
	std::vector<double> cumul_sizes;					// Running sum of the sizes
	for(int i = 1; i < argc-1 ; i++){
		/*
		 * For each of the models passed as argument
		 * gets its bounding box,the bounding box size, and the running sum
		 */
		bounding_boxes.push_back(get_bounding_box(argv[i]));
		sizes.push_back( 
				bounding_boxes.at(i-1).XLength() * 
				bounding_boxes.at(i-1).YLength() * 
				bounding_boxes.at(i-1).ZLength() ); 
		cumul_sizes.push_back(sizes.at(i-1)+(i>1?cumul_sizes.at(i-2):0));
	}

	// Seeds the random number generator with the last value passed as argument
	std::mt19937 mt(atoi(argv[argc-1]));
	// Each box has a probability of being picked proportional to its volume
	std::uniform_real_distribution<double> dist(0, cumul_sizes.back());
	double rand_double = dist(mt);
	int selected_box = 0;
	for(int i = 0; i < cumul_sizes.size() ; i++){
		if(rand_double <= cumul_sizes.at(i)){
			selected_box = i;
			break;
		} 
	}
	//std::cout << "Selected box #"<<selected_box<<std::endl << bounding_boxes.at(selected_box) << std::endl;

	// Generates random coordinates within the box minus the safety padding
	std::uniform_real_distribution<double> dist_x(bounding_boxes.at(selected_box).Min().X()+padding, bounding_boxes.at(selected_box).Max().X()-padding);
	double x = dist_x(mt);
	std::uniform_real_distribution<double> dist_y(bounding_boxes.at(selected_box).Min().Y()+padding, bounding_boxes.at(selected_box).Max().Y()-padding);
	double y = dist_y(mt);
	std::uniform_real_distribution<double> dist_z(bounding_boxes.at(selected_box).Min().Z()+padding, bounding_boxes.at(selected_box).Max().Z()-padding);
	double z = dist_z(mt);
	std::uniform_real_distribution<double> dist_yaw(-M_PI,M_PI);
	double yaw = dist_yaw(mt);

	// Sends the new pose to the object
	gazebo::client::setup(0, NULL);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	gazebo::transport::PublisherPtr	pub_pose = node->Advertise<gazebo::msgs::Pose>("~/DJIM100/set_pose_cmd");
	pub_pose->WaitForConnection();

	gazebo::msgs::Pose msg;
	gazebo::msgs::Set(&msg, ignition::math::Pose3d(x,y,z,0,0,yaw));
	pub_pose->Publish(msg);
	std::cout << "Setting pose to:  " << ignition::math::Pose3d(x,y,z,0,0,yaw) <<std::endl;

	return 0;
}
