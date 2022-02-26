#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdint>
#include <iostream>
#include <cmath>
#include "gazebo_interface.hpp"


GazeboInterface::GazeboInterface(){}

void GazeboInterface::init(bool verbose, std::string suffix){
	suffix="";
	/* 
	 * Boilerplate code to connect to gazebo and properly set up topics
	 */
	// Load gazebo
	gazebo::client::setup(0, NULL);
	// Create node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	// Listen to sensors topics
	sub_mag = node->Subscribe("/gazebo/default/DJIM100"+suffix+"/Dummy/Magnetometer/Magnetometer", &GazeboInterface::cb_angle_magnetic, this);
	sub_img	= node->Subscribe("/gazebo/default/DJIM100"+suffix+"/PiWithSensors/PiCameraModule/link/Camera/image", &GazeboInterface::cb_img, this);
	if(verbose){
		std::cout << "Subscribed to camera data" << std::endl;
	} 

	sub_dist= node->Subscribe("/gazebo/default/DJIM100"+suffix+"/PiWithSensors/VL53L1X/link/VL53L1X_rangefinder/scan", &GazeboInterface::cb_distance, this);
	if(verbose){
		std::cout << "Subscribed to rangefinder data" << std::endl;
	} 

	// Subscribe to pose data
	sub_pos	= node->Subscribe("/gazebo/default/DJIM100"+suffix+"/pose", &GazeboInterface::cb_pose, this);
	sub_rot = node->Subscribe("/gazebo/default/DJIM100"+suffix+"/angle", &GazeboInterface::cb_angle, this);
	if(verbose){
		std::cout << "Subscribed to pose data" << std::endl;
	} 
	// Subscribe to inertial unit
	sub_imu = node->Subscribe("/gazebo/default/DJIM100"+suffix+"/Dummy/imu/imu", &GazeboInterface::cb_imu, this);
	if(verbose){
		std::cout << "Subscribed to IMU data" << std::endl;
	} 

	// Publishers
	pub_vel = node->Advertise<gazebo::msgs::Vector3d>("~/DJIM100"+suffix+"/lin_vel_cmd");
	pub_vel->WaitForConnection();
	if(verbose){
		std::cout << "Linear velocity controller connected" <<std::endl ;
	} 

	pub_ang = node->Advertise<gazebo::msgs::Vector3d>("~/DJIM100"+suffix+"/ang_vel_cmd");
	pub_ang->WaitForConnection();
	if(verbose){
		std::cout << "Angular velocity controller connected" <<std::endl ;
	} 

	pub_phys = node->Advertise<gazebo::msgs::Physics>("~/physics");
	pub_phys->WaitForConnection();

	if(verbose){
		std::cout << "Physics controller connected" <<std::endl ;
	} 
	// Start with object frame of reference
	set_object_frame();

	pub_world = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
	pub_world->WaitForConnection();

	gazebo::msgs::WorldControl msg;
	msg.mutable_reset()->set_time_only(true);
	pub_world->Publish(msg);

	if(verbose){
		std::cout << "World controller connected" <<std::endl ;
	} 

}

void GazeboInterface::quit_listener() {
	// Make sure to shut everything down.
	gazebo::client::shutdown();
}

double GazeboInterface::get_angle(){
	return _angle_magnetic;
}
unsigned int GazeboInterface::get_distance(){
	while(!_distance){}
	return _distance;
}

double GazeboInterface::get_true_yaw(){
	return 180*_angle.at(2)/M_PI;
}
std::vector<double> GazeboInterface::get_imu(){
	return _imu;
}
cv::Mat GazeboInterface::getImage(){
	while(_frame.size().width <= 0 || _frame.size().height <= 0){}
	return _frame.clone();
}

void GazeboInterface::set_object_frame(){
	_world_reference = 0;
}
void GazeboInterface::set_world_frame(){
	_world_reference = 1;
}

/*
 * Publisher functions
 */
void GazeboInterface::set_linear_velocity(std::vector<double> v){
	gazebo::msgs::Vector3d msg;
	if(!_world_reference){
		// rotate vector to offset object rotation
		std::vector<double> v_rot = {0,0};
		double yaw = _angle.at(2);
		v_rot[0] = v.at(0)*cos(yaw) - v.at(1)*sin(yaw);
		v_rot[1] = v.at(1)*cos(yaw) + v.at(0)*sin(yaw);
		gazebo::msgs::Set(&msg, ignition::math::Vector3d(v_rot.at(0),v_rot.at(1),v.at(2)));
	} else {
		// the velocity is interpreted as relative to the object reference frame
		gazebo::msgs::Set(&msg, ignition::math::Vector3d(v.at(0),v.at(1),v.at(2)));
	}
	pub_vel->Publish(msg);
}

void GazeboInterface::set_angular_velocity(std::vector<double> v){
	gazebo::msgs::Vector3d msg;
	gazebo::msgs::Set(&msg, ignition::math::Vector3d(v.at(0),v.at(1),v.at(2)));
	pub_ang->Publish(msg);
}

void GazeboInterface::resume_simulation(){
	gazebo::msgs::WorldControl msg;
	msg.set_pause(0);
	pub_world->Publish(msg);
	pub_world->Publish(msg);
}

void GazeboInterface::pause_simulation(){
	gazebo::msgs::WorldControl msg;
	msg.set_pause(1);
	pub_world->Publish(msg);
}

void GazeboInterface::set_sim_speed(int real_time_percentage){
	gazebo::msgs::Physics msg;
	msg.set_real_time_update_rate(10*real_time_percentage);
	pub_phys->Publish(msg);
}

/*
 * Callback functions for subscribed topics
 */

void GazeboInterface::cb_img(ConstImageStampedPtr &msg) {
	/*
	 * Callback for the camera image topic, converts it to an OpenCV Mat
	 */
	int width;
	int height;
	char *data;

	width = (int) msg->image().width();
	height = (int) msg->image().height();
	//+1 for null terminate
	data = new char[msg->image().data().length() + 1];

	memcpy(data, msg->image().data().c_str(), msg->image().data().length());
	//gazebo output rgb data
	//PixelFormat.R8G8B8
	cv::Mat image(height, width, CV_8UC3, data);
	_frame = image.clone();
	delete data;  // DO NOT FORGET TO DELETE THIS, ELSE GAZEBO WILL TAKE ALL YOUR MEMORY
}

void GazeboInterface::cb_imu(ConstIMUPtr &msg){
	_sim_time.sec  = msg->stamp().sec();
	_sim_time.nsec = msg->stamp().nsec();

	_imu.at(0)= msg->stamp().sec()+(double)msg->stamp().nsec()/1000000000;	
	_imu.at(1)= msg->angular_velocity().x();
	_imu.at(2)=	msg->angular_velocity().y();
	_imu.at(3)= msg->angular_velocity().z();
	_imu.at(4)=	msg->linear_acceleration().x();
	_imu.at(5)= msg->linear_acceleration().y();
	_imu.at(6)=	msg->linear_acceleration().z();
}


void GazeboInterface::cb_distance(ConstLaserScanStampedPtr &msg) {
	/*
	 * Callback for the laser scan topic, since the actual hardware is a
	 * rangefinder, only the closest ray is used as distance, discarding
	 * the rest of the data.
	 */
	//_distance = (int)(msg->range_min * 1000 + 0.5);
	google::protobuf::RepeatedField<double> ranges = msg->scan().ranges() ;
	double min = ranges.Get(0);
	for(int i = 1 ; i < ranges.size(); i ++){
		if(ranges.Get(i) < min){
			min = ranges.Get(i);
		}
	}
	min = (min > 8 ? 8 : min );
	_distance = (int)(min * 1000 +0.5) ;
}

void GazeboInterface::cb_pose(ConstVector3dPtr &msg){
	/*
	 * Updates the 3d position of the object (Ground truth, should not be 
	 * accessed by any code that operates the object).
	 */
	_pose = {msg->x(), msg->y(), msg->z()};
}

void GazeboInterface::cb_angle(ConstVector3dPtr &msg){
	/*
	 * Updates the Euler angles of the object (Ground truth, should not be 
	 * accessed by any code that operates the object).
	 */
	_angle = {msg->x(), msg->y(), msg->z()};
}
void GazeboInterface::cb_angle_magnetic(ConstMagnetometerPtr &msg){
	_angle_magnetic = 180 * atan2(msg->field_tesla().y(), msg->field_tesla().x())/M_PI -120;
	_angle_magnetic = _angle_magnetic > 180 ?_angle_magnetic - 360:_angle_magnetic;
	_angle_magnetic = _angle_magnetic <-180 ?_angle_magnetic + 360:_angle_magnetic;
}

timestamp_t GazeboInterface::get_sim_time(){
	return _sim_time;
}
