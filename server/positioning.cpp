#include "gazebo_interface.hpp"
#include "positioning.hpp"
#include <unistd.h> 

Positioning::Positioning(GazeboInterface *gz){
	hw = gz;
	yaw = hw->get_angle();
	position = {0,0,0};
}

double Positioning::get_yaw(){
	return yaw;
}

void Positioning::update_yaw(double rotation){
	yaw = 0.1*(yaw+rotation)+0.9*hw->get_angle();
	yaw = fmod(yaw,360);
	yaw = yaw >  180? yaw-360:yaw;
	yaw = yaw < -180? yaw+360:yaw;
}

void Positioning::update_position(std::vector<double> movement, double direction){
	// direction += 45; //The IMU is rotated
	double x_mov = movement.at(0)*cos(direction/180*M_PI);
	double y_mov = movement.at(0)*sin(direction/180*M_PI);

	position.at(0) = position.at(0) + x_mov*cos(yaw/180*M_PI) - y_mov*sin(yaw/180*M_PI);
	position.at(1) = position.at(1) + y_mov*cos(yaw/180*M_PI) + x_mov*sin(yaw/180*M_PI);
	position.at(2) = position.at(2) + movement.at(1);
	std::cout  << "yaw: " << yaw <<"   x: " << position.at(0)<< "  y: " << position.at(1)<<std::endl;
}
