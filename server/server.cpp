// Pi running control logic
#include <netinet/tcp.h>
#include <unistd.h> 
#include <stdio.h> 
#include <arpa/inet.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <string.h> 
#include <opencv2/opencv.hpp>
#include "net_protocol.cpp"
#include "gazebo_interface.hpp"

GazeboInterface::GazeboInterface(){}
//Boilerplate
void GazeboInterface::init_listener(bool verbose, std::string suffix){
	_socket = init_server();
	send_message(_socket, "INIT");
	send_message(_socket, (verbose?"VERBOSE":"-"));
	send_message(_socket, suffix);
}
void GazeboInterface::quit_listener(){
	send_message(_socket, "QUIT");
}

// Obtain sensor data
double GazeboInterface::get_angle(){
	send_message(_socket,"GET_ANGLE");
	return string_to_vect3d(rcv_msg(_socket)).at(0);
}

std::vector<double> GazeboInterface::get_imu(){
	send_message(_socket,"GET_IMU");
	return string_to_vect3d(rcv_msg(_socket));
}
unsigned int GazeboInterface::get_distance(){
	send_message(_socket,"GET_DISTANCE");
	return stoul(rcv_msg(_socket), 0, 10);
}
cv::Mat GazeboInterface::getImage(int width, int height){
	send_message(_socket,"GET_IMG");
	cv::Mat img = cv::Mat::zeros(width,height,CV_8UC3); 
	int imgSize = img.total()*img.elemSize(), bytes;
	uchar sockData[imgSize];
	for (int i = 0; i < imgSize; i += bytes) {
		if ((bytes = recv(_socket, sockData +i, imgSize  - i, 0)) == -1)  {
			std::cout <<"recv failed at "<<i<<std::endl;
			//break;
		}
	}
	int ptr=0;
	for (int i = 0;  i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			img.at<cv::Vec3b>(i,j) = cv::Vec3b(sockData[ptr+ 0],sockData[ptr+1],sockData[ptr+2]);
			ptr=ptr+3;
		}
	}
	cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
	return img;
}
cv::Mat GazeboInterface::getImage(){
	return getImage( 480, 640);
}


// Move object
void GazeboInterface::set_linear_velocity(std::vector<double> v){
		send_message(_socket,"SET_LIN_VEL");
		rcv_msg(_socket);
		send_message(_socket,vect_to_string(v));
}
void GazeboInterface::set_angular_velocity(std::vector<double> v){
		send_message(_socket,"SET_ANG_VEL");
		rcv_msg(_socket);
		send_message(_socket,vect_to_string(v));
}

// Get ground truth
double GazeboInterface::get_true_yaw(){
	send_message(_socket,"GET_TRUE_YAW");
	return string_to_vect3d(rcv_msg(_socket)).at(0);
}

// Change frame of reference
void GazeboInterface::set_object_frame(){
	send_message(_socket,"SET_FRAME");
	rcv_msg(_socket);
	send_message(_socket,"OBJECT");
}
void GazeboInterface::set_world_frame(){
	send_message(_socket,"SET_FRAME");
	rcv_msg(_socket);
	send_message(_socket,"WORLD");
}


int GazeboInterface::init_server(){
	/*
	 * Connection boilerplate code
	 */
	int server_fd, new_socket, valread, bytes; 
	struct sockaddr_in address; 
	int opt = 1; 
	int addrlen = sizeof(address); 
	// Creating socket file descriptor 
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
	{ 
		perror("socket failed"); 
		exit(EXIT_FAILURE); 
	} 
	// Forcefully attaching socket to the port 8080 
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT | TCP_NODELAY, &opt, sizeof(opt))) { 
		perror("setsockopt"); 
		exit(EXIT_FAILURE); 
	} 
	address.sin_family = AF_INET; 
	address.sin_addr.s_addr = INADDR_ANY; 
	address.sin_port = htons( PORT ); 
	// Forcefully attaching socket to the port  
	if (bind(server_fd, (struct sockaddr *)&address, 
								sizeof(address))<0) { 
		perror("bind failed"); 
		exit(EXIT_FAILURE); 
	} 
	if (listen(server_fd, 3) < 0) { 
		perror("listen"); 
		exit(EXIT_FAILURE); 
	} 
	if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
					(socklen_t*)&addrlen))<0) { 
		perror("accept"); 
		exit(EXIT_FAILURE); 
	} 
	return new_socket;
}
