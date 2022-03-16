// PC with simulator
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <string.h>
#include <iostream>
#include <sys/wait.h>
#include <sys/types.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "net_protocol.cpp"
#include "gazebo_interface.hpp"

#define MAX_TIME_DIFF 1000000

long long timestamp_diff(timestamp_t t1, timestamp_t t2);
timestamp_t chrono_to_timestamp( std::chrono::duration<long int, std::ratio<1, 1000000000> > t);

void send_image(int sock, cv::Mat img){
	std::cout << "Sending image" << std::endl;
	img = img.reshape(0,1);
	int imgSize = img.total()*img.elemSize();
	send(sock , img.data , imgSize , 0 );
}



int main(int argc, char const *argv[]) {
	/*
	 * Connection boilerplate code
	 */

	int sock = 0;
	struct sockaddr_in serv_addr;
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("\n Socket creation error \n");
		return -1;
	}
	int i = 1;
	setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (void *)&i, sizeof(i));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) {
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}
	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
		printf("\nConnection Failed \n");
		return -1;
	}

 
 
	/*
	 * Main loop
	 */
	std::string input= rcv_msg(sock);
    auto start = std::chrono::steady_clock::now();
	auto end = start;
	long long acc = 0;

	GazeboInterface gz;
	while(input.compare("QUIT")){
		if(!input.compare("INIT")){
			gz.init(!rcv_msg(sock).compare("VERBOSE"),rcv_msg(sock));
		} else if(!input.compare("GET_ANGLE")){
			send_message(sock, std::to_string(gz.get_angle()));
		} else if(!input.compare("GET_DISTANCE")){
			send_message(sock, std::to_string(gz.get_distance()) );
		} else if(!input.compare("GET_IMU")){
			send_message(sock, vect_to_string(gz.get_imu()));
		} else if(!input.compare("GET_IMG")){
			gz.pause_simulation();
			send_image(sock,gz.getImage());
			// Wait ack
			wait_ack(sock);
			gz.resume_simulation();
		} else if(!input.compare("SET_LIN_VEL")){
			send_message(sock, "0");
			gz.set_linear_velocity(string_to_vect3d(rcv_msg(sock)));
		} else if(!input.compare("SET_ANG_VEL")){
			send_message(sock, "0");
			gz.set_angular_velocity(string_to_vect3d(rcv_msg(sock)));
		} else if(!input.compare("GET_TRUE_YAW")){
			send_message(sock, std::to_string(gz.get_true_yaw()));
		} else if(!input.compare("SET_FRAME")){
			send_message(sock, "0");
			if(!rcv_msg(sock).compare("WORLD")){
				gz.set_world_frame();
			} else {
				gz.set_object_frame();
			}
		}
		input = rcv_msg(sock);
		gz.resume_simulation();

		// Makes the server wait until the simulation catches up with real time
		end = std::chrono::steady_clock::now();
		if(argc > 1 && timestamp_diff(gz.get_sim_time(),chrono_to_timestamp(end - start)) < acc-MAX_TIME_DIFF){
			while(timestamp_diff(gz.get_sim_time(),chrono_to_timestamp(end - start)) < acc-MAX_TIME_DIFF){ }
			acc += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
			start = std::chrono::steady_clock::now();
			end = start;
			std::cout << "CATCHED UP" << std::endl;
		}
	}




	gz.quit_listener();
	std::cout << "Total simulation run time:  "<< gz.get_imu().at(0) << std::endl;
	return 0;
 
}

timestamp_t chrono_to_timestamp( std::chrono::duration<long int,std::ratio<1, 1000000000>> t){
	timestamp_t timestamp;
	timestamp.sec  = std::chrono::duration_cast<std::chrono::seconds>(t).count();
	timestamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(t).count()-std::chrono::duration_cast<std::chrono::seconds>(t).count()*1000000000;
	return timestamp;
}

long long timestamp_diff(timestamp_t t1, timestamp_t t2){
	return (t1.nsec - t2.nsec) + (t1.sec-t2.sec)*1000000000;
}
