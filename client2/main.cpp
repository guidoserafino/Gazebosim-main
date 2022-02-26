// PC with simulator
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <string.h>
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
	

  int sock_two = 0;
	struct sockaddr_in serv_addr_two;
	if ((sock_two = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("\n Socket creation error \n");
		return -1;
	}
	int j = 1;
	setsockopt(sock_two, IPPROTO_TCP, TCP_NODELAY, (void *)&j, sizeof(j));
	serv_addr_two.sin_family = AF_INET;
	serv_addr_two.sin_port = htons(PORT);
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, "127.0.0.1", &serv_addr_two.sin_addr)<=0) {
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}
	if (connect(sock_two, (struct sockaddr *)&serv_addr_two, sizeof(serv_addr_two)) < 0) {
		printf("\nConnection Failed \n");
		return -1;
	}

 
	/*
	 * Main loop
	 */
	
	std::string input_two = rcv_msg(sock_two);
	

	  auto start_two = std::chrono::steady_clock::now();
  auto end_two = start_two;
  long long acc_two = 0;



	GazeboInterface gz;
	

	while(input_two.compare("QUIT")){
		if(!input_two.compare("INIT")){
			gz.init(!rcv_msg(sock_two).compare("VERBOSE"),rcv_msg(sock_two));
		} else if(!input_two.compare("GET_ANGLE")){
			send_message(sock_two, std::to_string(gz.get_angle()));
		} else if(!input_two.compare("GET_DISTANCE")){
			send_message(sock_two, std::to_string(gz.get_distance()) );
		} else if(!input_two.compare("GET_IMU")){
			send_message(sock_two, vect_to_string(gz.get_imu()));
		} else if(!input_two.compare("GET_IMG")){
			gz.pause_simulation();
			send_image(sock_two,gz.getImage());
			// Wait ack
			wait_ack(sock_two);
			gz.resume_simulation();
		} else if(!input_two.compare("SET_LIN_VEL")){
			send_message(sock_two, "0");
			gz.set_linear_velocity(string_to_vect3d(rcv_msg(sock_two)));
		} else if(!input_two.compare("SET_ANG_VEL")){
			send_message(sock_two, "0");
			gz.set_angular_velocity(string_to_vect3d(rcv_msg(sock_two)));
		} else if(!input_two.compare("GET_TRUE_YAW")){
			send_message(sock_two, std::to_string(gz.get_true_yaw()));
		} else if(!input_two.compare("SET_FRAME")){
			send_message(sock_two, "0");
			if(!rcv_msg(sock_two).compare("WORLD")){
				gz.set_world_frame();
			} else {
				gz.set_object_frame();
			}
		}
		input_two = rcv_msg(sock_two);
		gz.resume_simulation();

		// Makes the server wait until the simulation catches up with real time
		end_two = std::chrono::steady_clock::now();
		if(argc > 1 && timestamp_diff(gz.get_sim_time(),chrono_to_timestamp(end_two - start_two)) < acc_two-MAX_TIME_DIFF){
			while(timestamp_diff(gz.get_sim_time(),chrono_to_timestamp(end_two - start_two)) < acc_two-MAX_TIME_DIFF){ }
			acc_two+= std::chrono::duration_cast<std::chrono::nanoseconds>(end_two - start_two).count();
			start_two = std::chrono::steady_clock::now();
			end_two = start_two;
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