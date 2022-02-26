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
	

  int sock_three = 0;
	struct sockaddr_in serv_addr_three;
	if ((sock_three = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("\n Socket creation error \n");
		return -1;
	}
	int k = 1;
	setsockopt(sock_three, IPPROTO_TCP, TCP_NODELAY, (void *)&k, sizeof(k));
	serv_addr_three.sin_family = AF_INET;
	serv_addr_three.sin_port = htons(PORT);
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, "127.0.0.1", &serv_addr_three.sin_addr)<=0) {
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}
	if (connect(sock_three, (struct sockaddr *)&serv_addr_three, sizeof(serv_addr_three)) < 0) {
		printf("\nConnection Failed \n");
		return -1;
	}
	/*
	 * Main loop
	 */
	
	std::string input_three = rcv_msg(sock_three);
    

    auto start_three = std::chrono::steady_clock::now();
  auto end_three = start_three;
  long long acc_three = 0;

	GazeboInterface gz;


	while(input_three.compare("QUIT")){
		if(!input_three.compare("INIT")){
			gz.init(!rcv_msg(sock_three).compare("VERBOSE"),rcv_msg(sock_three));
		} else if(!input_three.compare("GET_ANGLE")){
			send_message(sock_three, std::to_string(gz.get_angle()));
		} else if(!input_three.compare("GET_DISTANCE")){
			send_message(sock_three, std::to_string(gz.get_distance()) );
		} else if(!input_three.compare("GET_IMU")){
			send_message(sock_three, vect_to_string(gz.get_imu()));
		} else if(!input_three.compare("GET_IMG")){
			gz.pause_simulation();
			send_image(sock_three,gz.getImage());
			// Wait ack
			wait_ack(sock_three);
			gz.resume_simulation();
		} else if(!input_three.compare("SET_LIN_VEL")){
			send_message(sock_three, "0");
			gz.set_linear_velocity(string_to_vect3d(rcv_msg(sock_three)));
		} else if(!input_three.compare("SET_ANG_VEL")){
			send_message(sock_three, "0");
			gz.set_angular_velocity(string_to_vect3d(rcv_msg(sock_three)));
		} else if(!input_three.compare("GET_TRUE_YAW")){
			send_message(sock_three, std::to_string(gz.get_true_yaw()));
		} else if(!input_three.compare("SET_FRAME")){
			send_message(sock_three, "0");
			if(!rcv_msg(sock_three).compare("WORLD")){
				gz.set_world_frame();
			} else {
				gz.set_object_frame();
			}
		}
		input_three = rcv_msg(sock_three);
		gz.resume_simulation();

		// Makes the server wait until the simulation catches up with real time
		end_three = std::chrono::steady_clock::now();
		if(argc > 1 && timestamp_diff(gz.get_sim_time(),chrono_to_timestamp(end_three - start_three)) < acc_three-MAX_TIME_DIFF){
			while(timestamp_diff(gz.get_sim_time(),chrono_to_timestamp(end_three - start_three)) < acc_three-MAX_TIME_DIFF){ }
			acc_three+= std::chrono::duration_cast<std::chrono::nanoseconds>(end_three - start_three).count();
			start_three = std::chrono::steady_clock::now();
			end_three = start_three;
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
