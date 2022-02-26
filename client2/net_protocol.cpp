#include <unistd.h> 
#include <chrono>
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/tcp.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <sstream>
#include <iterator>
#include <iostream>

#define PORT 1112 

std::string rcv_msg(int sock){
	std::cout << "Receiving msg size" << std::endl;
	uint32_t data_length, rcvDataLength=0;
	recv(sock,&rcvDataLength,sizeof(uint32_t),0); 	// Receive the message length
	data_length = ntohl(rcvDataLength);				// Ensure correct byte order
	char rcvBuf[data_length];    					// Allocate a receive buffer
	std::cout << "Msg size:  " <<  data_length << std::endl;
	std::cout << "Receiving msg" << std::endl;
	recv(sock,rcvBuf,data_length,0);	 			// Receive string data
	std::string received_string;                    // assign data to a string
	received_string.assign(rcvBuf,data_length);
	std::cout << "Received msg: '"<<received_string <<"'"<< std::endl;
	std::cout << "--------------------" << std::endl;
	return received_string;
}

void send_message(int sock, std::string msg){
	uint32_t data_length = htonl(msg.size()); 		// Ensure network byte order
	send(sock,&data_length ,sizeof(uint32_t), 0);	// Send the data length
	send(sock,msg.c_str(),msg.size(), 0);			// Send the string data
	std::cout<< "Sent: " << msg << std::endl;
}

std::string vect_to_string(std::vector<double> vec){
	std::ostringstream oss;
	if (!vec.empty()) {
		// Convert all but the last element to avoid a trailing ","
		std::copy(vec.begin(), vec.end()-1, std::ostream_iterator<double>(oss, ","));
		// Now add the last element with no delimiter
		oss << vec.back();
	}
	return oss.str();
}

std::vector<double> string_to_vect3d(std::string str){
	std::vector<double> values;
	std::string token;
	std::stringstream tokenStream(str);
	while (std::getline(tokenStream, token,','))
	{
		values.push_back(stod(token));
	}
	return values;
}

void wait_ack(int sock_fd){
	tcp_info info;
	socklen_t tcp_info_len = sizeof(tcp_info);
    do {
		getsockopt(sock_fd,SOL_TCP, TCP_INFO, (void *) &info, &tcp_info_len);
		//wait till all packets acknowledged 
    } while (info.tcpi_unacked > 0);
}
