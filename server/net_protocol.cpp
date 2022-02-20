#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <sstream>
#include <iterator>
#include <iostream>

#define PORT 1112 

std::string rcv_msg(int sock){
	uint32_t data_length, rcvDataLength=0;
	recv(sock,&rcvDataLength,sizeof(uint32_t),0); 	// Receive the message length
	data_length = ntohl(rcvDataLength);				// Ensure correct byte order
	char rcvBuf[data_length];	    				// Allocate a receive buffer
	recv(sock,rcvBuf,data_length,0);	 			// Receive string data
	std::string received_string;                    // assign data to a string
	received_string.assign(rcvBuf,data_length);
	//std::cout << received_string << std::endl;
	return received_string;
}

void send_message(int sock, std::string msg){
	uint32_t data_length = htonl(msg.size()); 		// Ensure network byte order
	send(sock,&data_length ,sizeof(uint32_t), 0); 	// Send the data length
	send(sock,msg.c_str(),msg.size(), 0); 			// Send the string data
	//std::cout << msg << std::endl;
}

std::string vect_to_string(std::vector<double> vec){
	std::string s = "";
	for(int i = 0; i<vec.size() ; i++){
		s+= std::to_string(vec.at(i));
		s+=", ";
	}
	s+= std::to_string(vec.back());
	return s;
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
