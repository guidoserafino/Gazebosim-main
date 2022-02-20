#ifndef __GAZEBO_INTERFACE__
#define __GAZEBO_INTERFACE__

#include <opencv2/opencv.hpp>

class GazeboInterface {
private:
	/* data */
	int _socket;
	/* methods*/
	int init_server();
	cv::Mat getImage(int width, int height);

public:
	GazeboInterface ();

	// Boilerplate
	void init_listener(bool verbose, std::string suffix);
	void quit_listener();

	// Obtain sensor data
	std::vector<double> get_imu();
	double get_angle();
	unsigned int get_distance();
	cv::Mat getImage();

	// Move object
	void set_linear_velocity(std::vector<double> v);
	void set_angular_velocity(std::vector<double> v);

	// Get ground truth
	double get_true_yaw();

	// Change frame of reference
	void set_object_frame();
	void set_world_frame();
};
#endif
