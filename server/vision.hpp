#ifndef __VISION_H__
#define __VISION_H__

#include <opencv2/opencv.hpp>
#include "known_objects.hpp"
#include "gazebo_interface.hpp"
#include "tracker.hpp"

#define FOCAL_LENGTH 3.6
#define PIXEL_SIZE 0.0014
#define SENSOR_HEIGHT 2.74
#define SENSOR_WIDTH 3.76
#define IMAGE_HEIGHT 480
#define IMAGE_WIDTH 640
#define HORIZONTAL_FOV 53.5
#define VERTICAL_FOV 41.41

class Vision {
public:
	Vision (GazeboInterface * gz);

	double obj_dist(double real_height);
	double obj_height(double real_distance);
	double obj_width(double real_distance);
	double obj_dist(cv::Point p1, cv::Point p2, double real_height);
	double obj_height(cv::Point p1, cv::Point p2, double real_distance);
	double obj_width(cv::Point p1, cv::Point p2, double real_distance);
	cv::Vec3f point_polar_coords(cv::Point p, double rho);
	cv::Vec3f point_coords(cv::Point p, double rho);
	cv::Vec3f point_cylindrical_coords(cv::Point p, double distance);
	int known_object_roi(target_object obj);
	int init_object_roi();
	int update_object_roi();
	void toggle_roi();
	void update_frame();
	cv::Mat get_frame();
	cv::Rect2d get_roi();
	static cv::Point roi_center(cv::Rect roi);


private:
	/* data */
	Tracker *tracker;
	cv::Point p1, p2;
	cv::Rect2d current_roi = cv::Rect2d(0,0,0,0);
	GazeboInterface *hw;
	cv::Mat current_frame;
	int show_roi = 1,
		show_image = 1;
};

#endif
