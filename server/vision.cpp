#include <opencv2/opencv.hpp>
#include "tracker.hpp"
#include "vision.hpp"
#include "known_objects.hpp"
#include "gazebo_interface.hpp"


// In all these functions we assume the camera distorsion is negligible and do a first order linear approximation
Vision::Vision(GazeboInterface *gz){
	hw = gz;
	tracker = new Tracker();
}

double Vision::obj_dist(double real_height){
	return obj_dist(p1, p2, real_height);
}
double Vision::obj_dist(cv::Point p1, cv::Point p2, double real_height){
	cv::Point diff = p1-p2;
	double pixel_height = sqrt(diff.x*diff.x + diff.y*diff.y);
	return (real_height*FOCAL_LENGTH*IMAGE_HEIGHT)/(pixel_height*SENSOR_HEIGHT);
}

double Vision::obj_height(double real_distance){
	return obj_height(p1,p2,real_distance);
}
double Vision::obj_height(cv::Point p1, cv::Point p2, double real_distance){
	cv::Point diff = p1-p2;
	double pixel_height = abs(diff.y);
	return (real_distance*pixel_height*SENSOR_HEIGHT)/(FOCAL_LENGTH*IMAGE_HEIGHT);
}

double Vision::obj_width(double real_distance){
	return obj_width(p1,p2,real_distance);
}
double Vision::obj_width(cv::Point p1, cv::Point p2, double real_distance){
	cv::Point diff = p1-p2;
	double pixel_height = abs(diff.x);
	return (real_distance*pixel_height*SENSOR_WIDTH)/(FOCAL_LENGTH*IMAGE_WIDTH);
}

cv::Vec3f Vision::point_polar_coords(cv::Point p, double rho){
	//ISO 80000-2:2019 convention: radial distance r, polar angle θ (theta), and azimuthal angle φ (phi). The symbol ρ (rho) is often used instead of r.
	// Phi here is the elevation angle (i.e. measured from the reference plane), not the inclination angle (measured from the Z axis).
	double theta = ((double)p.x/IMAGE_WIDTH-0.5)*HORIZONTAL_FOV;
	double phi = -((double)p.y/IMAGE_HEIGHT-0.5)*VERTICAL_FOV;
	return cv::Vec3f(rho, theta, phi);
}

cv::Vec3f Vision::point_coords(cv::Point p, double rho){
	cv::Point2d img_center = cv::Point2d(IMAGE_WIDTH/2,IMAGE_HEIGHT/2);
	double x = obj_width(p,img_center,rho)*(p.x<img_center.x?-1:1);
	double z = obj_height(p,img_center,rho)*(p.y>img_center.y?-1:1);
	double y = sqrt(pow(rho,2)-pow(x,2)-pow(z,2));
	return cv::Vec3f(x,y,z);
}

cv::Vec3f Vision::point_cylindrical_coords(cv::Point p, double distance){
	//The ISO standard 31-11 recommends (ρ, φ, z), where ρ is the radial coordinate, φ the azimuth, and z the height.
	cv::Point2d img_center = cv::Point2d(IMAGE_WIDTH/2,IMAGE_HEIGHT/2);
	double z = obj_height(p,img_center,distance)*(p.y>img_center.y?-1:1);
	double phi = ((double)p.x/IMAGE_WIDTH-0.5)*HORIZONTAL_FOV;
	double rho = sqrt(pow(distance,2)-pow(z,2));
	return cv::Vec3f(rho, phi, z);
}

int Vision::known_object_roi(target_object obj){
	tracker->init(obj);
}

int Vision::init_object_roi(){
	cv::Rect2d roi = cv::selectROI("frame",current_frame,false);
	if(roi.area()!=0){
		tracker->init(current_frame, roi);
		current_roi = roi;
		return 1;
	} 
	return 0;
}

int Vision::update_object_roi(){
	current_roi = tracker->update(current_frame);
	p1 = cv::Point(current_roi.x,current_roi.y);
	p2 = cv::Point(current_roi.x,current_roi.y+current_roi.height);
	if(current_roi.area()==0){
		return 0;
	}
	return 1;
}

void Vision::toggle_roi(){
	show_roi = !show_roi;
}
void Vision::update_frame(){
	current_frame = hw->getImage();
	if(show_image){
		if(show_roi){
			cv::Mat frame_copy = current_frame.clone();
			cv::rectangle( frame_copy, current_roi, cv::Scalar( 255, 0, 0 ), 2, 1 );
			// Show ROI and image center
			cv::circle(frame_copy, roi_center(current_roi), 2, cv::Scalar(0,255,0), -1);
			cv::circle(frame_copy, cv::Point2d(IMAGE_WIDTH/2,IMAGE_HEIGHT/2), 2, cv::Scalar(255,0,0), -1);
			cv::imshow("frame",frame_copy);
		} else {
			cv::imshow("frame",current_frame);
		}
	} 
	
}

cv::Mat Vision::get_frame(){
	return current_frame;
}

cv::Rect2d Vision::get_roi(){
	return current_roi;
}
cv::Point Vision::roi_center(cv::Rect roi){
	return (roi.br() + roi.tl())/2;
}

