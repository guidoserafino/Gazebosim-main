#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/ocl.hpp>
#include "gazebo_interface.hpp"
#include "vision.hpp"
#include "movement_primitives.hpp"
#include "known_objects.hpp"
#include <unistd.h>

#define KNOWN_SIZE 270

	int main(int argc, char **argv){
	GazeboInterface hw;
	int exit=0;
	hw.init_listener(false, "");
	Vision vision(&hw);
	Movement mv(&hw, &vision);
	cv::Point2d img_center = cv::Point2d(IMAGE_WIDTH/2,IMAGE_HEIGHT/2);
	cv::Mat frame;
	cv::Rect2d roi	;
	int valid_roi = 1, fix_size = 0;
	double item_size = 0;
	double estimated_obj_distance = 0;
	unsigned int range_finder_obj_distance;
	char input = ' ';

	vision.known_object_roi(EXIT_SIGN);
	
	/* 
	    auto start2 = std::chrono::steady_clock::now();
		for(int i = 0; i<100 ; i++){
			hw.get_distance();	
		}
	    auto end2 = std::chrono::steady_clock::now();
		std::cout << "Time for distance: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - start2).count()/100 << std::endl;
	    auto start1 = std::chrono::steady_clock::now();
		for(int i = 0; i<100 ; i++){
			hw.getImage();	
		}
	    auto end1 = std::chrono::steady_clock::now();
		std::cout << "Time for image:    " << std::chrono::duration_cast<std::chrono::nanoseconds>(end1 - start1).count()/100 << std::endl;
		hw.quit_listener();
		return 0;
	*/

	if(argc > 1){
		vision.update_frame();
		vision.update_object_roi();
		roi = vision.get_roi();
		estimated_obj_distance = vision.obj_dist(200);
		cv::Point roi_middle = Vision::roi_center(roi);
		cv::Vec3f start = vision.point_coords(roi_middle, estimated_obj_distance);
		mv.manual_navigation('.');
		vision.update_frame();
		vision.update_object_roi();
		roi = vision.get_roi();
		estimated_obj_distance = vision.obj_dist(200);
		roi_middle = Vision::roi_center(roi);
		cv::Vec3f end = vision.point_coords(roi_middle, estimated_obj_distance) ;
		std::cout << end-start << std::endl;
		hw.quit_listener();
		return 0;
	}

	while(input != 27 and ! exit){
		vision.update_frame();
		// Update sensor data
		frame = vision.get_frame();
		range_finder_obj_distance = hw.get_distance();
			

		// Reset/fix measurements
		if(input=='x'){
			item_size=0;
			valid_roi = 0;
		} else if(input == '~'){
			fix_size = ! fix_size;
		} 
		if(input=='v'){
			vision.toggle_roi();
		}
		
			
			// Select a new region of interest and perform image processing
		if(input=='p'||valid_roi){
			// Select and display ROI
			if(! valid_roi){
				valid_roi = vision.init_object_roi();
				exit = 1;
			} else {
				valid_roi = vision.update_object_roi();
			}
			roi = vision.get_roi();

			cv::Point roi_middle = Vision::roi_center(roi);
			// Get object distance estimate
			cv::Point p1 = cv::Point(roi.x,roi.y);
			cv::Point p2 = cv::Point(roi.x,roi.y+roi.height);
			estimated_obj_distance = vision.obj_dist(p1,p2,item_size);

			if(range_finder_obj_distance <= 4000 && roi.height!=0 && !fix_size) {
				// Update object size estimate
				double new_measurement =  vision.obj_height(p1,p2,range_finder_obj_distance);
				if(item_size == 0){
					item_size = new_measurement;
				} else {
					item_size = (3*item_size+new_measurement)/4;
				}
			}

			// Print computation results
			std::cout << "------------------------------------------------------------------------"   << std::endl;
			std::cout << "Measured object distance (range):                                      "   << range_finder_obj_distance <<std::endl;
			if(roi.height>1 && roi.width > 1){
				std::cout << "Estimated object size (image + range):                                 "   << item_size <<std::endl;
				std::cout << "Estimated object distance (image + estimated size):                    "   << estimated_obj_distance <<std::endl;
				std::cout << "Estimated object distance with known size (image + ground truth size): "   << vision.obj_dist(p1,p2,KNOWN_SIZE) <<std::endl;
				std::cout << "Difference in distance from ground truth:                              "   << 
					(range_finder_obj_distance-estimated_obj_distance)/range_finder_obj_distance*100 << "%" <<std::endl;
				std::cout << "Difference in size from ground truth:                                  "   << (KNOWN_SIZE-item_size)/KNOWN_SIZE*100 << "%" <<std::endl;
				std::cout << "----------- coordinates-------------------------------------------------"   << std::endl;
				std::cout << "Object coordinates (euclidean):            "
					<< vision.point_coords(roi_middle, estimated_obj_distance) << std::endl;
				std::cout << "Object coordinates (polar):                "
					<< vision.point_polar_coords(roi_middle, estimated_obj_distance) << std::endl;
				std::cout << "Object coordinates (cylindrical):          "
					<< vision.point_cylindrical_coords(roi_middle, estimated_obj_distance ) << std::endl << std::endl;
			} 

			std::cout << "Required time:  " << hw.get_imu().at(0) << std::endl;
		} 
		input = cv::waitKey(0);
		mv.manual_navigation(input);
	}
	hw.quit_listener();
	return 0;
}
