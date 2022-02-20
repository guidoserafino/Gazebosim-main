#include <ctype.h>
#include <iostream>
#include "known_objects.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video/tracking.hpp>
#include "tracker.hpp"

Tracker::Tracker(){
	backprojMode = false;
	selectObject = false;
	showHist = true;
	vmin = 10; 
	vmax = 256;
	smin = 30;
	hsize = 16;
	hranges[0]=0; hranges[1]=180;
	phranges = hranges;
	histimg = cv::Mat::zeros(200,320,CV_8UC3);

}
void Tracker::init( target_object obj){
	cv::Mat image;
	std::string filename;
	switch(obj){
		case EXIT_SIGN:
			filename = "emergency_exit.png";
			break;
		case FIRE_EXTINGUISHER:
			filename = "fire_extinguisher.png";
			break;
		default:
			break;
	}
	image = cv::imread("../../imgs/"+filename);
	//cv::imshow("frame", image);
	//cv::waitKey(0);
	init(image, cv::Rect2d(0,0, image.cols, image.rows));
	std::cout<<"tracker initialized"<<std::endl;
}

void Tracker::init( cv::Mat frame, cv::Rect object_roi ) {
// User draws box around object to track. This triggers CAMShift to start tracking
	std::cout<<object_roi<<std::endl;
	cv::Mat image;
	cv::createTrackbar( "Vmin", "frame", &vmin, 256, 0 );
	cv::createTrackbar( "Vmax", "frame", &vmax, 256, 0 );
	cv::createTrackbar( "Smin", "frame", &smin, 256, 0 );
	selection = object_roi;
	frame.copyTo(image);
	cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
	int _vmin = vmin, _vmax = vmax;
	cv::inRange(hsv, cv::Scalar(0, smin, MIN(_vmin,_vmax)), cv::Scalar(180, 256, MAX(_vmin, _vmax)), mask);
	int ch[] = {0, 0};
	hue.create(hsv.size(), hsv.depth());
	cv::mixChannels(&hsv, 1, &hue, 1, ch, 1);

	// Object has been selected by user, set up CAMShift search properties
	cv::Mat roi(hue, selection), maskroi(mask, selection);
	calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
	normalize(hist, hist, 0, 255, cv::NORM_MINMAX);
	trackWindow = selection;
	histimg = cv::Scalar::all(0);
	int binW = histimg.cols / hsize;
	cv::Mat buf(1, hsize, CV_8UC3);
	for( int i = 0; i < hsize; i++ ){
		buf.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(i*180./hsize), 255, 255);
	}
	cvtColor(buf, buf, cv::COLOR_HSV2BGR);
	for( int i = 0; i < hsize; i++ ) {
		int val = cv::saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
		rectangle( histimg, cv::Point(i*binW,histimg.rows), cv::Point((i+1)*binW,histimg.rows - val), cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8 );
	}
}

cv::Rect Tracker::update( cv::Mat frame ) {
	cv::Mat image;
	cv::RotatedRect trackBox;
	frame.copyTo(image);
	for(int i = 0; i<5 ; i++){
		cvtColor(image, hsv, cv::COLOR_BGR2HSV);
		int _vmin = vmin, _vmax = vmax;
		inRange(hsv, cv::Scalar(0, smin, MIN(_vmin,_vmax)), cv::Scalar(180, 256, MAX(_vmin, _vmax)), mask);
		int ch[] = {0, 0};
		hue.create(hsv.size(), hsv.depth());
		mixChannels(&hsv, 1, &hue, 1, ch, 1);
		// Perform CAMShift
		calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
		backproj &= mask;
		trackBox = CamShift(backproj, trackWindow, cv::TermCriteria( cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1 ));
		if( trackWindow.area() <= 1 ) {
			std::cout << "Area <= 1" << std::endl;
			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
			trackWindow = cv::Rect(trackWindow.x - r, trackWindow.y - r,
					trackWindow.x + r, trackWindow.y + r)
				& cv::Rect(0, 0, cols, rows);
		}
	}
	return trackBox.boundingRect();
}
