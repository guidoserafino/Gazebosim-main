#ifndef __OBJECT_TRACKER_H__
#define __OBJECT_TRACKER_H__

#include <opencv2/opencv.hpp>
#include "known_objects.hpp"

class Tracker {
public:
	Tracker ();
	void init( target_object obj);
	void init( cv::Mat frame, cv::Rect object_roi );
	cv::Rect update( cv::Mat frame );

private:
	/* data */
	bool backprojMode;
	bool selectObject;
	bool showHist;
	cv::Point origin;
	cv::Rect selection;
	int vmin, vmax, smin;
	cv::Rect trackWindow;
	int hsize;
	const float* phranges = hranges;
	cv::Mat hsv, hue, mask, hist, histimg, backproj;
	float hranges[2];
};


#endif
