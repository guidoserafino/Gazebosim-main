#ifndef __MOVEMENT_PRIMITIVES_H__
#define __MOVEMENT_PRIMITIVES_H__

#include "gazebo_interface.hpp"
#include "positioning.hpp"
#include "vision.hpp"

class Movement {
public:
	Movement (GazeboInterface * gz, Vision * v);

private:
	/* data */
	double speed;
	double angular_velocity;
	Positioning *pos;
	GazeboInterface *hw;
	Vision *vision;
public:
	void linear_translation(float distance,float movement_direction, float precision);
	void horizontal_centering(double tollerance);
	void relative_linear_translation(double direction, double distance, double height);
	void adjust_distance(double target_distance, double object_height, double precision_m);
	void relative_translation(double max_object_offcentering, double movement_direction, double end_angle, double end_angle_precision);
	void circle(int segments, int stop, double max_object_offcentering, double single_segment_precision);
	void manual_navigation(char command);
	void face_surface();
	void rotate(double rotation, double precision);
	void stop();
};

#endif	
