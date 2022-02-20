#ifndef __positioning_H__
#define __positioning_H__

#include "gazebo_interface.hpp"

class Positioning{
	private:
		double yaw;
		GazeboInterface *hw;

	public:
		Positioning(GazeboInterface *gz);
		std::vector<double> position;
		double get_yaw();
		void update_yaw(double rotation);
		void update_position(std::vector<double> movement, double direction);
};

#endif
