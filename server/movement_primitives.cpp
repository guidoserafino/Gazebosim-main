#include "gazebo_interface.hpp"
#include "positioning.hpp"
#include "movement_primitives.hpp"
#include <numeric>
#include <opencv2/opencv.hpp>
#include "vision.hpp"



Movement::Movement(GazeboInterface *gz, Vision *v){
	hw = gz;
	vision = v;
	pos = new Positioning(hw);
	speed=0.2;
	angular_velocity = 0.2;
}

void Movement::stop(){
	hw->set_linear_velocity({0,0,0});
	hw->set_angular_velocity({0,0,0});
}
// Use keyboard input to move the object

void Movement::manual_navigation(char command){
	switch(command){
		case 'w':
			hw->set_linear_velocity({speed,0,0});
			break;
		case 's':
			hw->set_linear_velocity({-speed,0,0});
			break;
		case 'd':
			hw->set_linear_velocity({0,-speed,0});
			break;
		case 'a':
			hw->set_linear_velocity({0,speed,0});
			break;
		case 'j':
			hw->set_linear_velocity({0,0,-speed});
			break;
		case 'k':
			hw->set_linear_velocity({0, 0, speed});
			break;
		case 'q':
			hw->set_angular_velocity({0,0,0.4});
			break;
		case 'e':
			hw->set_angular_velocity({0,0,-0.4});
			break;
		case 'X':
			speed-=0.5;
			break;
		case 'A':
			speed+=0.5;
			break;
		case 'l':
			hw->set_object_frame();
			break;
		case 'h':
			hw->set_world_frame();
			break;
		case '=':
			horizontal_centering(5);
			break;
		case '\\':
			relative_translation(3,0,-90, 4);
			break;
		case '/':
			relative_translation(3,0,90, 6);
			break;
		case '.':
			linear_translation(3,  0,0.2);
			linear_translation(3, 90,0.2);
			linear_translation(3,180,0.2);
			linear_translation(3,270,0.2);
			break;
		case '(':
			circle(8,-8,3,5);
		case '-':
			face_surface();
			break;
		case '>':
			rotate(90,5);
			break;
		case '<':
			rotate(-90,5);
			break;
		case '1':
			adjust_distance(1, 0.2, 0.1);
			break;


		case ' ':
			stop();
			break;
		default:
			break;
	}
}

void Movement::circle(int segments, int stop, double max_object_offcentering, double single_segment_precision){
	/*
	 * segments:					How many segments to approximate the circumference with.
	 * stop:						After which segment to stop the rotation.
	 * 									$ 360/segments*stop = angle $
	 * max_object_offcentering:		How much can the tracked object move away from the center of the image before readjusting
	 * single_segment_precision:	In each segment, by how much the end angle can be off by.
	 * 									$ sin(single_segment_precision)*radius = max lenght error in each segment $ .
	 */
	double used_speed;
	double single_segment_rotation = -360.0/segments;
	double movement_direction = -(180-360.0/segments)/2;
	if(stop < 0){
		// clockwise rotation
		stop *=-1;
		used_speed *=-1;
		single_segment_rotation *=-1;
		movement_direction *=-1;
	}
	for(int i = 0; i<stop ; i++){
		relative_translation(max_object_offcentering, movement_direction, single_segment_rotation, single_segment_precision);
	}
}

void Movement::relative_linear_translation(double distance, double direction, double height){
	stop();

	direction = fmod(direction,360);
	direction = direction >  180 ? direction-360 : direction;
	direction = direction < -180 ? direction+360 : direction;

	double starting_angle = pos->get_yaw(), end_angle;
	double starting_distance, end_distance;
	horizontal_centering(5);
	double offset = pos->get_yaw() - starting_angle;

	vision->update_frame();
	vision->update_object_roi();
	cv::Rect2d roi = vision->get_roi();
	cv::Point p1 = cv::Point(roi.x,roi.y);
	cv::Point p2 = cv::Point(roi.x,roi.y+roi.height);
	starting_distance = vision->obj_dist(p1, p2, height)/1000;

	end_distance = sqrt(pow(distance,2)+pow(starting_distance,2)-2*distance*starting_distance*cos(M_PI/180*(direction-offset)));
	end_angle 	 = 180/M_PI*acos( (pow(end_distance,2)+pow(distance,2)-pow(starting_distance,2))/(2*distance*end_distance));
	double desired_rotation = -copysign(180-end_angle-direction+offset,direction);
	std::cout << "End distance: " << end_distance << "    Rotation: " << desired_rotation << std::endl;


	relative_translation(5,direction-offset,desired_rotation,5);
	rotate(offset, 5);
}

void Movement::adjust_distance(double target_distance_m, double object_height, double precision_m){
	double starting_angle = pos->get_yaw();
	double current_distance;
	horizontal_centering(5);
	double offset = pos->get_yaw() - starting_angle;
	int framenum=0;

	vision->update_frame();
	vision->update_object_roi();
	cv::Rect2d roi = vision->get_roi();
	cv::Point p1 = cv::Point(roi.x,roi.y);
	cv::Point p2 = cv::Point(roi.x,roi.y+roi.height);
	current_distance = vision->obj_dist(p1, p2, object_height)/1000;

	while(abs(target_distance_m-current_distance) > precision_m){
		vision->update_frame();
		vision->update_object_roi();
		framenum = (framenum+1)%10;
		if(!framenum){
			if(cv::waitKey(1) == ' '){
				break;
			}
		}
		roi = vision->get_roi();
		p1 = cv::Point(roi.x,roi.y);
		p2 = cv::Point(roi.x,roi.y+roi.height);
		current_distance = vision->obj_dist(p1, p2, object_height)/1000;
		std::cout << "current distance: " << current_distance << "   target distance: " << target_distance_m << std::endl;

		hw->set_linear_velocity({copysign(speed, -current_distance+target_distance_m)/*std::min(1.0, abs(target_distance_m-current_distance)/0.2)*/,0,0});
	}

	rotate(offset,5);
}

void Movement::relative_translation(double max_object_offcentering, double movement_direction, double end_angle, double end_angle_precision){
	/*
	 * max_object_offcentering:	How much can the tracked object move away from the center of the image before readjusting
	 * movement_direction: 		The angle relative to the current yaw along wich translation should happen
	 * end_angle: 				The angle, relative to the current yaw, at which the movement should stop.
	 *			 					e.g. if set at 90 the uav will stop once it find itslef perpendicular to its starting
	 *			 					orientation. This cannot exceed +/-180°.
	 * end_angle_precision:		How close to the target angle is deemed acceptable. If this target angle is surpassed,
	 * 								the UAV slows down and goes back, tracking the target with a higher precision.
	 */
	stop();
	if(end_angle > 180 || end_angle < -180){
		return;
	}
	double x_speed = speed*cos(movement_direction*M_PI/180);
	double y_speed = speed*sin(movement_direction*M_PI/180);
	double starting_angle = pos->get_yaw();
	double obj_angle;
	double angle_difference;
	double error;
	int reversed = 0, framenum=0;
	do{
		vision->update_frame();
		vision->update_object_roi();
		obj_angle=vision->point_cylindrical_coords(vision->roi_center(vision->get_roi()),1)[1];
		framenum = (framenum+1)%10;
		if(!framenum){
			if(cv::waitKey(1) == ' '){
				break;
			}
		}
		// if the object is moving too much to the side of the FOV recenter.
		if(abs(obj_angle) > max_object_offcentering){
			horizontal_centering(max_object_offcentering);
			continue;
			//obj_angle=vision->point_cylindrical_coords(vision->roi_center(vision->get_roi()),1)[1];
		}
		angle_difference = pos->get_yaw() - starting_angle ;
		angle_difference = angle_difference >  180 ? angle_difference-360 : angle_difference;
		angle_difference = angle_difference < -180 ? angle_difference+360 : angle_difference;

		error = end_angle - angle_difference;
		error = error >  180 ? error-360 : error;
		error = error < -180 ? error+360 : error;

		/*
		std::cout <<"Starting angle:     "<< starting_angle << std::endl;
		std::cout <<"Current angle:      "<< pos->get_yaw() << std::endl;
		std::cout <<"Angle difference:   "<< angle_difference << std::endl;
		std::cout <<"Error:              "<< error<< std::endl;
		std::cout <<"--------------------"<< std::endl;
		*/

		double vx = x_speed*cos(angle_difference*M_PI/180) - y_speed*sin(angle_difference*M_PI/180);
		double vy = y_speed*cos(angle_difference*M_PI/180) + x_speed*sin(angle_difference*M_PI/180);
		double sign = 1;
		if(error*end_angle < 0){
			std::cout << "Reversing" << std::endl;
			// overshoot, slow down and go back
			if(!reversed){
				x_speed *= 0.8;
				y_speed *= 0.8;
				//max_object_offcentering *= 0.8;
			}
			sign=-1;
			reversed = 1;
		} else {
			if(reversed){
				x_speed *= 0.8;
				y_speed *= 0.8;
				//max_object_offcentering *= 0.8;
			}
			reversed = 0;
		}
		hw->set_linear_velocity({sign*vx,sign*vy,0});
	}while((abs(error) > end_angle_precision ) || abs(obj_angle) > max_object_offcentering);
	//std::cout << hw->get_imu().at(0) << std::endl;
	stop();
}

void Movement::linear_translation(float distance,float movement_direction, float precision){
	stop();
	double x_speed = speed*cos(movement_direction*M_PI/180);
	double y_speed = speed*sin(movement_direction*M_PI/180);
	double distance_traveled_x = 0, distance_traveled_y=0;
	double distance_traveled=0;
	double prev_stamp = 0;
	double prev_speed_x = 0, prev_speed_y=0, speed_x=0, speed_y=0;
	double prev_accel_x = 0, prev_accel_y=0;
	double coeff = 1;
	int framenum=0;
	do{
		framenum = (framenum+1)%100;
		if(!framenum){
			if(cv::waitKey(1) == ' '){
				break;
			}
		}
		std::vector<double> imu = hw->get_imu();
		double timestamp = imu.at(0);
		double accel_x = imu.at(4), accel_y = imu.at(5);
		if(prev_stamp != 0){
			double elapsed_time = timestamp-prev_stamp;
			prev_speed_x = speed_x;
			speed_x += (accel_x+prev_accel_x)/2*elapsed_time;
			distance_traveled_x += (speed_x+prev_speed_x)/2*elapsed_time;

			prev_speed_y = speed_y;
			speed_y += (accel_y+prev_accel_y)/2*elapsed_time;
			distance_traveled_y += (speed_y+prev_speed_y)/2*elapsed_time;

			distance_traveled = sqrt(pow(distance_traveled_x,2)+pow(distance_traveled_y,2));

			coeff = copysign(1, distance-distance_traveled);
		}

		hw->set_linear_velocity({x_speed*coeff,y_speed*coeff,0});
		//std::cout << distance_traveled << std::endl;
		prev_stamp = timestamp;
		prev_accel_x = accel_x;
		prev_accel_y = accel_y;
	}while(abs(distance-distance_traveled)>precision);
	stop();
	//std::cout << "x: " << distance_traveled_x << "   y: " << distance_traveled_y << std::endl;
	pos->update_position({distance_traveled, 0}, movement_direction);
}

void Movement::horizontal_centering(double tolerance){
	/*
	 * tolerance: 	maximum allowable offset of the center of the object ROI from
	 * 					the center of the frame. Measured in degrees
	 */
	stop();
	double angle = 0;
	double curr_angular_velocity = angular_velocity;
	int reversed, framenum=1;
	double prev_stamp = 0;
	double prev_speed = 0;
	vision->update_frame();
	vision->update_object_roi();
	angle=vision->point_cylindrical_coords(vision->roi_center(vision->get_roi()),1)[1];
	reversed = (angle > 0 ? 0 :1);

	//int oscillations = 0;

	while(abs(angle)>tolerance){
		vision->update_frame();
		vision->update_object_roi();
		if(cv::waitKey(1) == ' '){
			break;
		}
		angle=vision->point_cylindrical_coords(vision->roi_center(vision->get_roi()),1)[1];
		hw->set_angular_velocity({0,0,curr_angular_velocity*(angle>0?-1:1)*std::min<double>(1.0, abs(angle/(2*tolerance)))});
		std::vector<double>imu = hw->get_imu();
		if (prev_stamp != 0){
			double elapsed_time = imu.at(0)-prev_stamp;
			double dtheta = elapsed_time*180/M_PI*(imu.at(3)+prev_speed)/2;
			pos->update_yaw(dtheta);
		}
		prev_stamp = imu.at(0);
		prev_speed = imu.at(3);

		if((angle > 0 && reversed) || (angle < 0 && !reversed)){
			//oscillations++;
			reversed = !reversed;
			curr_angular_velocity *= 0.8;
		}
	}
	stop();

	//std::cout << oscillations << ", " << prev_stamp << std::endl;
}

void Movement::face_surface(){
	/*
	 *
	 */
	stop();
	//double starting_angle = get_true_yaw();
	double prev_stamp = 0;
	double prev_speed = 0;
	int current_distance, prev_distance ;
	int delta;
	int switches = 6;
	int framenum = 0;
	double curr_angular_velocity = angular_velocity;
	prev_distance =hw->get_distance();

	hw->set_angular_velocity({0,0,curr_angular_velocity});
	while(switches){
		vision->update_frame();
		framenum = (framenum+1)%10;
		if(!framenum){
			if(cv::waitKey(1) == ' '){
				break;
			}
		}

		std::vector<double>imu = hw->get_imu();
		if (prev_stamp != 0){
			double elapsed_time = imu.at(0)-prev_stamp;
			double dtheta = elapsed_time*180/M_PI*(imu.at(3)+prev_speed)/2;
			pos->update_yaw(dtheta);
		}
		prev_stamp = imu.at(0);
		prev_speed = imu.at(3);

		current_distance = hw->get_distance();
		delta = current_distance-prev_distance;
		prev_distance = current_distance;
		if ( delta > 0) {
			switches--;
			curr_angular_velocity *=-0.9;
			hw->set_angular_velocity({0,0,curr_angular_velocity});
		}
	}
	//std::cout << starting_angle << ", " << get_true_yaw() << std::endl;
	stop();
}

void Movement::rotate(double rotation, double precision){
	/*
	 * rotation:	The desired rotation in degrees. Note that a rotation
	 * 					greater than 180° in modulo will be substituted with
	 * 					a rotation in the opposite direction.
	 * precision:	The	maximum difference from the required rotation and the
	 * 					actual rotation. If this value is smaller than the sensor
	 * 					noise it makes no sense.
	 */
	stop();
	double curr_angular_velocity = angular_velocity;
	double starting_angle = pos->get_yaw();
	int direction, framenum=1;
	double difference,
		   current_angle = starting_angle,
		   target_angle=fmod((rotation+current_angle),360),
		   prev_speed=0,
		   prev_stamp=0;
	std::vector<double> imu;
	// adjust for discontinuity at +/-180°
	target_angle = target_angle >  180 ? target_angle-360 : target_angle;
	target_angle = target_angle < -180 ? target_angle+360 : target_angle;
	difference = fmod((target_angle-current_angle),360);
	difference = difference> 180? difference-360:difference;
	difference = difference<-180? difference+360:difference;
	direction = copysign(1, -rotation);
	while(abs(difference)>precision){
		std::cout << "Current angle: " << pos->get_yaw() << std::endl;
		std::cout << "Target angle:  " << target_angle << std::endl;
		std::cout << "Difference:    " << difference   << std::endl;
		std::cout << "--------------------------------------------------------------------------------"  << std::endl;
		//vision->update_frame();
		framenum = (framenum+1)%100;
		if(!framenum){
			//std::cout << "error:          " << difference << std::endl;
			if(cv::waitKey(1) == ' '){
				break;
			}
		}
		imu = hw->get_imu();
		if (prev_stamp != 0){
			double elapsed_time = imu.at(0)-prev_stamp;
			double dtheta = elapsed_time*180/M_PI*(imu.at(3)+prev_speed)/2;
			pos->update_yaw(dtheta);
		}
		prev_stamp = imu.at(0);
		prev_speed = imu.at(3);

		current_angle = pos->get_yaw();
		difference = fmod((target_angle-current_angle),360);
		difference = difference> 180? difference-360:difference;
		difference = difference<-180? difference+360:difference;
		if(direction*difference > 0){
			// if we went over the specified angle, reverses the direction and
			// decreases the angular speed to have a better chance of sampling
			// at the right moment.
			curr_angular_velocity*=0.8;
			direction=-direction;
		}
		hw->set_angular_velocity({0,0,curr_angular_velocity*direction*(std::min<double>(1.0,abs(difference)/30))});
	}
	//std::cout << "target angle:  " << target_angle  <<std::endl;
	//pos->update_yaw(current_angle - starting_angle);
	stop();
}
