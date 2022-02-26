#ifndef __GAZEBO_INTERFACE__
#define __GAZEBO_INTERFACE__

#include <opencv2/opencv.hpp>
#include <gazebo/transport/transport.hh>

struct timestamp_t {
	long int sec;
	long int nsec;
};
typedef struct timestamp_t timestamp_t;

class GazeboInterface {
	private:
		cv::Mat _frame;
		double _angle_magnetic;
		std::vector<double> _imu={0,0,0,0,0,0,0};
		unsigned int _world_reference = 0, _distance=0;
		std::vector<double> _pose = {0,0,0}, _angle = {0,0,0};
		gazebo::transport::SubscriberPtr sub_img,
			sub_dist,
			sub_rot,
			sub_pos,
			sub_mag,
			sub_imu;
		gazebo::transport::PublisherPtr pub_vel, pub_ang, pub_world, pub_phys;
		timestamp_t _sim_time;


		/*
		 * Callback functions for subscribed topics
		 */

		void cb_img(ConstImageStampedPtr &msg) ;

		void cb_imu(ConstIMUPtr &msg);


		void cb_distance(ConstLaserScanStampedPtr &msg) ;

		void cb_pose(ConstVector3dPtr &msg);

		void cb_angle(ConstVector3dPtr &msg);
		void cb_angle_magnetic(ConstMagnetometerPtr &msg);

	public:
		GazeboInterface();

		void init(bool verbose, std::string suffix);
		void quit_listener();

		double get_angle();
		unsigned int get_distance();

		double get_true_yaw();
		std::vector<double> get_imu();
		cv::Mat getImage();

		void set_object_frame();
		void set_world_frame();

		/*
		 * Publisher functions
		 */
		void set_linear_velocity(std::vector<double> v);

		void set_angular_velocity(std::vector<double> v);

		void resume_simulation();

		void pause_simulation();

		void set_sim_speed(int real_time_percentage);

		timestamp_t get_sim_time();
};

#endif
