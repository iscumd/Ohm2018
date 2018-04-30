#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <ohm_igvc_msgs/Range.h>
#include <ohm_igvc_msgs/RangeArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "pid.h"

#include <cmath>
#include <array>
#include <string>
#include <boost/math/constants/constants.hpp>

// TODO: change to ROS params ASAP!
#define STOP_VEL 0.0
#define DRIVE_VEL 0.0
#define HEADING_MAX 180.0
#define HEADING_MIN -180.0

class heading_controller {
	public:
		heading_controller() {
			// get internal params
			node.param("max_linear", max_linear_speed, 0.3);
			node.param("max_angular", max_angular_speed, 0.25);
			node.param("drive_mode", drive_mode, "manual")

			// get pid params
			node.param("kP", kP, 0.01);
			node.param("kI", kI, 0.0);
			node.param("kD", kD, 0.0);
			node.param("max_integral_error", max_i_err, 0.5);

			if(kP != 0.0) PID_type |= PID::terms_t::P;
			if(kI != 0.0) PID_type |= PID::terms_t::I;
			if(kD != 0.0) PID_type |= PID::terms_t::D;

			// tf stuff
			node.param("base_frame", base_frame_id, "base");
			node.param("reference_frame", ref_frame_id, "world");

			// subscriptions and publications
			camera_sub = node.subscribe<ohm_igvc_msgs::TurnAngles>("turn_angles", &heading_controller::ranges_callback, this);
			lidar_sub = node.subscribe<ohm_igvc_msgs::RangeArray>("free_ranges", &heading_controller::turn_angles_callback, this);
			vel_pub = node.advertise<geometry_msgs::Twist>("auto_control", 1);

		};

		void ranges_callback(const ohm_igvc_msgs::RangeArray::ConstPtr &ranges) {
			lidar_ranges.header = ranges->header;
			lidar_ranges.ranges = ranges->ranges;
		}

		void turn_angles_callback(const ohm_igvc_msgs::TurnAngles::ConstPtr &angles) {
			camera_angles.header = angles->header;
			camera_angles.angles = angles->angles;
		}

		/*** HELPER FUNCTIONS ***/

		double get_heading() { // toss all the update code into one neat function
			tf::StampedTransform tform;
			if(pose_listener.waitForTransform(base_frame_id, ref_frame_id, ros::Time::now(), ros::Duration(0.15))) {
				pose_listener.lookupTransform(base_frame_id, ref_frame_id, ros::Time(0), tform);
				return tf::getYaw(tform.getRotation()) * (180.0 / boost::math::double_constants::pi) + 180.0; // convert to degrees and put into [0, 360)
			} 
			
			return 361.0;
		};

		bool circular_range_compare(double min, double max, double val) {
			if(min > max && fabs(max - min) + min > 360.0) return ((val >= min && val < 360.0) || (val <= max && val >= 0.0));
			else return (val >= min && val <= max); // compares values on a [0, 360) and handles wrapping to 0
		};

		bool rough_equals(double x, double y, double threshold) { return (std::abs(x - y) < threshold); };
	
	private:
		// internal variables
		double max_linear_speed;
		double max_angular_speed;
		std::string drive_mode;

		ohm_igvc_msgs::RangeArray lidar_ranges;
		ohm_igvc_msgs::TurnAngles camera_angles;

		// PID
		double kP;
		double kI;
		double kP;
		double max_i_err;

		PID::terms_t PID_type; // which terms we are using

		// ros specific
		ros::NodeHandle node;
		ros::Subscriber lidar_sub;
		ros::Subscriber camera_sub;
		ros::ServiceClient waypoint_srv;
		ros::Publisher vel_pub;

		// tf stuff
		std::string base_frame_id;
		std::string ref_frame_id;
		tf::TransformListener pose_listener;
		


};
int main(int argc, char** argv) {
	ros::init(argc, argv, "heading_control");
	ros::NodeHandle node;
	ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("auto_control", 1);
	
	double kP, kI, kD, max_i_err;
	
	node.param("kP", kP, 0.5);
	node.param("kI", kI, 0.5);
	node.param("kD", kD, 0.5);
	node.param("max_integral_error", max_i_err, 0.5);

	ROS_INFO("kP: %f, kI: %f, kD: %f, max_i_err: %f", kP, kI, kD, max_i_err);

	geometry_msgs::Twist drive_command;
	drive_command.linear.x = STOP_VEL;

	ros::Rate r(10);

	std::array<double, 5> targets = {0.0, 50.0, 100.0, 175.0, 260.0};
	int target = 0;

	tf::TransformListener pose_listener;

	std::string drive_mode = "manual";
	bool updatedDriveMode = false;

	PID controller(kP, kI, kD, max_i_err);
	
	while(ros::ok() && target < targets.size()) {
		ros::param::get("/drive_mode", drive_mode);
		if(drive_mode == "manual") {
			if(updatedDriveMode) updatedDriveMode = false;
			ROS_INFO_THROTTLE(15, "Robot is in manual mode. Not testing.");
			r.sleep();
			continue;
		} else if(drive_mode == "auto" && !updatedDriveMode) {
			updatedDriveMode = true;
			ROS_INFO("Switched to auto mode. Begin testing.");
			controller.reset();
			target = 0;
		}

		double current_heading = get_heading(pose_listener);

		if(current_heading <= 360.0) {










			/*
			drive_command.angular.z = controller.update(current_heading, PID::terms_t::P);
		
			if(drive_command.angular.z > 0.35) drive_command.angular.z = 0.35;
			else if(drive_command.angular.z < -0.35) drive_command.angular.z = -0.35;	

			ROS_INFO("Target: %f | Actual: %f | Update: %f", targets[target], current_heading, drive_command.angular.z);

			if(rough_equals(current_heading, targets[target], 0.5)) {
				ROS_INFO("HIT Target: %f | Actual: %f", targets[target], current_heading);
				target++;
				controller.target(targets[target]);
			}
			*/
		} else {
			drive_command.angular.z = 0.0;
		}
		
		vel_pub.publish(drive_command);

		ros::spinOnce();

		r.sleep();
	}
}


/*for(auto heading_range = possible_headings.ranges.begin(); heading_range != possible_headings.ranges.end(); ++heading_range) {
			if(circular_range_compare(heading_range->start, heading_range->end, heading)) {
				
			}
		}*/
