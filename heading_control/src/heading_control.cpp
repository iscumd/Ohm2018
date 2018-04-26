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
#include <boost/math/constants/constants.hpp>

// TODO: change to ROS params ASAP!
#define STOP_VEL 0.0
#define DRIVE_VEL 0.0
#define HEADING_MAX 180.0
#define HEADING_MIN -180.0

double get_heading(tf::TransformListener &listener) { // toss all the update code into one neat function
	tf::StampedTransform tform;
	ros::Time now = ros::Time::now();
	listener.waitForTransform("ohm_base_link", "world", now, ros::Duration(0.15));	
	listener.lookupTransform("ohm_base_link", "world", now, tform);
	tf::Quaternion q = tform.getRotation(); // tf puts rotation data into quaternions, not RPY

	// stolen from wikipedia, shamelessly
	double ysqr = q.getAxis().y() * q.getAxis().y(), zsqr = q.getAxis().z() * q.getAxis().z();

	// yaw (z-axis rotation)
	double t0 = +2.0f * (q.getAxis().w() * q.getAxis().z() + q.getAxis().x() * q.getAxis().y());
	double t1 = +1.0f - 2.0f * (ysqr + zsqr);
	return std::atan2(t0, t1) * (180.0 / boost::math::double_constants::pi) + 180.0; // convert to degrees and put into [0, 360)
}

bool circular_range_compare(double min, double max, double val) {
	if(min > max && fabs(max - min) + min > 360.0) return ((val >= min && val < 360.0) || (val <= max && val >= 0.0));
	else return (val >= min && val <= max); // compares values on a [0, 360) and handles wrapping to 0
}

bool rough_equals(double x, double y, double threshold) { return (std::abs(x - y) < threshold); };

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

		drive_command.angular.z = controller.update(current_heading, PID::terms_t::P);
		
		if(drive_command.angular.z > 1.0) drive_command.angular.z = 1.0;
		else if(drive_command.angular.z < -1.0) drive_command.angular.z = -1.0;	

		ROS_INFO("Target: %f | Actual: %f | Update: %f", targets[target], current_heading, drive_command.angular.z);

		if(rough_equals(current_heading, targets[target], 0.5)) {
			ROS_INFO("HIT Target: %f | Actual: %f", targets[target], current_heading);
			target++;
			controller.target(targets[target]);
		}

		vel_pub.publish(drive_command);

		r.sleep();
		
		ros::spinOnce();
	}
}


/*for(auto heading_range = possible_headings.ranges.begin(); heading_range != possible_headings.ranges.end(); ++heading_range) {
			if(circular_range_compare(heading_range->start, heading_range->end, heading)) {
				
			}
		}*/
