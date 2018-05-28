#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <helper_nodes/util.h>
#include <ohm_igvc_msgs/coordinate_convert.h>
#include <ohm_igvc_msgs/Range.h>
#include <ohm_igvc_msgs/RangeArray.h>
#include <ohm_igvc_msgs/Waypoint.h>
#include <ohm_igvc_srvs/waypoint.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "pid.h"

#include <cmath>
#include <array>
#include <string>

// TODO: change to ROS params ASAP!
#define STOP_VEL 0.0
#define DRIVE_VEL 0.0
#define HEADING_MAX 180.0
#define HEADING_MIN -180.0

// lidar and camera message variables
ohm_igvc_msgs::RangeArray lidar_ranges;
ohm_igvc_msgs::TurnAngles camera_angles;	

// waypoint variables
int waypoint_id = 0;
ohm_igvc_msgs::Waypoint goal;

// robot pose
ohm_igvc_msgs::Waypoint pose;

void ranges_callback(const ohm_igvc_msgs::RangeArray::ConstPtr &ranges) {
	lidar_ranges.header = ranges->header;
	lidar_ranges.ranges = ranges->ranges;
}

void turn_angles_callback(const ohm_igvc_msgs::TurnAngles::ConstPtr &angles) {
	camera_angles.header = angles->header;
	camera_angles.angles = angles->angles;
}

bool get_pose(tf::TransformListener listener) { // toss all the update code into one neat function
	tf::StampedTransform tform;
	if(listener.waitForTransform(base_frame_id, ref_frame_id, ros::Time::now(), ros::Duration(0.15))) {
		listener.lookupTransform(base_frame_id, ref_frame_id, ros::Time(0), tform);
		pose.position.x = tform.getOrigin().x();
		pose.position.y = tform.getOrigin().y();
		pose.heading = tf::getYaw(tform.getRotation()) * (180.0 / boost::math::double_constants::pi) + 180.0; // convert to degrees and put into [0, 360)
		return true;	
	} 
	
	return false;
}

bool get_next_waypoint() {
	ohm_igvc_srvs::waypoint req_wp;
	ohm_igvc_srvs::coordinate_convert req_conv;

	req_wp.request.ID = waypoint_id;
	
	if(!waypoint_service.call(req_wp)) return false;

	req_conv.request.coordinate.latitude = req_wp.response.waypoint.latitude;
	req_conv.request.coordinate.longitude = req_wp.response.waypoint.longitude;

	if(!coord_convert.call(req_conv)) return false;

	req_cell.request.real_coordinate.x = req_conv.response.coordinate.x;
	req_cell.request.real_coordinate.y = req_conv.response.coordinate.y;

	goal.position.x = req_conv.response.coordinate.x;
	goal.position.y = req_conv.response.coordinate.y;
	goal.heading = req_wp.coordinate.heading;

	waypoint_id++;

	return true;
}
	
int main(int argc, char** argv) {
	ros::init(argc, argv, "heading_control");

	/*** VARIABLES ***/
	// internal variables
	double waypoint_hit_thresh;
    double turn_to_heading_thresh;
	double max_linear_speed;
	double max_angular_speed;
	double desired_heading;
	std::string drive_mode;
	bool gps_enable;
	bool lidar_enable;
	bool camera_enable;
	bool et_go_home = false;
	ohm_igvc_msgs::waypoint start;

	// PID
	double kP;
	double kI;
	double kP;
	double max_i_err;

	PID::terms_t PID_type; // which terms we are using

	// ros specific
	ros::NodeHandle node;
	ros::Subscriber lidar_sub = node.subscribe<ohm_igvc_msgs::RangeArray>("free_ranges", &ranges_callback);
	ros::Subscriber camera_sub = node.subscribe<ohm_igvc_msgs::TurnAngles>("turn_angles", &turn_angles_callback);
	ros::ServiceClient waypoint_srv = node.serviceClient<ohm_igvc_srvs::waypoint>("waypoint");
	ros::ServiceClient coordinate_convert = node.serviceClient<ohm_igvc_srvs::coordinate_convert>("coordinate_convert");
	ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("auto_control", 1);

	// tf stuff
	std::string base_frame_id;
	std::string ref_frame_id;
	tf::TransformListener pose_listener;
	
	// get internal params
	node.param("waypoint_hit", waypoint_hit_thresh, 1.0);
	node.param("turn_to_heading", turn_to_heading_thresh, 90.0);
	node.param("max_linear", max_linear_speed, 0.3);
	node.param("max_angular", max_angular_speed, 0.25);
	node.param("drive_mode", drive_mode, "manual");
	node.param("enable_gps", gps_enable, true);
	node.param("enable_lidar", lidar_enable, true);
	node.param("enable_camera", camera_enable, true);

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

	geometry_msgs::Twist drive_command;
	drive_command.linear.x = STOP_VEL;

	ros::Rate r(10);

	std::array<double, 5> targets = {0.0, 50.0, 100.0, 175.0, 260.0};
	int target = 0;

	tf::TransformListener pose_listener;

	std::string drive_mode = "manual";
	bool updatedDriveMode = false;

	if(get_pose()) {
		start = pose;
	}

	PID controller(kP, kI, kD, max_i_err);
	
	while(ros::ok()) {
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

		if(get_pose()) {
			desired_heading = pose.heading;			
		
			if(gps_enable) {
				desired_heading = geometric::angular_distance(pose.position, goal);

				bool hit_waypoint = rough_cmp::lt_eq(geometric::distance(pose.position, goal), waypoint_hit_thresh, 0.1);
				bool hit_heading = rough_cmp::equals(desired_heading, goal.heading, 3.0);

				if(hit_waypoint && hit_heading) {
					if(!et_go_home && !get_next_waypoint()) {
						et_go_home = true;
						goal = start;
					} else {
						drive_command.linear.x = 0.0;
						drive_command.angular.z = 0.1;

						vel_pub.publish(drive_command);
						break;
					}
				} else if(hit_waypoint) {
					drive_command.linear.x = STOP_VEL;
					desired_heading = goal.heading;
				} else {
					if(
				}
			}
			
			if(lidar_enable) {
				
			}

			if(camera_enable) {

			}
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

	return 0;
}


/*for(auto heading_range = possible_headings.ranges.begin(); heading_range != possible_headings.ranges.end(); ++heading_range) {
			if(circular_range_compare(heading_range->start, heading_range->end, heading)) {
				
			}
		}*/
