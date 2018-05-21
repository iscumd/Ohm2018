#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Point.h>
#include "util.h"
#include <ohm_igvc_msgs/Waypoint.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <string>

ohm_igvc_msgs::Waypoint pose;
double raw_heading;

std::string base_frame_id;
std::string ref_frame_id;

bool get_pose(tf::TransformListener &pose_listener) { // toss all the update code into one neat function
	tf::StampedTransform tform;
	if(pose_listener.waitForTransform(ref_frame_id, base_frame_id, ros::Time::now(), ros::Duration(0.15))) {
		pose_listener.lookupTransform(ref_frame_id, base_frame_id, ros::Time(0), tform);
		pose.position.x = tform.getOrigin().x();
		pose.position.y = tform.getOrigin().y();
		pose.heading = tf::getYaw(tform.getRotation()) * (180.0 / geometric::pi) + 180.0; // convert to degrees and put into [0, 360)
		raw_heading = pose.heading - 180.0;
		return true;	
	} 
		
	return false;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "odometry_listener");
	ros::NodeHandle node;

	node.param("base_frame", base_frame_id, std::string("base"));
	node.param("reference_frame", ref_frame_id, std::string("world"));

	tf::TransformListener listener;
	
	while(ros::ok()) {
		if(get_pose(listener)) {
			ROS_INFO("Got pose!");
			ROS_INFO("\t-> X: %f, Y: %f", pose.position.x, pose.position.y);
			ROS_INFO("\t-> Heading (norm): %f, Heading (true): %f", pose.heading, raw_heading);
		} else {
			ROS_INFO("Did not get pose!");
		}
	}
}
