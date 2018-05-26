#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include "util.h"
#include <ohm_igvc_msgs/RangeArray.h>
#include <ohm_igvc_msgs/Waypoint.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#define DEG2RAD(x) ((geometric::pi * x) / 180.00)
#define RAD2DEG(x) ((180.00 * x) / geometric::pi)

ohm_igvc_msgs::Waypoint r_pose;
double unwrapped_angle;

std::string base_frame_id;
std::string ref_frame_id;

bool get_pose(tf::TransformListener &pose_listener) { // toss all the update code into one neat function
	tf::StampedTransform tform;
	if(pose_listener.waitForTransform(ref_frame_id, base_frame_id, ros::Time::now(), ros::Duration(0.15))) {
		pose_listener.lookupTransform(ref_frame_id, base_frame_id, ros::Time(0), tform);
		r_pose.position.x = tform.getOrigin().x();
		r_pose.position.y = tform.getOrigin().y();
		r_pose.heading = circular_range::wrap(RAD2DEG(tf::getYaw(tform.getRotation())), 360.0); // convert to degrees and put into [0, 360)
		unwrapped_angle = RAD2DEG(tf::getYaw(tform.getRotation()));
		return true;	
	} 
		
	return false;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_listener");
	ros::NodeHandle node;
	ros::Rate r(10.0);

	node.param("base_frame", base_frame_id, std::string("base"));
	node.param("reference_frame", ref_frame_id, std::string("world"));

	tf::TransformListener pose_listener;
	
	while(ros::ok()) {
		if(get_pose(pose_listener)) {
			ROS_INFO("angle post-tf (wrapped): %f | angle post-tf (unwrapped): %f", r_pose.heading, unwrapped_angle);
		} else {
			ROS_INFO("Didn't get a pose");
		}

		r.sleep();
	}
}
