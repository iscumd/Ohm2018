#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include "util.h"
#include <ohm_igvc_msgs/RangeArray.h>
#include <ohm_igvc_msgs/Waypoint.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define DEG2RAD(x) ((geometric::pi * x) / 180.00)
#define RAD2DEG(x) ((180.00 * x) / geometric::pi)

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_broadcaster");
	ros::NodeHandle node;
	ros::Rate r(10.0);

    tf::TransformBroadcaster base_br;
	geometry_msgs::TransformStamped t;

	double angle = 0.0;
	double increment = 1.0;
	
	while(ros::ok()) {
		if(angle >= 180.0) {
			angle = -180.0;
		} else {
			angle += increment;
		}

		ROS_INFO("angle pre-tf: %f", angle);
		
		t.header.stamp = ros::Time::now();
		t.header.frame_id = "world";
		t.child_frame_id = "base";
	
		t.transform.translation.x = 0.0;
  		t.transform.translation.y = 0.0;
  		t.transform.translation.z = 0.0;
  		t.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(angle));

		base_br.sendTransform(t);

		r.sleep();
	}
}
