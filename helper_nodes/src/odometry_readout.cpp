#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include "util.h"
#include <ohm_igvc_msgs/RangeArray.h>
#include <ohm_igvc_msgs/Waypoint.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <iterator>

ohm_igvc_msgs::Waypoint r_pose;
ohm_igvc_msgs::RangeArray ranges;
double raw_heading;

std::string base_frame_id;
std::string ref_frame_id;

bool get_pose(tf::TransformListener &pose_listener) { // toss all the update code into one neat function
	tf::StampedTransform tform;
	if(pose_listener.waitForTransform(ref_frame_id, base_frame_id, ros::Time::now(), ros::Duration(0.15))) {
		pose_listener.lookupTransform(ref_frame_id, base_frame_id, ros::Time(0), tform);
		r_pose.position.x = tform.getOrigin().x();
		r_pose.position.y = tform.getOrigin().y();
		r_pose.heading = circular_range::wrap(tf::getYaw(tform.getRotation()) * (180.0 / geometric::pi), 360.0); // convert to degrees and put into [0, 360)
		raw_heading = r_pose.heading - 180.0;
		return true;	
	} 
		
	return false;
};

void get_ranges(const ohm_igvc_msgs::RangeArray::ConstPtr &array) {
	ranges = *array;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odometry_listener");
	ros::NodeHandle node;

	ros::Subscriber ranges_sub = node.subscribe(std::string("/ohm/ranges"), 1, &get_ranges);

	ros::Publisher ranges_start = node.advertise<visualization_msgs::MarkerArray>("ranges_start", 1); 
	ros::Publisher ranges_end = node.advertise<visualization_msgs::MarkerArray>("ranges_end", 1); 
	ros::Publisher pose_and_bounds = node.advertise<visualization_msgs::MarkerArray>("pose_and_bounds", 1);

	node.param("base_frame", base_frame_id, std::string("ohm_base_link"));
	node.param("reference_frame", ref_frame_id, std::string("world"));

	tf::TransformListener listener;

	visualization_msgs::MarkerArray start_ranges;
	visualization_msgs::MarkerArray end_ranges;
	visualization_msgs::MarkerArray pnb;

	std_msgs::ColorRGBA RED;
	RED.r = 1.0;
	RED.g = 0.0;
	RED.b = 0.0;
	RED.a = 1.0;

	std_msgs::ColorRGBA WHITE;
	WHITE.r = 1.0;
	WHITE.g = 1.0;
	WHITE.b = 1.0;
	WHITE.a = 1.0;

	visualization_msgs::Marker start_range;
	start_range.header.frame_id = ref_frame_id;
	start_range.header.stamp = ros::Time();
	start_range.ns = "start_ranges";
	start_range.type = visualization_msgs::Marker::ARROW;
	start_range.action = visualization_msgs::Marker::ADD;
	start_range.scale.x = 5.0;
	start_range.scale.y = 0.15;
	start_range.scale.z = 0.1;
	start_range.color.a = 1.0; 
	start_range.color.r = 213.0 / 255.0;
	start_range.color.g = 247.0 / 255.0;
	start_range.color.b = 64.0 / 255.0;
	start_range.lifetime = ros::Duration(0.5);

	visualization_msgs::Marker end_range;
	end_range.header.frame_id = ref_frame_id;
	end_range.header.stamp = ros::Time();
	end_range.ns = "end_range";
	end_range.type = visualization_msgs::Marker::ARROW;
	end_range.action = visualization_msgs::Marker::ADD;
	end_range.pose.position.x = 0.0;
	end_range.scale.x = 5.0;
	end_range.scale.y = 0.15;
	end_range.scale.z = 0.1;
	end_range.color.a = 1.0; 
	end_range.color.r = 219.0 / 255.0;
	end_range.color.g = 50.0 / 255.0;
	end_range.color.b = 50.0 / 255.0;
	end_range.lifetime = ros::Duration(0.5);

	visualization_msgs::Marker pose;
	pose.header.frame_id = ref_frame_id;
	pose.header.stamp = ros::Time();
	pose.ns = "pose";
	pose.type = visualization_msgs::Marker::ARROW;
	pose.action = visualization_msgs::Marker::ADD;
	pose.scale.x = 2.0;
	pose.scale.y = 0.15;
	pose.scale.z = 0.1;
	pose.lifetime = ros::Duration(2.0);

	visualization_msgs::Marker bounds;
	bounds.header.frame_id = ref_frame_id;
	bounds.header.stamp = ros::Time();
	bounds.ns = "lidar_bounds";
	bounds.type = visualization_msgs::Marker::ARROW;
	bounds.action = visualization_msgs::Marker::ADD;
	bounds.scale.x = 20.0;
	bounds.scale.y = 0.15;
	bounds.scale.z = 0.1;
	bounds.color.a = 1.0;
	bounds.color.r = 25.0 / 255.0;
	bounds.color.g = 1.0;
	bounds.color.b = 0.0;
	bounds.lifetime = ros::Duration(2.0);
	
	while(ros::ok()) {
		if(get_pose(listener)) {
			ROS_INFO("Got pose!");
			ROS_INFO("\t-> X: %f, Y: %f", r_pose.position.x, r_pose.position.y);
			ROS_INFO("\t-> Heading (norm): %f, Heading (true): %f", r_pose.heading, raw_heading);

			/** SET POSES AND BOUNDS **/
			// set pose data
			pose.pose.position.x = r_pose.position.x;
			pose.pose.position.y = r_pose.position.y;
			
			pose.pose.orientation = tf::createQuaternionMsgFromYaw(r_pose.heading * (geometric::pi / 180.0));
			pose.color = RED;
			pose.id = 0; // normalized pose
			
			pnb.markers.push_back(pose);

			pose.pose.orientation = tf::createQuaternionMsgFromYaw(raw_heading * (geometric::pi / 180.0));
			pose.color = WHITE;
			pose.id = 1; // true pose

			pnb.markers.push_back(pose);

			// set bounds data
			bounds.pose.position.x = r_pose.position.x;
			bounds.pose.position.y = r_pose.position.y;
			bounds.pose.orientation = tf::createQuaternionMsgFromYaw(circular_range::wrap(r_pose.heading - 135.0, 360.0) * (geometric::pi / 180.0));
			bounds.id = 0; // left lidar bound			

			pnb.markers.push_back(bounds);
			
			bounds.pose.orientation = tf::createQuaternionMsgFromYaw(circular_range::wrap(r_pose.heading + 135.0, 360.0) * (geometric::pi / 180.0));
			bounds.id = 1; // right lidar bound

			pnb.markers.push_back(bounds);

			/** SET RANGES **/
			// set range positions
			start_range.pose.position.x = r_pose.position.x;
			start_range.pose.position.y = r_pose.position.y;

			end_range.pose.position.x = r_pose.position.x;
			end_range.pose.position.y = r_pose.position.y;

			// set ranges;
			for(auto range = ranges.ranges.begin(); range != ranges.ranges.end(); ++range) {
				start_range.pose.orientation = tf::createQuaternionMsgFromYaw(range->start * (geometric::pi / 180.0));
				start_range.id = std::distance(ranges.ranges.begin(), range);

				end_range.pose.orientation = tf::createQuaternionMsgFromYaw(range->end * (geometric::pi / 180.0));
				end_range.id = start_range.id;
				
				start_ranges.markers.push_back(start_range);
				end_ranges.markers.push_back(end_range);
			}
	
			/** PUBLISH MARKERS **/
			pose_and_bounds.publish(pnb);
			ranges_start.publish(start_ranges);
			ranges_end.publish(end_ranges);

			pnb.markers.clear();
			start_ranges.markers.clear();
			end_ranges.markers.clear();

		} else {
			ROS_INFO("Did not get pose!");
		}

		ros::spinOnce();
	}
}
