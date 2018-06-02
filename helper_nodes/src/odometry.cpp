#include <ros/ros.h>
#include <ros/console.h>
#include "util.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <ohm_igvc_msgs/Target.h>
#include <ohm_igvc_srvs/coordinate_convert.h>
#include <vn300/Pose.h>
#include <string>
#include <cmath>

#define DEG2RAD(x) ((3.14159265359 * x) / 180.00)

class odometry {
  public:
    odometry();
    void position_callback(const vn300::Pose::ConstPtr &pos);
    bool convert_callback(ohm_igvc_srvs::coordinate_convert::Request &rq, ohm_igvc_srvs::coordinate_convert::Response &rp);
	double gps_x(double lon) { 
		//ROS_INFO("K_EW = %f", K_EW);
		//ROS_INFO("lon = %f", lon);
		//ROS_INFO("start lon = %f", origin.longitude);
		return (K_EW * (lon - origin.longitude)); 
	};

	double gps_y(double lat) { 
		//ROS_INFO("K_NS = %f", K_NS);
		//ROS_INFO("lat = %f", lat);
		//ROS_INFO("start lat = %f", origin.latitude);
		return (K_NS * (lat - origin.latitude)); 
	};

  private:
    ros::Subscriber positionSub;
    ros::Publisher pose;
    ros::ServiceServer coord_convert;
    ros::NodeHandle node;
    ohm_igvc_msgs::Target origin;

    double K_NS, K_EW;

    tf::TransformBroadcaster base_br;
	geometry_msgs::TransformStamped t;

    geometry_msgs::Pose2D position;
};

odometry::odometry() {
	K_NS = 111120.00;

	ros::NodeHandle nh_private("~");
	nh_private.param("K_NS", K_NS, K_NS);
    nh_private.param("origin_latitude", origin.latitude, 0.0);
    nh_private.param("origin_longitude", origin.longitude, 0.0);

	K_EW = K_NS * std::cos(DEG2RAD(origin.latitude));

	ROS_INFO("K_NS = %f", K_NS);
	ROS_INFO("K_EW = %f", K_EW);

    positionSub = node.subscribe<vn300::Pose>("pose", 5, &odometry::position_callback, this);

	coord_convert = node.advertiseService("coordinate_convert", &odometry::convert_callback, this);

    pose = node.advertise<geometry_msgs::Pose2D>("odom", 5);
    position.x = 0.0;
    position.y = 0.0;
    position.theta = 0.0;
}

void odometry::position_callback(const vn300::Pose::ConstPtr &pos) {
    position.y = gps_x(pos->position[1]); // 1 is lon
	position.x = gps_y(pos->position[0]); // 0 is lat

    position.theta = pos->heading[0];
	
	t.header.stamp = ros::Time::now();
	t.header.frame_id = "world";
	t.child_frame_id = "ohm_base_link";
	
	t.transform.translation.x = position.x;
  	t.transform.translation.y = position.y;
  	t.transform.translation.z = 0.0;
  	t.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(position.theta));

    base_br.sendTransform(t);

	pose.publish(position);
}

bool odometry::convert_callback(ohm_igvc_srvs::coordinate_convert::Request &rq, ohm_igvc_srvs::coordinate_convert::Response &rp) {
	rp.coordinate.y = gps_x(rq.coordinate.longitude);
	rp.coordinate.x = gps_y(rq.coordinate.latitude);

	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "odometry");

    odometry node;

    ros::spin();

    return 0;
}
