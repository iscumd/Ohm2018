#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <string>

struct scan_transformer {
  	ros::NodeHandle nh;
  	ros::Publisher pub;
  	ros::Subscriber scanSub;
  	laser_geometry::LaserProjection projector_;
  	tf::TransformListener listener;
	std::string ref_frame_id;

  	scan_transformer() {
      	ROS_INFO_STREAM("Converting /scan to XY point cloud");

      	// Create a ROS subscriber for the input laser scan
      	bool use_tf = false;
		nh.param("use_tf", use_tf, use_tf);
		nh.param(std::string("reference_frame"), ref_frame_id, std::string("world"));
		
      	if(use_tf) scanSub = nh.subscribe(std::string("scan"), 1, &scan_transformer::scan_to_XY_tf, this);
      	else scanSub = nh.subscribe(std::string("scan"), 1, &scan_transformer::scan_to_XY, this);

      	// Create a ROS publisher for the output point cloud
      	pub = nh.advertise<sensor_msgs::PointCloud>(std::string("scan_to_xy_out"), 1);

      	ros::Duration(5.0).sleep();
  	};

  	void scan_to_XY (const sensor_msgs::LaserScan::ConstPtr& scan_in) {
		// Create a container for the data.
  	  	sensor_msgs::PointCloud cloud;
		cloud.header.stamp = ros::Time::now();
		cloud.header.frame_id = scan_in->header.frame_id;

  	  	//convert scan data into point cloud
    	projector_.projectLaser(*scan_in, cloud);

    	//publish point cloud
    	pub.publish(cloud);  

  	};

	void scan_to_XY_tf(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
		// make sure to set frame_id param for isc_sick to base_laser
    	if(!listener.waitForTransform(scan_in->header.frame_id, ref_frame_id, scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0))) {
    	  ROS_INFO("scan_to_XY_tf: waitForTransform failed");
    	  return;
   		}
  
   		sensor_msgs::PointCloud cloud;
    	projector_.transformLaserScanToPointCloud(ref_frame_id, *scan_in, cloud, listener);

    	pub.publish(cloud);
  	}; 
};

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "scan_to_xy");

  scan_transformer transformer;

  // Spin
  ros::spin ();
}
