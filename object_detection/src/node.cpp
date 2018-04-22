#include "object_detection.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "object_detection");
	ros::NodeHandle nh;

	object_detector rec(nh);

	ros::spin();
}
