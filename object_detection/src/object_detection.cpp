#include "object_detection.h"

object_detector::object_detector(ros::NodeHandle nh) : node(nh) {
	// parameters
	node.param("forgivable", forgivable, 3);
	node.param("max_point_distance", eps, 1.0); // meters
	node.param("min_group_count", min_group_count, 4);
	node.param("reaction_distance", min_obstacle_distance, 100.0); // meters (?)
	node.param("base_frame", base_frame_id, std::string("base"));
	node.param("reference_frame", ref_frame_id, std::string("world"));

	pcl_input = node.subscribe(std::string("scan_to_xy_out"), 1, &object_detector::find_point_groups, this);

	object_groups_pub = node.advertise<sensor_msgs::PointCloud>("object_groups", 1);
	object_markers_pub = node.advertise<visualization_msgs::Marker>("object_markers", 1);
	range_pub = node.advertise<ohm_igvc_msgs::RangeArray>("ranges", 1);
}

//*** MAIN FUNCTIONS ***//

void object_detector::find_point_groups(const sensor_msgs::PointCloud::ConstPtr &pcl) {
	std::list<geometry_msgs::Point32> group, points(pcl->points.begin(), pcl->points.end());
	std::vector<std::list<geometry_msgs::Point32>> groups;
	groups.reserve(200);

	// ROS_INFO("Iterating over point cloud:");
	// ROS_INFO("\tpcl.size() => %d", (int)points.size());

	last_received_pcl = pcl->header.stamp;
	pcl_frame = pcl->header.frame_id;

	for(auto point = points.begin(), last_point = points.begin(); points.size() > min_group_count;) {
		if(group.empty()) {
			// ROS_INFO("Starting new group (%d)", (int)groups.size());
			group.push_back(*point);
			last_point = point;
			point = find_next_point(point, points.end());
			points.erase(last_point);
		} else {
			if(point != points.end()) {
				// ROS_INFO("\tAdding point (%f, %f) to group (%d)", point->x, point->y, (int)groups.size());
				group.push_back(*point);
				last_point = point;
				point = find_next_point(point, points.end());
				points.erase(last_point);
			} else {
				// ROS_INFO("Ending group (%d)", (int)groups.size());
				groups.push_back(group);
				group.clear();
				point = points.begin();
			}
		}	
	}

	groups.push_back(group);
	// ROS_INFO("Ending last group (%d)", (int)groups.size() - 1);

	int point_count = 0;

	for(auto group = groups.begin(); group != groups.end(); ++group) {
		for(auto point = group->begin(); std::distance(point, group->end()) > min_group_count;) {
			group->insert(point, geometric::n_average(point, std::next(point,min_group_count)));	
			if(std::distance(std::next(point, min_group_count), group->end()) < min_group_count) {
				point = group->erase(point, group->end());
			} else {			
				point = group->erase(point, std::next(point, min_group_count));
			}
		}

		point_count += group->size();

	}

	// ROS_INFO("\tFound %d groups, consisting of %d points", (int)groups.size(), point_count);

	find_valid_ranges(groups);
	publish_obstacles_rviz(groups);
}
		
void object_detector::find_valid_ranges(std::vector<std::list<geometry_msgs::Point32>> groups) {
	auto prev_group = groups.end();
	ohm_igvc_msgs::RangeArray ranges;
	ohm_igvc_msgs::Range range;

	if(get_pose()) {
		double last_dist = 0.0, last_angle = circular_range::wrap((pose.heading - 135.0), 360.0), this_angle, this_dist;
	
		for(auto group = groups.begin(); group != groups.end(); ++group) {
			for(auto point = group->begin(); point != group->end(); ++point) {
				// ROS_INFO("point %d, group %d: (%f, %f)", (int)std::distance(group->begin(), point), (int)std::distance(groups.begin(), group), point->x, point->y);

				this_angle = circular_range::wrap(geometric::angular_distance(pose.position, point32_to_point(*point)), 360.0);
				this_dist = geometric::distance(point32_to_point(*point), pose.position);

				// ROS_INFO("\t@ %f deg, %f m", this_angle, this_dist);

				if(this_dist > min_obstacle_distance) { // is this distance greater than reaction distance
					if(last_dist <= min_obstacle_distance) { //if so, check whether the last one was less than reaction distance					
						range.start = this_angle; // if it was, then we can start a possible free range here
						// ROS_INFO("Set range start = %f [%d]", range.start, (int)ranges.ranges.size());
					} else { // if it wasn't, then check if this is the last possible point
						if(std::next(group, 1) == groups.end() && std::next(point, 1) == group->end()) { 
							range.end = this_angle;
							if(circular_range::smallest_difference(range.start, range.end) > 10.0) { // please remove this magic number, cunt																	
								// ROS_INFO("Set range end = %f [%d]\nPush back range (%f, %f) [%d]", range.end, (int)ranges.ranges.size(), range.start, range.end, (int)ranges.ranges.size());
								ranges.ranges.push_back(range);
							}
						}
					} 
				} else { // is this distance less than the reaction distance
					if(last_dist > min_obstacle_distance) { // if so, was the last distance greater?
						range.end = this_angle; // if it was, we can end the range on this angle
						if(circular_range::smallest_difference(range.start, range.end) > 10.0) { // please remove this magic number, cunt			
							// ROS_INFO("Set range end %f [%d]\nPush back range (%f, %f) [%d]", range.end, (int)ranges.ranges.size(), range.start, range.end, (int)ranges.ranges.size());					
							ranges.ranges.push_back(range);
						} //else {
							// ROS_INFO("Range is too small; ignoring. (%f, %f, diff = %f)", range.start, range.end, circular_range::smallest_difference(range.start, range.end));
						//}
					}
				} 
				
				last_dist = this_dist;
				last_angle = this_angle;
			}
		}
	} else {
		ROS_INFO("Did not get pose!");
	}
		
	ranges.header.frame_id = pcl_frame;
	ranges.header.stamp = last_received_pcl;
	range_pub.publish(ranges);
}

//*** HELPER FUNCTIONS ***//

void object_detector::publish_obstacles_rviz(std::vector<std::list<geometry_msgs::Point32>> groups) {
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = pcl_frame;
	marker.header.stamp = ros::Time();
	marker.ns = "object_rec";
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;
	marker.scale.x = 0.03;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.lifetime = ros::Duration(2.0);

	sensor_msgs::PointCloud pcl;
	sensor_msgs::ChannelFloat32 colors;
	pcl.header.frame_id = pcl_frame;
	colors.name = "intensity";
	
	for(auto group = groups.begin(); group != groups.end(); ++group) {
		for(auto point = group->begin(); point != group->end(); ++point) {
			pcl.points.push_back(*point);
			marker.points.push_back(point32_to_point(*point));
			colors.values.push_back((double)std::distance(groups.begin(), group));
		}

		marker.id = std::distance(group, groups.end());
		object_markers_pub.publish(marker);
		marker.points.clear();
	}

	pcl.channels.push_back(colors);
	object_groups_pub.publish(pcl);
};

bool object_detector::get_pose() { // toss all the update code into one neat function
	tf::StampedTransform tform;
	if(pose_listener.waitForTransform(ref_frame_id, base_frame_id, ros::Time::now(), ros::Duration(0.15))) {
		pose_listener.lookupTransform(ref_frame_id, base_frame_id, ros::Time(0), tform);
		pose.position.x = tform.getOrigin().x();
		pose.position.y = tform.getOrigin().y();
		pose.heading = circular_range::wrap(tf::getYaw(tform.getRotation()) * (180.0 / geometric::pi), 360.0); // convert to degrees and put into [0, 360)
		return true;	
	} 
	
	return false;
};

geometry_msgs::Point object_detector::point32_to_point(geometry_msgs::Point32 p) {
	geometry_msgs::Point q;
	q.x = p.x;
	q.y = p.y;
	q.z = p.z;
	return q;
};

template <typename point_iterator>
point_iterator object_detector::find_next_point(point_iterator current, point_iterator end) {
	int forgiven = 0;
	point_iterator next_point = std::next(current, 1);
	while(next_point != end) {
		if(geometric::distance<geometry_msgs::Point32>(*next_point, *current) < eps) {
			return next_point;
		} else if(forgiven < forgivable) {
			next_point++;
			forgiven++;
		} else {
			return end;
		}
	}

	return end;
}
