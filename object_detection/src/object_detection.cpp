#include "object_detection.h"

object_detector::object_detector(ros::NodeHandle nh) : node(nh) {
	// parameters
	node.param("forgivable", forgivable, 3);
	node.param("max_point_distance", eps, 1.0);
	node.param("min_group_count", min_group_count, 4);
	node.param("min_obstacle_distance", min_obstacle_distance, 100.0);

	std::string pcl_topic;

	node.param("pcl_input_topic", pcl_topic, std::string("scan_to_xy_out"));

	ROS_INFO("pcl_input_topic: %s", pcl_topic.c_str());

	pcl_input = node.subscribe(pcl_topic, 1, &object_detector::find_point_groups, this);

	object_groups_pub = node.advertise<sensor_msgs::PointCloud>("object_groups", 1);
	object_markers_pub = node.advertise<visualization_msgs::Marker>("object_markers", 1);
	range_pub = node.advertise<ohm_igvc_msgs::RangeArray>("ranges", 1);
}

//*** MAIN FUNCTIONS ***//

void object_detector::find_point_groups(const sensor_msgs::PointCloud::ConstPtr &pcl) {
	std::list<geometry_msgs::Point32> group, points(pcl->points.begin(), pcl->points.end());
	std::vector<std::list<geometry_msgs::Point32>> groups;
	groups.reserve(200);

	ROS_INFO("Iterating over point cloud:");
	ROS_INFO("\tpcl.size() => %d", (int)points.size());

	last_received_pcl = pcl->header.stamp;
	pcl_frame = pcl->header.frame_id;

	for(auto point = points.begin(), last_point = points.begin(); points.size() > min_group_count;) {
		if(group.empty()) {
			group.push_back(*point);
			last_point = point;
			point = find_next_point(point, points.end());
			points.erase(last_point);
		} else {
			if(point != points.end()) {
				group.push_back(*point);
				last_point = point;
				point = find_next_point(point, points.end());
				points.erase(last_point);
			} else {
				groups.push_back(group);
				group.clear();
				point = points.begin();
			}
		}	
	}

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

	ROS_INFO("\tFound %d groups, consisting of %d points", (int)groups.size(), point_count);

	find_valid_ranges(groups);
}
		
void object_detector::find_valid_ranges(std::vector<std::list<geometry_msgs::Point32>> groups) {
	auto prev_group = groups.end();
	tf::StampedTransform tform;
	ohm_igvc_msgs::RangeArray ranges;
	ohm_igvc_msgs::Range range;

	try {
		pose_listener.lookupTransform("base", "world", ros::Time::now(), tform);

		// start here
		geometry_msgs::Point32 position;
		position.x = tform.getOrigin().x();
		position.y = tform.getOrigin().y();
		double heading = tf::getYaw(tform.getRotation()) * (180.0 / boost::math::double_constants::pi) + 180.0;

		double last_dist = 0.0, last_angle, this_angle, this_dist;
	
		for(auto group = groups.begin(); group != groups.end(); ++group) {
			for(auto point = group->begin(); point != group->end(); ++point) {
				this_angle = geometric::angular_distance(position, *point);
				this_dist = geometric::distance(*point, position);

				if(this_dist > min_obstacle_distance) {
					if(last_dist < min_obstacle_distance) {						
						range.start = (last_dist > 0 ? circular_range::average(this_angle, last_angle) : this_angle);
					} else {
						range.end = this_angle;
						if(std::next(group, 1) == groups.end() && std::next(point, 1) == group->end()) {
							ranges.ranges.push_back(range);
						}
					}
				} else {
					range.end = (last_dist > 0 ? circular_range::average(this_angle, last_angle) : this_angle);
					ranges.ranges.push_back(range);
				} 
				
				last_dist = this_dist;
				last_angle = this_angle;
			}
		}
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
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
		if(distance<geometry_msgs::Point32>(*next_point, *current) < eps) {
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
