#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

#include "LidarSegment2d.h"
#include "MinkowskiSum2d.h"

// Read in LIDAR data when it is available. Use new_laser
// flag to prevent reading on every loop since sensor only
// outputs data at 5.5Hz
bool new_laser = true;
sensor_msgs::LaserScan laser_in;
void laser_callback(const sensor_msgs::LaserScan& laser_input) {
	laser_in = laser_input;
	new_laser = true;
}  // laser_callback

// Sets up the blue sphere representing the quad in rviz
void SetupQuadVisualization(visualization_msgs::Marker& quad_dummy, int id) {
	quad_dummy.header.frame_id = "world";
	quad_dummy.id = id;
	quad_dummy.type = visualization_msgs::Marker::SPHERE;
	quad_dummy.action = visualization_msgs::Marker::ADD;
	quad_dummy.scale.x = 0.282*2.0f;
	quad_dummy.scale.y = 0.282*2.0f;
	quad_dummy.scale.z = 0.282*2.0f;
	quad_dummy.color.a = 0.5;
	quad_dummy.color.b = 1.0;
}

// Show every lidar point as a red dot in rviz
void SetupFullLaserVisualization(visualization_msgs::Marker& full_laser,
																 int id) {
	full_laser.header.frame_id = "world";
	full_laser.id = id;
	full_laser.type = visualization_msgs::Marker::POINTS;
	full_laser.action = visualization_msgs::Marker::ADD;
	full_laser.scale.x = 0.04;
	full_laser.scale.y = 0.04;
	full_laser.color.a = 1.0;
	full_laser.color.r = 1.0;
}

// Show only the segmented vertices as white dots in rviz
void SetupSegmentedLaserVisualization(visualization_msgs::Marker& seg_laser,
																			int id) {
	seg_laser.header.frame_id = "world";
	seg_laser.id = id;
	seg_laser.type = visualization_msgs::Marker::POINTS;
	seg_laser.action = visualization_msgs::Marker::ADD;
	seg_laser.scale.x = 0.05;
	seg_laser.scale.y = 0.05;
	seg_laser.color.a = 1.0;
	seg_laser.color.r = 1.0;
	seg_laser.color.g = 1.0;
	seg_laser.color.b = 1.0;
}

// Show black lines between the segmented vertices, representing the walls the
// quadrotor sees
void SetupSegmentedLinesVisualization(visualization_msgs::Marker& laser_lines,
																			int id) {
	laser_lines.header.frame_id = "world";
	laser_lines.id = id;
	laser_lines.type = visualization_msgs::Marker::LINE_STRIP;
	laser_lines.action = visualization_msgs::Marker::ADD;
	laser_lines.scale.x = 0.035;
	laser_lines.color.a = 1.0;
}

// Show the vertices of the Minkowski sum as white dots
void SetupMinkowskiPointsVisualization(visualization_msgs::Marker& mink_points,
																			 int id) {
	mink_points.header.frame_id = "world";
	mink_points.id = id;
	mink_points.type = visualization_msgs::Marker::POINTS;
	mink_points.action = visualization_msgs::Marker::ADD;
	mink_points.scale.x = 0.075;
	mink_points.scale.y = 0.075;
	mink_points.color.a = 1.0;
  mink_points.color.g = 1.0;
}

// Show the lines between the Minkowski vertices as a black line
void SetupMinkowskiLinesVisualization(visualization_msgs::Marker& mink_lines,
																			int id) {
	mink_lines.header.frame_id = "world";
	mink_lines.id = id;
	mink_lines.type = visualization_msgs::Marker::LINE_STRIP;
	mink_lines.action = visualization_msgs::Marker::ADD;
	mink_lines.scale.x = 0.035;
	mink_lines.color.a = 1.0;
	mink_lines.color.b = 1.0;
}

namespace vis {
	typedef Eigen::Vector2f vec2;
	typedef std::vector<Eigen::Vector2f> vec2vec;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "visualiztion_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	ros::Subscriber laser_scan = nh.subscribe("scan", 1, laser_callback);

	// RVIZ publishers
	ros::Publisher quad_pub =
		nh.advertise<visualization_msgs::Marker>("quad_dummy_vis", 0);
	visualization_msgs::Marker quad_dummy;
	SetupQuadVisualization(quad_dummy, 0);

	ros::Publisher full_point_pub =
		nh.advertise<visualization_msgs::Marker>("laser_full_points_vis", 0);
	visualization_msgs::Marker full_laser_points;
	SetupFullLaserVisualization(full_laser_points, 3);

	ros::Publisher seg_point_pub =
		nh.advertise<visualization_msgs::Marker>("laser_seg_points_vis", 0);
	visualization_msgs::Marker seg_laser_points;
	SetupSegmentedLaserVisualization(seg_laser_points, 4);

	ros::Publisher line_pub =
		nh.advertise<visualization_msgs::Marker>("laser_lines_vis",0);
	visualization_msgs::Marker laser_lines;
	SetupSegmentedLinesVisualization(laser_lines, 5);

	ros::Publisher mink_point_pub =
		nh.advertise<visualization_msgs::Marker>("mink_point_vis", 0);
	visualization_msgs::Marker mink_points;
	SetupMinkowskiPointsVisualization(mink_points, 6);

	ros::Publisher mink_line_pub =
		nh.advertise<visualization_msgs::Marker>("mink_line_vis", 0);
	visualization_msgs::Marker mink_lines;
	SetupMinkowskiLinesVisualization(mink_lines, 7);
	
	while (ros::ok()) {
		ros::spinOnce();
		if (new_laser) {
			std::vector<float>range_list;
			vis::vec2vec point_list;

			// Process the lidar scan into 2d points and output raw data
			// to the rviz message
			full_laser_points.points.clear();
			static const float radius = 0.3556 * 0.5;
			float theta = M_PI / 4.0;
			vis::vec2 tmp_point;
			for (auto it = laser_in.ranges.begin(); it != laser_in.ranges.end(); ++it) {
				if (*it <= 6.0 && *it > radius) {
					range_list.push_back(*it);
					tmp_point << (*it)*cos(theta), -(*it)*sin(theta);
					point_list.push_back(tmp_point);

					geometry_msgs::Point point;
					point.x = tmp_point[0];
					point.y = tmp_point[1];
					point.z = 0.02;
					full_laser_points.points.push_back(point);
				}
				theta -= laser_in.angle_increment;
			}
			
			// Segment that data using clustering + split-and-merge
			LidarSegment2d segmented(point_list, range_list, 0.1);
			vis::vec2vec segmented_points =	segmented.segmented_points();

			// Output the segmented data to the rviz messages
			seg_laser_points.points.clear();
			laser_lines.points.clear();
			for (auto it = segmented_points.begin();
					 it != segmented_points.end(); ++it) {
				geometry_msgs::Point point;
				point.x = (*it)[0];
				point.y = (*it)[1];
				point.z = 0.05;
				seg_laser_points.points.push_back(point);
				point.z = 0.0;
				laser_lines.points.push_back(point);
			}

			// Perform the approximate minkowski sum on the data
			MinkowskiSum2d minkowski(segmented_points, radius);
			vis::vec2vec minkowski_points = minkowski.ReturnMinkowskiSum();

			// Output the minkowski data ot the rviz messages
			mink_points.points.clear();
			mink_lines.points.clear();
			for (auto it = minkowski_points.begin(); it != minkowski_points.end(); ++it) {
				geometry_msgs::Point point;
				point.x = (*it)[0];
				point.y = (*it)[1];
				point.z = 0.05;
				mink_points.points.push_back(point);
				point.z = 0.0;
				mink_lines.points.push_back(point);
			}

			// Publish all variables
			quad_pub.publish(quad_dummy);
			full_laser_points.header.stamp = ros::Time();
			full_point_pub.publish(full_laser_points);
			seg_laser_points.header.stamp = ros::Time();
			seg_point_pub.publish(seg_laser_points);
			laser_lines.header.stamp = ros::Time();
			line_pub.publish(laser_lines);
			mink_point_pub.publish(mink_points);
			mink_line_pub.publish(mink_lines);
			new_laser = false;
		}
		loop_rate.sleep();
	}
	return 0;
}

