#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>

#include "darc_saca/PointList.h"
#include "LidarSegment2d.h"

sensor_msgs::LaserScan laser_in;
bool new_laser = false;
void laser_callback(const sensor_msgs::LaserScan& laser_msg) {
  laser_in = laser_msg;
  new_laser = true;
}  // laser_callback

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "data_processing_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
  
  ros::Subscriber laser_scan = nh.subscribe("scan", 1, laser_callback);
  ros::Publisher  segment_pub = nh.advertise<darc_saca::PointList>("segmented_points", 1);

  std::ofstream raw_file, segment_file;
  float theta_rel;

  std::vector<float> range_list;
  std::vector<Eigen::Vector2f> point_list;

  while (ros::ok()) {
    ros::spinOnce();
    if (new_laser) {
      range_list.clear();
      point_list.clear();

      theta_rel = -M_PI / 4.0;// + laser_in.angle_min;
      Eigen::Vector2f tmp_point;
      for (int index = 0; index < laser_in.ranges.size(); ++index) {
        if (laser_in.ranges[index] <= 7.0 && laser_in.ranges[index] > 0) {
          range_list.push_back(laser_in.ranges[index]);
	  tmp_point[0] =  range_list.back()*cos(theta_rel);
          tmp_point[1] = -range_list.back()*sin(theta_rel);
          point_list.push_back(tmp_point);
	}
        theta_rel += laser_in.angle_increment;
      }

      LidarSegment2d lidar_full_points(point_list, range_list, 0.1);
      std::vector<Eigen::Vector2f> lidar_segmented_points = lidar_full_points.segmented_points();

      darc_saca::PointList segmented_points;
      geometry_msgs::Vector3 point;
      for (int index = 0; index < lidar_segmented_points.size(); ++index) {
        point.x = lidar_segmented_points[index][0];
        point.y = lidar_segmented_points[index][1];
        segmented_points.points.push_back(point);
      }
      segment_pub.publish(segmented_points);
      new_laser = false;
    }
    loop_rate.sleep();
  }
  return 0;
}

