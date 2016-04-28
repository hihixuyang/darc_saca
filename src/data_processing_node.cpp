#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>

#include "darc_saca/PointList.h"
#include "LidarSegment2d.h"
#include "MinkowskiSum2d.h"

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
  ros::Publisher minkowski_pub = nh.advertise<darc_saca::PointList>("minkowski_points", 1);
  float radius = 0.3556;
  float theta_rel;

  std::vector<float> range_list;
  std::vector<Eigen::Vector2f> point_list;

  while (ros::ok()) {
    ros::spinOnce();

    if (new_laser) {
      range_list.clear();
      point_list.clear();
      theta_rel = M_PI / 4.0;// + laser_in.angle_min;
      Eigen::Vector2f tmp_point;
      for (std::vector<float>::iterator it = laser_in.ranges.begin();
           it != laser_in.ranges.end(); ++it) {
        if ((*it) <= 6.0) {
          range_list.push_back(*it);
					tmp_point[0] =  (*it)*cos(theta_rel);
          tmp_point[1] = -(*it)*sin(theta_rel);
          point_list.push_back(tmp_point);
      	}
        theta_rel -= laser_in.angle_increment;
      }

      LidarSegment2d lidar_full_points(point_list, range_list, 0.1);
      std::vector<Eigen::Vector2f> lidar_segmented_points = lidar_full_points.segmented_points();

      MinkowskiSum2d minkowski(lidar_segmented_points, radius);
      std::vector<Eigen::Vector2f> minkowski_full_points = minkowski.ReturnMinkowskiSum();

      darc_saca::PointList segmented_points;
      geometry_msgs::Vector3 point;
      for (std::vector<Eigen::Vector2f>::iterator it = lidar_segmented_points.begin();
           it != lidar_segmented_points.end(); ++it) {
        point.x = (*it)[0];
        point.y = (*it)[1];
        segmented_points.points.push_back(point);
      }
      segment_pub.publish(segmented_points);

      darc_saca::PointList minkowski_points;
      for (std::vector<Eigen::Vector2f>::iterator it = minkowski_full_points.begin();
           it != minkowski_full_points.end(); ++it) {
         point.x = (*it)[0];
         point.y = (*it)[1];
         minkowski_points.points.push_back(point);
      }
      minkowski_pub.publish(minkowski_points);

      new_laser = false;
    }
    loop_rate.sleep();
  }
  return 0;
}

