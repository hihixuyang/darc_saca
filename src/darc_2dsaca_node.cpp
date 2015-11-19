#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>

#include "QuadrotorACA2d.h"
#include "Obstacle2d.h"
#include "LidarSegment.h"
#include "wallVertices.h"

Eigen::Vector3f u_goal;
void u_callback(const geometry_msgs::Vector3& u_in) {
	u_goal << u_in.x, u_in.y, u_in.z;
}  // u_callback

float yaw_input;
void yaw_callback(const std_msgs::Float32& yaw_in) {
	yaw_input = yaw_in.data;
}  // yaw_callback

std::vector<Eigen::Vector2f> full_point_list;
std::vector<float> range_list;
bool new_data = true;
void laser_callback(const sensor_msgs::LaserScan& laser_in) {
	full_point_list.clear();
	full_point_list.resize(laser_in.ranges.size());
	range_list.clear();
	range_list.resize(laser_in.ranges.size());
	float theta = laser_in.angle_min;
	Eigen::Vector2f tmp_point;
	for (int data_index = 0; data_index < laser_in.ranges.size();
			 ++data_index) {
		range_list[data_index] = laser_in.ranges[data_index];
		tmp_point[0] = laser_in.ranges[data_index]*cos(theta);
		tmp_point[1] = -laser_in.ranges[data_index]*sin(theta);
		full_point_list[data_index] = tmp_point;
		theta += laser_in.angle_increment;
	}
	new_data = true;
}  // laser_callback


int main(int argc, char* argv[]) {
	ros::init(argc, argv, "darc_2dsaca_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	ros::Subscriber u_sub = nh.subscribe("desired_u", 1, u_callback);
	ros::Subscriber yaw_sub = nh.subscribe("desired_yaw", 1, yaw_callback);
	ros::Subscriber laser_scan = nh.subscribe("laser_scan",1,laser_callback);
	
	ros::Publisher pos_pub =
		nh.advertise<geometry_msgs::Vector3>("/vrep/position", 1);
	ros::Publisher quat_pub =
		nh.advertise<geometry_msgs::Quaternion>("/vrep/quaternion", 1);

	double time_horizon;
	if (nh.getParam("/time_horizon", time_horizon)) {;}
	else {
		ROS_ERROR("Set Time Horizon");
		return 0;
	}
	QuadrotorACA2d quad(time_horizon);
	
	QuadrotorACA2d::State x0 = QuadrotorACA2d::State::Zero();
	x0[0] = 1.0;
	x0[1] = -1.0;
	x0[2] = 1.2;
	quad.set_x(x0);
	
	std::vector<Vertex> v(9);
	std::vector<Eigen::Vector3f> n(9);
	buildObstacles(v, n, 1, 1);

	srand(time(NULL));
	ROS_ERROR("STARTING LOOP");

	float radius = quad.radius();

	double distance_threshold;
	if (nh.getParam("/dist_thresh", distance_threshold)) {;}
	else {
		ROS_ERROR("Set Distance Threshold");
		return 0;
	}
	
	std::vector<Obstacle2d> obstacle_list;
	while(ros::ok()) {
		ros::spinOnce();

		if (new_data) {
			// Make Point List Noisy
			/*
			for (int lidar_index = 0; lidar_index < full_point_list.size(); ++lidar_index) {
				full_point_list[lidar_index] += quad.sensing_noise();
			}
			*/
			LidarSegment lidar_full_points(full_point_list, range_list, distance_threshold);
			std::vector<Eigen::Vector2f> lidar_segmented_points = lidar_full_points.segmented_points();

			obstacle_list.clear();
			for (int index = 1; index < lidar_segmented_points.size(); ++index) {
				Eigen::Vector2f right = lidar_segmented_points[index];
				Eigen::Vector2f left = lidar_segmented_points[index - 1];
				Eigen::Vector2f normal = (Eigen::Rotation2Df(M_PI/2.0f) * (left - right)).normalized();
				Obstacle2d wall(right, left, normal, radius);
				obstacle_list.push_back(wall);
			}
			new_data = false;
		}
		
		Eigen::Vector4f u_curr(u_goal[0], u_goal[1], u_goal[2], yaw_input);
		//Eigen::Vector4f u_curr(1.0, 0.0, 0.0, 0.0);
		quad.AvoidCollisions(u_curr, obstacle_list);
		quad.ApplyInput();

		Eigen::Vector3f pos_curr = quad.true_position();
		geometry_msgs::Vector3 pos_out;
		pos_out.x = pos_curr[0];
		pos_out.y = pos_curr[1];
		pos_out.z = pos_curr[2];
		pos_pub.publish(pos_out);
		
		Eigen::Quaternionf q_curr = quad.true_quaternion();
		geometry_msgs::Quaternion quat_out;
		quat_out.w = q_curr.w();
		quat_out.x = q_curr.x();
		quat_out.y = q_curr.y();
		quat_out.z = q_curr.z();
		quat_pub.publish(quat_out);
		
		loop_rate.sleep();
	}
	return 0;
}
