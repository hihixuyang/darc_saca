#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>

#include "QuadrotorACA2d.h"
#include "QuadrotorACA3d.h"
#include "Obstacle2d.h"
#include "Obstacle3d.h"
#include "wallVertices.h"

Eigen::Vector3f u_goal;
void u_callback(const geometry_msgs::Vector3& u_in) {
	u_goal << u_in.x, u_in.y, u_in.z;
}

float yaw_input;
void yaw_callback(const std_msgs::Float32& yaw_in) {
	yaw_input = yaw_in.data;
}

std::vector<float> scan_data(360);
void laser_callback(const sensor_msgs::LaserScan& laser_in) {
	for (int data_index = 0; data_index < 360; ++data_index) {
		scan_data[data_index] = laser_in.ranges[data_index];
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "darc_saca_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	ros::Subscriber u_sub = nh.subscribe("desired_u", 1, u_callback);
	ros::Subscriber yaw_sub = nh.subscribe("desired_yaw", 1, yaw_callback);
	ros::Subscriber laser_scan = nh.subscribe("laser_scan",1,laser_callback);
	
	ros::Publisher pos_pub =
		nh.advertise<geometry_msgs::Vector3>("/vrep/position", 1);
	ros::Publisher quat_pub =
		nh.advertise<geometry_msgs::Quaternion>("/vrep/quaternion", 1);

	int sim_2d = 0;
	if (nh.getParam("/2d_sim", sim_2d)) {;}
	else {
		ROS_ERROR("Set sim dimension");
		return 0;
	}

	float tau = 1.25;
	QuadrotorACA2d quad_2d(tau);
	QuadrotorACA3d quad_3d(tau);

	if (sim_2d) {
		QuadrotorACA2d::State x0 = QuadrotorACA2d::State::Zero();
		x0[2] = 1.2;
		quad_2d.set_x(x0);
	} else {
		QuadrotorACA3d::State x0 = QuadrotorACA3d::State::Zero();
		x0[2] = 1.2;
		quad_3d.set_x(x0);
	}
	
	std::vector<Vertex> v(9);
	std::vector<Eigen::Vector3f> n(9);
	buildObstacles(v, n, 1, 1);

	srand(time(NULL));
	ROS_ERROR("STARTING LOOP");

	float radius = quad_2d.radius();
	
	while(ros::ok()) {
		ros::spinOnce();
	  std:vector<Obstacle2d> obstacle_list_2d;
	  std:vector<Obstacle3d> obstacle_list_3d
			
		if (sim_2d) {
			std::vector<Obstacle2d> obstacle_list;
			Eigen::Vector2f p = quad.true_position().head(2);
			for (int i = 1; i < 5; ++i) {
				Eigen::Vector2f right = v[i].tr.head(2) - p + quad.sensing_noise();
				Eigen::Vector2f left = v[i].tl.head(2) - p + quad.sensing_noise();
				Eigen::Vector2f normal = (Eigen::Rotation2Df(M_PI/2.0f) * (left - right)).normalized();
				Obstacle2d wall(right, left, normal, radius);
				obstacle_list.push_back(wall);
			}
		} else {
			std::vector<Obstacle3d> obstacle_list;
			Eigen::Vector3f p = quad.true_position;
			for (int i = 0; i < 5; ++i) {
				Eigen::Vector3f noise = quad.sensing_noise();
				Eigen::Vector3f tr = v[i].tr -p + quad.sensing_noise();
				Eigen::Vector3f br = v[i].br -p + quad.sensing_noise();
				Eigen::Vector3f tl = v[i].tl -p + quad.sensing_noise();
				Eigen::Vector3f bl = v[i].bl -p + quad.sensing_noise();
				Eigen::Vector3f normal = (tr - br).cross(bl - br);
				Obstacle3d w_a(tr, br, bl, normal, radius);
				Obstacle3d w_b(tr, tl, bl, normal, radius);
				obstacle_list.push_back(w_a);
				obstacle_list.push_back(w_b);
			}
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
