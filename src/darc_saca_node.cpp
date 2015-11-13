#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>

#include "QuadrotorACA3d.h"
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

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "darc_saca_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	ros::Subscriber u_sub = nh.subscribe("desired_u", 1, u_callback);
	ros::Subscriber yaw_sub = nh.subscribe("desired_yaw", 1, yaw_callback);

	ros::Publisher pos_pub =
		nh.advertise<geometry_msgs::Vector3>("/vrep/position", 1);
	ros::Publisher quat_pub =
		nh.advertise<geometry_msgs::Quaternion>("/vrep/quaternion", 1);

	QuadrotorACA3d quad(1.0);
	
	QuadrotorACA3d::State x0 = QuadrotorACA3d::State::Zero();
	x0[2] = 1.2;
	quad.set_x(x0);
	
	std::vector<Vertex> v(9);
	std::vector<Eigen::Vector3f> n(9);
	buildObstacles(v, n, 1, 1);

	srand(time(NULL));
	ROS_ERROR("STARTING LOOP");

	float radius = quad.radius();
	
	while(ros::ok()) {
		ros::spinOnce();
		std::vector<Obstacle3d> obstacle_list;
		Eigen::VectorXf p = quad.true_position();

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
		
		Eigen::Vector4f u_curr(u_goal[0], u_goal[1], u_goal[2], yaw_input);
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
