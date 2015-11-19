#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>

#include "QuadrotorACA2d.h"
#include "Obstacle2d.h"
#include "LidarSegment2d.h"
#include "MinkowskiSum2d.h"
#include "wallVertices.h"

// Read in the desired input from the xbox controller
Eigen::Vector3f u_goal;
void u_callback(const geometry_msgs::Vector3& u_in) {
	u_goal << u_in.x, u_in.y, u_in.z;
}  // u_callback

// Reads in yaw, also from xbox controller
float yaw_input;
void yaw_callback(const std_msgs::Float32& yaw_in) {
	yaw_input = yaw_in.data;
}  // yaw_callback

// Read in LIDAR data when it is available. Use new_data
// flag to prevent reading on every loop since sensor only
// outputs data at 5.5Hz
bool new_data = true;
sensor_msgs::LaserScan laser_in;
void laser_callback(const sensor_msgs::LaserScan& laser_input) {
	laser_in = laser_input;
	new_data = true;
}  // laser_callback

// The following functions setup rviz visualization for the quadrotor
void SetupQuadVisualization(visualization_msgs::Marker&, int);
void SetupInitialTrajectoryVisualization(visualization_msgs::Marker&, int);
void SetupFinalTrajectoryVisualization(visualization_msgs::Marker&, int);

// The following functions set up rviz visualization for the lidar data
void SetupFullLaserVisualization(visualization_msgs::Marker&, int);
void SetupSegmentedLaserVisualization(visualization_msgs::Marker&, int);
void SetupSegmentedLinesVisualization(visualization_msgs::Marker&, int);
void SetupMinkowskiPointsVisualization(visualization_msgs::Marker&, int);
void SetupMinkowskiLinesVisualization(visualization_msgs::Marker&, int);

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

	ros::Publisher quad_pub = nh.advertise<visualization_msgs::Marker>("quad_dummy_vis", 0);
	ros::Publisher init_traj_pub = nh.advertise<visualization_msgs::Marker>("init_traj_vis", 0);
	ros::Publisher fin_traj_pub = nh.advertise<visualization_msgs::Marker>("final_traj_vis", 0);
	ros::Publisher full_point_pub = nh.advertise<visualization_msgs::Marker>("laser_full_points_vis", 0);
	ros::Publisher seg_point_pub = nh.advertise<visualization_msgs::Marker>("laser_seg_points_vis", 0);
	ros::Publisher line_pub = nh.advertise<visualization_msgs::Marker>("laser_lines_vis",0);
	ros::Publisher mink_point_pub = nh.advertise<visualization_msgs::Marker>("mink_point_vis", 0);
	ros::Publisher mink_line_pub = nh.advertise<visualization_msgs::Marker>("mink_line_vis", 0);

	visualization_msgs::Marker quad_dummy;
	SetupQuadVisualization(quad_dummy, 0);
	visualization_msgs::Marker initial_trajectory_lines;
	SetupInitialTrajectoryVisualization(initial_trajectory_lines, 1);
	visualization_msgs::Marker final_trajectory_lines;
	SetupFinalTrajectoryVisualization(final_trajectory_lines, 2);
	visualization_msgs::Marker full_laser_points;
	SetupFullLaserVisualization(full_laser_points, 3);
	visualization_msgs::Marker seg_laser_points;
	SetupSegmentedLaserVisualization(seg_laser_points, 4);
	visualization_msgs::Marker laser_lines;
	SetupSegmentedLinesVisualization(laser_lines, 5);
	visualization_msgs::Marker mink_points;
	SetupMinkowskiPointsVisualization(mink_points, 6);
	visualization_msgs::Marker mink_lines;
	SetupMinkowskiLinesVisualization(mink_lines, 7);
	
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

		//std::cout << quad.est_yaw() << std::endl;
		if (new_data) {
			// Read points in 2D space for segmentation
			std::vector<Eigen::Vector2f> full_point_list(laser_in.ranges.size());
			
			// Save range data for clustering 
			std::vector<float> range_list(laser_in.ranges.size());

			// Clear points for visualization
			full_laser_points.points.clear();

			// Set relative angle of the lidar
			float theta = laser_in.angle_min - quad.est_yaw();
			
			Eigen::Vector2f tmp_point;
			for (int data_index = 0; data_index < laser_in.ranges.size();
					 ++data_index) {
				range_list[data_index] = laser_in.ranges[data_index];
				//range_list[data_index] += quad.sensing_noise();
				tmp_point[0] = range_list[data_index]*cos(theta);
				tmp_point[1] = -range_list[data_index]*sin(theta);
				full_point_list[data_index] = tmp_point;

				geometry_msgs::Point laser_point;
				laser_point.x = tmp_point[0];
				laser_point.y = tmp_point[1];
				laser_point.z = 0.02;
				full_laser_points.points.push_back(laser_point);
				
				theta += laser_in.angle_increment;
			}

			LidarSegment2d lidar_full_points(full_point_list, range_list, distance_threshold);
			std::vector<Eigen::Vector2f> lidar_segmented_points = lidar_full_points.segmented_points();

			seg_laser_points.points.clear();
			laser_lines.points.clear();
			mink_points.points.clear();
			mink_lines.points.clear();
			geometry_msgs::Point rviz_point;
			if (lidar_segmented_points.size() > 0) {
				rviz_point.x = lidar_segmented_points[0][0];
				rviz_point.y = lidar_segmented_points[0][1];
				rviz_point.z = 0.05;
				seg_laser_points.points.push_back(rviz_point);
				rviz_point.z = 0.0;
				laser_lines.points.push_back(rviz_point);
			}
			obstacle_list.clear();
			for (int index = 1; index < lidar_segmented_points.size(); ++index) {
				// Show segmented points and lines in visualization
				rviz_point.x = lidar_segmented_points[index][0];
				rviz_point.y = lidar_segmented_points[index][1];
				rviz_point.z = 0.05;
				seg_laser_points.points.push_back(rviz_point);
				rviz_point.z = 0.0;
				laser_lines.points.push_back(rviz_point);

				// Store segmented lines as obstacles for collision avoidance
				Eigen::Vector2f right = lidar_segmented_points[index];
				Eigen::Vector2f left = lidar_segmented_points[index - 1];
				Eigen::Vector2f normal = (Eigen::Rotation2Df(M_PI/2.0f) * (left - right)).normalized();
				Obstacle2d wall(right, left, normal, radius);
				obstacle_list.push_back(wall);	 
			}
			
			quad_pub.publish(quad_dummy);
			full_laser_points.header.stamp = ros::Time();
			full_point_pub.publish(full_laser_points);
			seg_laser_points.header.stamp = ros::Time();
			seg_point_pub.publish(seg_laser_points);
			laser_lines.header.stamp = ros::Time();
			line_pub.publish(laser_lines);
		
			new_data = false;
		}
		
		Eigen::Vector4f u_curr(u_goal[0], u_goal[1], u_goal[2], yaw_input);
		//Eigen::Vector4f u_curr(1.0, 0.0, 0.0, 0.0);
		quad.AvoidCollisions(u_curr, obstacle_list);
		quad.ApplyInput();

		// Visualization trajectories
		geometry_msgs::Point tmp_point;
		std::vector<Eigen::Vector2f> init_trajectory = quad.InitialDesiredTrajectory();
		initial_trajectory_lines.points.clear();
		for (int i = 0; i < init_trajectory.size(); ++i) {
			tmp_point.x = init_trajectory[i][0];
			tmp_point.y = init_trajectory[i][1];
			tmp_point.z = 0.0;
			initial_trajectory_lines.points.push_back(tmp_point);
		}
		initial_trajectory_lines.header.stamp = ros::Time();
		init_traj_pub.publish(initial_trajectory_lines);

		std::vector<Eigen::Vector2f> final_trajectory = quad.FinalDesiredTrajectory();
		final_trajectory_lines.points.clear();
		for (int i = 0; i < final_trajectory.size(); ++i) {
			tmp_point.x = final_trajectory[i][0];
			tmp_point.y = final_trajectory[i][1];
			tmp_point.z = 0.01;
			final_trajectory_lines.points.push_back(tmp_point);
		}
		final_trajectory_lines.header.stamp = ros::Time();
		fin_traj_pub.publish(final_trajectory_lines);
		
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

void SetupInitialTrajectoryVisualization(visualization_msgs::Marker& trajectory_points,
																				 int id) {
	trajectory_points.header.frame_id = "world";
	trajectory_points.id = id;
	trajectory_points.type = visualization_msgs::Marker::LINE_STRIP;
	trajectory_points.action = visualization_msgs::Marker::ADD;
	trajectory_points.scale.x = 0.04;
	trajectory_points.color.a = 1.0;
	trajectory_points.color.r = 1.0;
}

void SetupFinalTrajectoryVisualization(visualization_msgs::Marker& trajectory_points,
																			 int id) {
	trajectory_points.header.frame_id = "world";
	trajectory_points.id = id;
	trajectory_points.type = visualization_msgs::Marker::LINE_STRIP;
	trajectory_points.action = visualization_msgs::Marker::ADD;
	trajectory_points.scale.x = 0.04;
	trajectory_points.color.a = 1.0;
	trajectory_points.color.g = 1.0;
}

void SetupFullLaserVisualization(visualization_msgs::Marker& full_laser_points,
																 int id) {
	full_laser_points.header.frame_id = "world";
	full_laser_points.id = id;
	full_laser_points.type = visualization_msgs::Marker::POINTS;
	full_laser_points.action = visualization_msgs::Marker::ADD;
	full_laser_points.scale.x = 0.04;
	full_laser_points.scale.y = 0.04;
	full_laser_points.color.a = 1.0;
	full_laser_points.color.r = 1.0;
}

	void SetupSegmentedLaserVisualization(visualization_msgs::Marker& seg_laser_points,
																				int id) {
	seg_laser_points.header.frame_id = "world";
	seg_laser_points.id = id;
	seg_laser_points.type = visualization_msgs::Marker::POINTS;
	seg_laser_points.action = visualization_msgs::Marker::ADD;
	seg_laser_points.scale.x = 0.05;
	seg_laser_points.scale.y = 0.05;
	seg_laser_points.color.a = 1.0;
	seg_laser_points.color.r = 1.0;
	seg_laser_points.color.g = 1.0;
	seg_laser_points.color.b = 1.0;
}

void SetupSegmentedLinesVisualization(visualization_msgs::Marker& laser_lines,
																			int id) {
	laser_lines.header.frame_id = "world";
	laser_lines.id = id;
	laser_lines.type = visualization_msgs::Marker::LINE_STRIP;
	laser_lines.action = visualization_msgs::Marker::ADD;
	laser_lines.scale.x = 0.035;
	laser_lines.color.a = 1.0;
}

void SetupMinkowskiPointsVisualization(visualization_msgs::Marker& mink_points,
																			 int id) {
	mink_points.header.frame_id = "world";
	mink_points.id = id;
	mink_points.type = visualization_msgs::Marker::POINTS;
	mink_points.action = visualization_msgs::Marker::ADD;
	mink_points.scale.x = 0.05;
	mink_points.scale.y = 0.05;
	mink_points.color.a = 1.0;
	mink_points.color.r = mink_points.color.g = mink_points.color.b = 0.8;
}
	
void SetupMinkowskiLinesVisualization(visualization_msgs::Marker& mink_lines,
																			int id) {
	mink_lines.header.frame_id = "world";
	mink_lines.id = id;
	mink_lines.type = visualization_msgs::Marker::LINE_STRIP;
	mink_lines.action = visualization_msgs::Marker::ADD;
	mink_lines.scale.x = 0.05;
	mink_lines.scale.y = 0.05;
	mink_lines.color.a = 1.0;
	mink_lines.color.r = mink_lines.color.g = mink_lines.color.b = 0.2;
}
