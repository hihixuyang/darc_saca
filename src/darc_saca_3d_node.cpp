#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>

#include "QuadrotorACA3d.h"
#include "LidarSegment2d.h"
#include "MinkowskiSum2d.h"
#include "Obstacle3d.h"
#include "wallVertices.h"
#include "darc_saca/State.h"

#include <iostream>
#include <unistd.h>

// Read in the desired input from the RC controller
Eigen::Vector3f u_goal;
float yaw_input;
void input_callback(const geometry_msgs::Twist& twist_in) {
  u_goal << twist_in.angular.x, twist_in.angular.y, twist_in.linear.z;
  yaw_input = twist_in.angular.z;
}  // input_callback

// Read in LIDAR data when it is available. Use new_data
// flag to prevent reading on every loop since sensor only
// outputs data at 5.5Hz
bool new_lidar = false;
sensor_msgs::LaserScan laser_in;
void laser_callback(const sensor_msgs::LaserScan& laser_input) {
	laser_in = laser_input;
	new_lidar = true;
}  // laser_callback

// Read in the reading from the top facing sonar
float top_sonar_dist = 50.0;
bool new_top_sonar = false;
void top_sonar_callback(const std_msgs::Float32& top_in) {
	top_sonar_dist = top_in.data;
  new_top_sonar = true;
}  // top_sonar_callback

// Read in the reading from the bottom facing sonar
float bottom_sonar_dist = -50.0;
bool new_bottom_sonar = false;
void bottom_sonar_callback(const std_msgs::Float32& bottom_in) {
	bottom_sonar_dist = -bottom_in.data; // negative data since it's below quad
  new_bottom_sonar = true;
}  // bottom_sonar_callback

// Read in IMU data form mavros
float rx, ry;
void imu_callback(const sensor_msgs::Imu& imu_in) {
  rx = 0.0;  // TODO QUATERNION TO RPY
  ry = 0.0;  // TODO QUATERNION TO RPY
}  // imu_callback

float wx, wy, wz, vx, vy, vz;
void vel_callback(const geometry_msgs::TwistStamped& vel_in) {
  wx = vel_in.twist.angular.x;
  wy = vel_in.twist.angular.y;
  wz = vel_in.twist.angular.z;
  vx = vel_in.twist.linear.x;
  vy = vel_in.twist.linear.y;
  vz = vel_in.twist.linear.z;
}  // vel_callback

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "darc_saca_3d_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	// desired_u and desired_yaw comes from input mapping node
	ros::Subscriber u_sub = nh.subscribe("desired_u", 1, input_callback);

	// scan comes from rplidar_ros
	ros::Subscriber laser_scan = nh.subscribe("scan",1,laser_callback);

	// read the top and bottom sonar readings which will be ADC on hardware
	ros::Subscriber top_sonar = nh.subscribe("top_sonar_reading",1,
																					 top_sonar_callback);
	ros::Subscriber bottom_sonar = nh.subscribe("bottom_sonar_reading",1,
																							bottom_sonar_callback);

  // Read the IMU data from the pixhawk using mavros
  ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 1, imu_callback);

  // Read velocity estimates from pixhawk
  ros::Subscriber vel_sub = nh.subscribe("/mavros/local_position/velocity", 1, vel_callback);

  // Publish the new collision free input
  ros::Publisher u_pub = nh.advertise<geometry_msgs::Twist>("new_u", 1);

  // Publish the estimated state
  ros::Publisher state_pub = nh.advertise<darc_saca::State>("x_hat", 1);

	// Read in the value of hte time horizon from the launch file
	double time_horizon;
	if (nh.getParam("/time_horizon", time_horizon)) {;}
	else {
		ROS_ERROR("Set Time Horizon");
		return 0;
	}

	// Read in the distance threshold for the obstacle segmentation from
	// the launch file
	double distance_threshold;
	if (nh.getParam("/dist_thresh", distance_threshold)) {;}
	else {
		ROS_ERROR("Set Distance Threshold");
		return 0;
	}

	// Read in if collision avoidance or full manual
	double pca_enabled;
	if (nh.getParam("/pca_on", pca_enabled)) {;}
	else {
		ROS_ERROR("Set collision avoidance flag");
		return 0;
	}

	// Setup the class instance of the quadrotor for collision avoidance
	QuadrotorACA3d quad(time_horizon);

	QuadrotorACA3d::State x0 = QuadrotorACA3d::State::Zero();
	quad.set_x(x0);

	srand(time(NULL));

	float radius = quad.radius();

	std::vector<Obstacle3d> obstacle_list;

  sleep(10);

	ROS_ERROR("STARTING LOOP");

  QuadrotorACA3d::State x_hat = QuadrotorACA3d::State::Zero();

  std::vector<Eigen::Vector2f> full_point_list;
  std::vector<float> range_list;
  std::vector<Eigen::Vector2f> lidar_segmented_points;
  std::vector<Eigen::Vector2f> minkowski_point_list;

	while(ros::ok()) {
		ros::spinOnce();

    // Read from sensors and update state estimate
    quad.ApplyKalman(rx, ry, wx, wy, wz, vx, vy, vz);

    // Publish the estimated state for debugging
    Eigen::VectorXf x_hat = quad.x_hat();
    darc_saca::State x_out;
    x_out.position.linear.x = x_hat[0];
    x_out.position.linear.y = x_hat[1];
    x_out.position.linear.z = x_hat[2];
    x_out.velocity.linear.x = x_hat[3];
    x_out.velocity.linear.y = x_hat[4];
    x_out.velocity.linear.z = x_hat[5];
    x_out.position.angular.x = x_hat[6];
    x_out.position.angular.y = x_hat[7];
    x_out.position.angular.z = x_hat[8];
    x_out.velocity.angular.x = x_hat[9];
    x_out.velocity.angular.y = x_hat[10];
    x_out.velocity.angular.z = x_hat[11];
    state_pub.publish(x_out);

		// Only process the lidar vertices if new data is received from
		// the lidar node, preventing extra computation
		if (new_lidar) {
			// Full 2d points of the laser for split-and-merge
			full_point_list.clear();

			// Full list of distances for clustering
			range_list.clear();

			//full_laser_points.points.clear();
			float theta_rel = laser_in.angle_min;

			Eigen::Vector2f tmp_point;
			for (int data_index = 0; data_index < laser_in.ranges.size();
					 ++data_index) {
			  if (laser_in.ranges[data_index] <= 6.0) {
  				range_list.push_back(laser_in.ranges[data_index]);
				  tmp_point[0] =  range_list.back()*cos(theta_rel);
  				tmp_point[1] = -range_list.back()*sin(theta_rel);
  				full_point_list.push_back(tmp_point);
        }
				theta_rel += laser_in.angle_increment;
			}

			// Perform the segmentation algorithm to get the reduced vertex list
			LidarSegment2d lidar_full_points(full_point_list,
																			 range_list,
																			 distance_threshold);
			lidar_segmented_points.clear();
			lidar_segmented_points = lidar_full_points.segmented_points();

			MinkowskiSum2d minkowski_points(lidar_segmented_points, radius);
			minkowski_point_list.clear();
			minkowski_point_list = minkowski_points.ReturnMinkowskiSum(0);
    }

    if (new_bottom_sonar) {
			bottom_sonar_dist = (bottom_sonar_dist + radius) / (cos(x_hat[6])*cos(x_hat[7]));
    }

    if (new_top_sonar) {
      top_sonar_dist = (top_sonar_dist - radius) / (cos(x_hat[6])*cos(x_hat[7]));
    }

		if (new_lidar || new_top_sonar || new_bottom_sonar) {
		  obstacle_list.clear();
		  /*
			for (int index = 1; index < minkowski_point_list.size(); ++index) {
				// Store segmented lines as obstacles for collision avoidance
				Eigen::Vector3f tr, br, tl, bl;
				tr << minkowski_point_list[index][0],
					    minkowski_point_list[index][1],
					    top_sonar_dist;
				br << minkowski_point_list[index][0],
					    minkowski_point_list[index][1],
					    bottom_sonar_dist;
				tl << minkowski_point_list[index - 1][0],
					    minkowski_point_list[index - 1][1],
    					top_sonar_dist;
				bl << minkowski_point_list[index - 1][0],
					    minkowski_point_list[index - 1][1],
					    bottom_sonar_dist;
				Eigen::Vector3f normal = ((tr - br).cross(bl - br)).normalized();
				Obstacle3d w_a(tr, br, bl, normal);
				Obstacle3d w_b(tr, tl, bl, normal);
				obstacle_list.push_back(w_a);
				obstacle_list.push_back(w_b);
			}
      */
      // also store the "floor" and "ceiling" as a large obstacles
			Eigen::Vector3f tr; tr <<  50.0, -50.0, bottom_sonar_dist;
			Eigen::Vector3f br; br << -50.0, -50.0, bottom_sonar_dist;
			Eigen::Vector3f tl; tl <<  50.0,  50.0, bottom_sonar_dist;
			Eigen::Vector3f bl; bl << -50.0,  50.0, bottom_sonar_dist;
      Eigen::Vector3f normal;
      normal << 0,0,1;
			Obstacle3d floor_a(tr, br, bl, normal);
			Obstacle3d floor_b(tr, tl, bl, normal);
			obstacle_list.push_back(floor_a);
			obstacle_list.push_back(floor_b);
			tr <<  50.0,  50.0, top_sonar_dist;
			br << -50.0,  50.0, top_sonar_dist;
			tl <<  50.0, -50.0, top_sonar_dist;
			bl << -50.0, -50.0, top_sonar_dist;
      normal << 0,0,-1;
			Obstacle3d ceil_a(tr, br, bl, normal);
			Obstacle3d ceil_b(tr, tl, bl, normal);
			obstacle_list.push_back(ceil_a);
	    obstacle_list.push_back(ceil_b);
      
			if (new_lidar) { new_lidar = false; }
      if (new_bottom_sonar) { new_bottom_sonar = false; }
      if (new_top_sonar) { new_top_sonar = false; }
		}

		Eigen::Vector4f u_curr(u_goal[0], u_goal[1], u_goal[2], yaw_input);
		nh.getParam("/pca_on", pca_enabled);

    //std::cout << "Run algorithm" << std::endl;
		quad.AvoidCollisions(u_curr, obstacle_list, pca_enabled);

    //std::cout << "Send new input" << std::endl;
    Eigen::Vector4f u_new = quad.u();
    geometry_msgs::Twist u_out;
    u_out.angular.x = u_new[0];
    u_out.angular.y = u_new[1];
    u_out.angular.z = yaw_input;
    u_out.linear.z = quad.GetThrottle();
    u_pub.publish(u_out);

		loop_rate.sleep();
	}
	return 0;
}
