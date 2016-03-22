#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

#include "QuadrotorACA3d.h"
#include "LidarSegment2d.h"
#include "MinkowskiSum2d.h"
#include "Obstacle3d.h"
#include "wallVertices.h"
#include "mavros_msgs/BatteryStatus.h"

#include <iostream>
#include <unistd.h>
#include <math.h>
#include <numeric>

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
//	new_lidar = true;
}  // laser_callback

// Read in the reading from the top facing sonar
float top_sonar_dist = 50.0;
bool new_top_sonar = false;
void top_sonar_callback(const std_msgs::Float32& top_in) {
	top_sonar_dist = top_in.data;
//  new_top_sonar = true;
}  // top_sonar_callback

// Read in the reading from the bottom facing sonar
float bottom_sonar_dist = -50.0;
bool new_bottom_sonar = false;
void bottom_sonar_callback(const sensor_msgs::Range& bottom_in) {
 	bottom_sonar_dist = -bottom_in.range; // negative data since it's below quad
  new_bottom_sonar = true;
}  // bottom_sonar_callback

float vx = 0.0, vy = 0.0, vz = 0.0;
void vel_callback(const geometry_msgs::TwistStamped& vel_in) {
  vx = vel_in.twist.linear.x;
  vy = vel_in.twist.linear.y;
  vz = vel_in.twist.linear.z;
}  // vel_callback

float rx = 0.0, ry = 0.0;
float wx = 0.0, wy = 0.0, wz = 0.0;
void imu_callback(const sensor_msgs::Imu& imu_in) {
  float x = imu_in.orientation.x;
  float y = imu_in.orientation.y;
  float z = imu_in.orientation.z;
  float w = imu_in.orientation.w;

  rx = atan2(2.0*y*z - 2.0*x*w, 1.0 - 2.0*x*x - 2.0*y*y);
  ry = atan2(-2.0*(x*z-y*w), sqrt(pow(1.0-2.0*y*y-2.0*z*z,2) + pow(2.0*(x*y-z*w),2)));
  wx = imu_in.angular_velocity.x;
  wy = imu_in.angular_velocity.y;
  wz = imu_in.angular_velocity.z;
}  // imu_callback

float voltage = 0.0;
void voltage_callback(const mavros_msgs::BatteryStatus& battery) {
  voltage = battery.voltage;
}  // voltage_callback

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "darc_saca_3d_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);

	// desired_u and desired_yaw comes from input mapping node
	ros::Subscriber u_sub = nh.subscribe("desired_u", 1, input_callback);

	// scan comes from rplidar_ros
	ros::Subscriber laser_scan = nh.subscribe("scan",1,laser_callback);

	// read the top and bottom sonar readings which will be ADC on hardware
	ros::Subscriber top_sonar = nh.subscribe("top_sonar_reading",1,
																					 top_sonar_callback);
	ros::Subscriber bottom_sonar = nh.subscribe("/mavros/px4flow/ground_distance",1,
																							bottom_sonar_callback);

  // Read velocity estimates from pixhawk
  ros::Subscriber vel_sub = nh.subscribe("/mavros/local_position/velocity", 1, vel_callback);

  // Read imu from pixhawk
  ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 1, imu_callback);

  // Read batter from pixhawk
  ros::Subscriber volt_sub = nh.subscribe("/mavros/battery", 1, voltage_callback);

  // Publish the new collision free input
  ros::Publisher u_pub = nh.advertise<geometry_msgs::Twist>("new_u", 1);

  // Publish the estimated state for debugging
  ros::Publisher p_pub = nh.advertise<geometry_msgs::Twist>("est_pos", 1);
  ros::Publisher v_pub = nh.advertise<geometry_msgs::Twist>("est_vel", 1);

  // Publish the desired velocity
  ros::Publisher vz_pub = nh.advertise<std_msgs::Float32>("des_vel", 1);

  // Read in the value of hte time horizon from the launch file
	double time_horizon;
	if (nh.getParam("/time_horizon", time_horizon)) {;}
	else {
		ROS_ERROR("Set Time Horizon");
		return 0;
	}

	// Read in the distance threshold for the obstacle segmentation from the launch file
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

  QuadrotorACA3d::State x_hat = QuadrotorACA3d::State::Zero();

  std::vector<Eigen::Vector2f> full_point_list;
  std::vector<float> range_list;
  std::vector<Eigen::Vector2f> lidar_segmented_points;
  std::vector<Eigen::Vector2f> minkowski_point_list;

	while(ros::ok()) {
		ros::spinOnce();

    // Read from sensors and update state estimate
    Eigen::Matrix<float,8,1> z;
    z[0] = rx; z[1] = ry;
    z[2] = wx; z[3] = wy; z[4] = wz;
    z[5] = vx; z[6] = vy; z[7] = vz;
    quad.ApplyKalman(z);

    QuadrotorACA3d::State x_hat = quad.x_hat();
    geometry_msgs::Twist p_out, v_out;
    p_out.linear.x = x_hat[0];
    p_out.linear.y = x_hat[1];
    p_out.linear.z = x_hat[2];
    v_out.linear.x = x_hat[3];
    v_out.linear.y = x_hat[4];
    v_out.linear.z = x_hat[5];
    p_out.angular.x = x_hat[6];
    p_out.angular.y = x_hat[7];
    p_out.angular.z = x_hat[8];
    v_out.angular.x = x_hat[9];
    v_out.angular.y = x_hat[10];
    v_out.angular.z = x_hat[11];
    p_pub.publish(p_out);
    v_pub.publish(v_out);

		// Only process the lidar vertices if new data is received from
		// the lidar node, preventing extra computation
		if (new_lidar) {
			// Full 2d points of the laser for split-and-merge
			full_point_list.clear();

			// Full list of distances for clustering
			range_list.clear();

			//full_laser_points.points.clear();
			float theta_rel = -M_PI / 4.0;
			Eigen::Vector2f tmp_point;
			for (std::vector<float>::iterator it = laser_in.ranges.begin();
			     it != laser_in.ranges.end(); ++it) {
			  if ((*it) <= 7.0) {
  				range_list.push_back(*it);
				  tmp_point[0] =  (*it)*cos(theta_rel);
  				tmp_point[1] = -(*it)*sin(theta_rel);
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
/*
    if (new_bottom_sonar) {
			bottom_sonar_dist = (bottom_sonar_dist + radius);
    }
*/
    if (new_top_sonar) {
      top_sonar_dist = (top_sonar_dist - radius);
    }

		if (new_lidar || new_top_sonar || new_bottom_sonar) {
		  obstacle_list.clear();

			for (int index = 1; 0 && index < minkowski_point_list.size(); ++index) {
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
/*
			tr <<  50.0,  50.0, top_sonar_dist;
			br << -50.0,  50.0, top_sonar_dist;
			tl <<  50.0, -50.0, top_sonar_dist;
			bl << -50.0, -50.0, top_sonar_dist;
      normal << 0,0,-1;
			Obstacle3d ceil_a(tr, br, bl, normal);
			Obstacle3d ceil_b(tr, tl, bl, normal);
			obstacle_list.push_back(ceil_a);
	    obstacle_list.push_back(ceil_b);
*/

			if (new_lidar) { new_lidar = false; }
      if (new_bottom_sonar) { new_bottom_sonar = false; }
      if (new_top_sonar) { new_top_sonar = false; }
		}

		Eigen::Vector4f u_curr(u_goal[0], u_goal[1], u_goal[2], yaw_input);

		nh.getParam("/pca_on", pca_enabled);
    quad.SetVoltage(voltage);
    if (bottom_sonar_dist < -0.1) {
  		if (quad.AvoidCollisions(u_curr, obstacle_list, pca_enabled)) {
  		  ROS_ERROR("Collision");
  		}
    }

    Eigen::Vector4f u_new = quad.u();
    geometry_msgs::Twist u_out;
    u_out.angular.x = u_new[0];
    u_out.angular.y = u_new[1];
    u_out.angular.z = yaw_input;
    //u_out.linear.z = u_new[2];

    static const float max_climb_rate = 0.5;  // m/s
    u_new[2] = max_climb_rate * u_new[2];
    std_msgs::Float32 vz_out;
    vz_out.data = u_new[2];
    vz_pub.publish(vz_out);

    static const float Kp = 2.0, Ki = 0.05;
    float err = u_new[2] - x_hat[5];
    static float int_err = 0.0;
    int_err += err;
    float effort = Kp * err + Ki * int_err;
    if (effort >=  1.0) {  // Positive windup
      int_err -= err;
      effort = 1.0;
    } else if (effort <= -1.0) {  // Negative windup
      int_err -= err;
      effort = -1.0;
    }
    u_out.linear.z = effort;

    u_pub.publish(u_out);
		loop_rate.sleep();
	}
	return 0;
}
