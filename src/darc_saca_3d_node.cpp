#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/State.h>

#include "QuadrotorACA3d.h"
#include "LidarSegment2d.h"
#include "MinkowskiSum2d.h"
#include "Obstacle3d.h"
#include "wallVertices.h"

#include <Eigen/Dense>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <numeric>
#include <string>
#include <iterator>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

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

// Read in the reading from the bottom facing sonar
float bottom_sonar_dist = -50.0;
bool new_bottom_sonar = false;
void bottom_sonar_callback(const std_msgs::Float32& range_in) {
  bottom_sonar_dist = range_in.data - 0.15;
  new_bottom_sonar = true;
}  // bottom_sonar_callback

QuadrotorACA3d::Observation observation;
void imu_callback(const sensor_msgs::Imu& imu_in) {
  float x = imu_in.orientation.x;
  float y = imu_in.orientation.y;
  float z = imu_in.orientation.z;
  float w = imu_in.orientation.w;

  observation[0] = atan2(2.0*w*x + 2.0*y*z, 1.0 - 2.0*x*x - 2.0*y*y);  // rx
  observation[1] = asin(2.0*w*y - 2.0*x*z);  // ry
  //observation[2] = atan2(2.0*w*z + 2.0*x*y, 1.0 - 2.0*y*y - 2.0*z*z) + M_PI;  // rz
  observation[2] = 0.0;
  observation[3] = imu_in.angular_velocity.x;  // wx
  observation[4] = imu_in.angular_velocity.y;  // wy
  //observation[5] = imu_in.angular_velocity.z;  // wz
  observation[5] = 0.0;
}  // imu_callback

void vel_callback(const geometry_msgs::TwistStamped& vel_in) {
  observation[6] = vel_in.twist.linear.x;
  observation[7] = vel_in.twist.linear.y;
  observation[8] = vel_in.twist.linear.z;
}  // vel_callback

bool offboard_mode = false;
void mode_callback(const mavros_msgs::State& state) {
  // string compare returns 0 if equal, so not of it AND armed
  // so that the offboard_mode it true only if in offboard, armed
  offboard_mode = !state.mode.compare("OFFBOARD") && state.armed;
}  // mode_callback

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "darc_saca_3d_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(60);

	// desired_u and desired_yaw comes from input mapping node
	ros::Subscriber u_sub = nh.subscribe("desired_u", 1, input_callback);

	// scan comes from rplidar_ros
	ros::Subscriber laser_scan = nh.subscribe("scan", 1, laser_callback);

	// read the bottom sonar reading which will be ADC on hardware
  ros::Subscriber bottom_sonar = nh.subscribe("bottom_lidar_reading", 1, bottom_sonar_callback);

  // Read velocity estimates from pixhawk
  ros::Subscriber vel_sub = nh.subscribe("/mavros/local_position/velocity", 1, vel_callback);

  // Read imu from pixhawk
  ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 1, imu_callback);

  // Read the pixhawk state
  ros::Subscriber mode_sub = nh.subscribe("/mavros/state", 1, mode_callback);

  // Publish the new collision free input
  ros::Publisher u_pub = nh.advertise<geometry_msgs::Twist>("new_u", 1);

  // Publish the desired velocity
  ros::Publisher vz_pub = nh.advertise<std_msgs::Float32>("des_vel", 1);

  // Publish the estimated velocity from the kalman filter
  ros::Publisher est_vel_pub = nh.advertise<geometry_msgs::Twist>("est_vel", 1);

  // Publish the measured orientation for debugging
  ros::Publisher r_pub = nh.advertise<geometry_msgs::Vector3>("meas_r", 1);

  // BEGIN DEBUGGING
  // Publish the trajectories for debugging and points
  ros::Publisher raw_points_pub = nh.advertise<sensor_msgs::PointCloud>("raw_points", 1);
  sensor_msgs::PointCloud raw_points_out;
  raw_points_out.header.frame_id = "map";
  
  ros::Publisher seg_points_pub = nh.advertise<sensor_msgs::PointCloud>("seg_points", 1);
  sensor_msgs::PointCloud seg_points_out;
  seg_points_out.header.frame_id = "map";
    
  ros::Publisher init_traj_pub = nh.advertise<sensor_msgs::PointCloud>("init_traj", 1);
  sensor_msgs::PointCloud init_traj_out;
  init_traj_out.header.frame_id = "map";

  ros::Publisher fin_traj_pub = nh.advertise<sensor_msgs::PointCloud>("fin_traj", 1);
  sensor_msgs::PointCloud fin_traj_out;
  fin_traj_out.header.frame_id = "map";

  ros::Publisher vel_pub = nh.advertise<sensor_msgs::PointCloud>("vel_marker", 1);
  sensor_msgs::PointCloud vel_out;
  vel_out.header.frame_id = "map";  
  // END DEBUGGING

  // Read in the value of hte time horizon from the launch file
	double time_horizon;
	if (nh.getParam("/time_horizon", time_horizon)) {;}	else {
		ROS_ERROR("Set Time Horizon");
		return 0;
	}

	// Read in the distance threshold for the obstacle segmentation from the launch file
	double distance_threshold;
	if (nh.getParam("/dist_thresh", distance_threshold)) {;} else {
		ROS_ERROR("Set Distance Threshold");
		return 0;
	}

	// Read in if collision avoidance or full manual
	double pca_enabled;
	if (nh.getParam("/pca_on", pca_enabled)) {;} else {
		ROS_ERROR("Set collision avoidance flag");
		return 0;
	}

	// Setup the class instance of the quadrotor for collision avoidance
	QuadrotorACA3d quad(time_horizon);

	QuadrotorACA3d::State x0 = QuadrotorACA3d::State::Zero();
	quad.set_x(x0);

	const float radius = quad.radius();

	std::vector<Obstacle3d> obstacle_list;

  std::vector<Eigen::Vector2f> full_point_list;
  std::vector<float> range_list;
  std::vector<Eigen::Vector2f> lidar_segmented_points;
  std::vector<Eigen::Vector2f> minkowski_point_list;

	while(ros::ok()) {
		ros::spinOnce();

		geometry_msgs::Vector3 r_out;
		r_out.x = observation[0];
		r_out.y = observation[1];
		r_out.z = observation[2];
    r_pub.publish(r_out);

    quad.ApplyKalman(observation);  // Update state estimate

    QuadrotorACA3d::State x_hat = quad.x_hat();
    geometry_msgs::Twist est_vel_out;
    est_vel_out.linear.x = x_hat[3];
    est_vel_out.linear.y = x_hat[4];
    est_vel_out.linear.z = x_hat[5];
    est_vel_out.angular.x = x_hat[9];
    est_vel_out.angular.y = x_hat[10];
    est_vel_out.angular.z = x_hat[11];
    est_vel_pub.publish(est_vel_out);

		// Only process the lidar vertices if new data is received from
		// the lidar node, preventing extra computation
		if (new_lidar) {
			full_point_list.clear();  // Full 2d points of the laser for split-and-merge
			range_list.clear();  // list of distances for clustering

			float theta_rel = M_PI / 4.0; //- quad.est_yaw();
			Eigen::Vector2f tmp_point;
			for (std::vector<float>::iterator it = laser_in.ranges.begin();
			     it != laser_in.ranges.end(); ++it) {
			  if ((*it) <= 6.0) {
  				range_list.push_back(*it);
				  tmp_point[0] =  (*it)*sin(theta_rel);
  				tmp_point[1] = -(*it)*cos(theta_rel);
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

      // Run the approximate Minkowski difference on the segmented lidar data
			MinkowskiSum2d minkowski_points(lidar_segmented_points, radius);
			minkowski_point_list.clear();
			//minkowski_point_list = minkowski_points.ReturnMinkowskiSum();
			minkowski_point_list = lidar_segmented_points;

			// BEGIN DEBUGGING
			vel_out.points.clear();
			geometry_msgs::Point32 point;
			for (float i = 0; i < 1.05; i += 0.05) {
  			point.x = i * x_hat[3]; 
  			point.y = i * x_hat[4]; 
  			point.z = i * x_hat[5];
  			vel_out.points.push_back(point);
  	}
			vel_out.header.stamp = ros::Time();
		  vel_pub.publish(vel_out);
 
			raw_points_out.points.clear();
			for (auto it = full_point_list.begin(); it != full_point_list.end(); ++it) {
			  point.x = (*it)[0]; point.y = (*it)[1];	point.z = 0.0;
			  raw_points_out.points.push_back(point);
			}
			raw_points_out.header.stamp = ros::Time();
			raw_points_pub.publish(raw_points_out);

      seg_points_out.points.clear();
      point.z = 0.0;
			for (auto it = lidar_segmented_points.begin(); it != lidar_segmented_points.end(); ++it) {
			  point.x = (*it)[0]; point.y = (*it)[1];
			  seg_points_out.points.push_back(point);
			}
			seg_points_out.header.stamp = ros::Time();
			seg_points_pub.publish(seg_points_out);
      // END DEBUGGING

    }

		if (new_lidar || new_bottom_sonar) {
		  obstacle_list.clear();

      if (new_bottom_sonar) {
        Eigen::Vector3f r_hat = quad.x_hat().segment(6,3);
        bottom_sonar_dist = bottom_sonar_dist * cos(r_hat[0]) * cos(r_hat[1]);
      }

      // If list is empty, don't call prev
      auto end = minkowski_point_list.empty() ?
        minkowski_point_list.end() : std::prev(minkowski_point_list.end());
			for (auto it = minkowski_point_list.begin(); it != end; ++it) {
				// Store segmented lines as obstacles for collision avoidance
				Eigen::Vector3f tr, br, tl, bl;
				tr << (*std::next(it))[0], (*std::next(it))[1], 50.0;
				br << (*std::next(it))[0], (*std::next(it))[1], bottom_sonar_dist - 25.0;
				tl << (*it)[0], (*it)[1], 50.0;
				bl << (*it)[0], (*it)[1], bottom_sonar_dist - 25.0;
				Eigen::Vector3f normal = ((tr - br).cross(bl - br)).normalized();
				Obstacle3d w_a(tr, br, bl, normal);
				Obstacle3d w_b(tr, tl, bl, normal);
				obstacle_list.push_back(w_a);
				obstacle_list.push_back(w_b);
			}

      // BEGIN DEBUGGING
//      obstacle_list.clear();
      // END DEBUGGING
      
      // also store the "floor" as a large obstacles
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

			if (new_lidar) { new_lidar = false; }
      if (new_bottom_sonar) { new_bottom_sonar = false; }
		}

		Eigen::Vector4f u_curr(u_goal[0], u_goal[1], u_goal[2], yaw_input);

		nh.getParam("/pca_on", pca_enabled);
 	  if (quad.AvoidCollisions(u_curr, obstacle_list, pca_enabled)) {
  	    ROS_ERROR("Collision");
    }


    // BEGIN DEBUGGING
    init_traj_out.points.clear();
    std::vector<Eigen::Vector3f> init_traj = quad.InitialDesiredTrajectory();
    geometry_msgs::Point32 point;
    for (auto it = init_traj.begin(); it != init_traj.end(); ++it) {
      point.x = (*it)[0]; point.y = (*it)[1]; point.z = (*it)[2];
      init_traj_out.points.push_back(point);
    }
    init_traj_out.header.stamp = ros::Time();
    init_traj_pub.publish(init_traj_out);

    fin_traj_out.points.clear();
    std::vector<Eigen::Vector3f> fin_traj = quad.FinalDesiredTrajectory();
    for (auto it = fin_traj.begin(); it != fin_traj.end(); ++it) {
      point.x = (*it)[0]; point.y = (*it)[1]; point.z = (*it)[2];
      fin_traj_out.points.push_back(point);
    }
    fin_traj_out.header.stamp = ros::Time();
    fin_traj_pub.publish(fin_traj_out);
    // END DEBUGGING

    Eigen::Vector4f u_new = quad.u();
    geometry_msgs::Twist u_out;
    u_out.angular.x = u_new[0];
    u_out.angular.y = u_new[1];
    u_out.angular.z = yaw_input;

    std_msgs::Float32 vz_out;
    vz_out.data = u_new[2];
    vz_pub.publish(vz_out);

    static const float max_climb_rate = 0.5;  // m/s
    float v_des = max_climb_rate * u_new[2];

    static float err = 0.0;
    err = v_des - quad.x_hat()[5];

    static float int_err = 0.0;
    if (!offboard_mode) {
      int_err = 0.0;
    } else {
      int_err += err;
    }

    // Read in gains of the controller for tuning
    static double Kp;
    nh.getParam("/kp_gain", Kp);
    static double Ki;
    nh.getParam("/ki_gain", Ki);

    static float effort = 0.0;
    effort = 0.35 + Kp * err + Ki * int_err;
    if (effort >=  1.0) {  // Positive windup
      int_err -= err;
      effort = 1.0;
    } else if (effort <= -1.0) {  // Negative windup
      int_err -= err;
      effort = -1.0;
    }

    // DEBUGGING
    double pass;
    nh.getParam("/pass_through", pass);
    if (pass) {
      u_out.linear.z = u_new[2];
    } else {
      u_out.linear.z = effort;
    }

    u_pub.publish(u_out);
		loop_rate.sleep();
	}
	return 0;
}
