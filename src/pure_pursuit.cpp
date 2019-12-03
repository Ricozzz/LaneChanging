#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include "math.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>


const float LOOK_AHEAD = 1.3;
const float SPEED_MULTIPLIER = 2;
const std::string WAYPOINT_FILEPATH = "/home/zan/final.csv";///home/zan/wp.csv



class PurePursuit {
private:
	ros::NodeHandle n;
    // TODO: create ROS subscribers and publishers
	ros::Subscriber pose;
	ros::Publisher drive;
    ros::Publisher vis_pub;
    std::string POSE_TOPIC = "/odom";///gt_pose
    std::string DRIVE_TOPIC = "/nav";
    //fstream fin;
    std::vector<std::vector<float>> waypoint;
    visualization_msgs::Marker marker;

public:
	PurePursuit() {
		waypoint = getWaypoints(WAYPOINT_FILEPATH);
		n = ros::NodeHandle();

        // TODO: create ROS subscribers and publishers
		pose = n.subscribe(POSE_TOPIC, 1, &PurePursuit::pose_callback, this);
		drive = n.advertise<ackermann_msgs::AckermannDriveStamped>(DRIVE_TOPIC, 10);
		vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

	
	}

	void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {

		float curr_x = pose_msg->pose.pose.position.x;
		float curr_y = pose_msg->pose.pose.position.y;
		
		std::vector<float> best_point = getBestPoint(curr_x, curr_y);

		float x = best_point[0];
		float y = best_point[1];
		
	////////////////////////////////////////////////

		
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "11";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;	
		for(int j=0; j<2; j++){
			for(int i=0;i<waypoint.size();i++){
				marker.id = i;
				marker.header.stamp = ros::Time::now();
				marker.pose.position.x = waypoint[i][0];
				marker.pose.position.y = waypoint[i][1];
				//vis_pub.publish(marker);
			}
		}
		marker.header.stamp = ros::Time::now();
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		vis_pub.publish(marker);

    /////////////////////////////////////////////////////////////////
    /// Transform points

		double quatx= pose_msg->pose.pose.orientation.x;
		double quaty= pose_msg->pose.pose.orientation.y;
		double quatz= pose_msg->pose.pose.orientation.z;
		double quatw= pose_msg->pose.pose.orientation.w;

		tf::Quaternion q(quatx, quaty, quatz, quatw);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		float vehicle_x, vehicle_y;

		Eigen::Matrix2f R;
		Eigen::Vector2f x_c;
		Eigen::Vector2f x_v;
		Eigen::Vector2f x_m;

		R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);

		x_m << x,y;
		x_c << curr_x, curr_y;

		x_v = R.inverse()*(x_m - x_c);

		vehicle_x = x_v(0);
		vehicle_y = x_v(1);

        // Calculate curvature/steering angle
		float angle;
		float curvature;
		float p = 1/M_PI;

		curvature = 2 * vehicle_y / (pow(vehicle_y, 2) + pow(vehicle_x, 2));//abs(vehicle_y)?
		angle = p * curvature;

		ROS_INFO("x_v, x_y, yaw = %f, %f, %f", vehicle_x, vehicle_y, yaw);

		ROS_INFO("the angle is:%f, curvature:%f", angle, curvature);
	        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
		float velocity;
		if(angle > 0.4189){angle = 0.4189;}
		if(angle < -0.4189){angle = -0.4189;}

		velocity = .5*SPEED_MULTIPLIER;
		if (abs(angle) < (10*3.14/180)){
			velocity = 2*SPEED_MULTIPLIER;
		}
		if (abs(angle) < (20*3.14/180) && abs(angle) > (10*3.14/180)){
			velocity = 1.0*SPEED_MULTIPLIER;
		}

	navigation(angle, velocity);
	}

	void navigation(float angle, float velocity){
		ackermann_msgs::AckermannDriveStamped drv;
		drv.header.frame_id = "pure";
		drv.drive.steering_angle = angle;
		drv.drive.speed = velocity;
		ROS_INFO("the speed is x:%f, angle:%f", velocity, angle);
		drive.publish(drv);
	}


	std::vector<std::vector<float>> getWaypoints(std::string filepath){
		std::vector<std::vector<float>> waypoints;

		std::ifstream fin(filepath);
		if (!fin.good()){
			ROS_INFO("Waypoint File Not Found!!");
		}
		std::string line;
		while(getline(fin, line)){
			std::istringstream sin(line);
			std::vector<float> row;
			std::string wp;
			while(getline(sin, wp, ',')){
				row.push_back(atof(wp.c_str()));
			}

			waypoints.push_back(row);

		}
		ROS_INFO("there are total %lu groups of waypoints", waypoint.size());
		return waypoints;
	}

	std::vector<float> getBestPoint(float curr_x, float curr_y){
	

		//find the closest point
		int min_index;
		float min_dist = 10000000;

		for(int i=0; i<waypoint.size();i++){
			int dist = pow((curr_x-waypoint[i][0]), 2) + pow((curr_y-waypoint[i][1]), 2);
			if(dist < min_dist){
				min_index = i;
				min_dist = dist;
			}
		}

		ROS_INFO("the closest point is: %i, with x:%f, y:%f", min_index,waypoint[min_index][0],waypoint[min_index][1]);

		//find the most look ahead point
		float x, y;
		int search = 0;
		bool cross_zero = false;
		for(int i=0;i<waypoint.size();i++){
		    //search = index+i;
		    //if exactly the look ahead distance
			
		//float ddd = pow(waypoint[index+i][0]-curr_x,2)+pow(waypoint[index+i][1]-curr_y,2);
		//ROS_INFO("loop is: %i, x:%f,y:%f,dist:%f",index+i, x,y, ddd);
			if(pow(waypoint[search][0]-curr_x,2)+pow(waypoint[search][1]-curr_y,2) == pow(LOOK_AHEAD,2)){
			//index = min_index + i;
				x = waypoint[search][0];
				y = waypoint[search][1];
				break;
			}
		    //if the best is some point between two points
			if((pow(waypoint[search][0]-curr_x,2)+pow(waypoint[search][1]-curr_y,2) <= pow(LOOK_AHEAD,2)) && (pow(waypoint[search+1][0]-curr_x,2)+pow(waypoint[search+1][1]-curr_y,2) >= pow(LOOK_AHEAD,2))){
			//index = min_index + i;
				x = (waypoint[search][0] + waypoint[search+1][0])/2;
				y = (waypoint[search][1] + waypoint[search+1][1])/2;
				break;
			}
		        search = min_index+i;
		}
		ROS_INFO("the look ahead point is x:%f, y:%f", x, y);
		std::vector<float> best_point;
		best_point.push_back(x);
		best_point.push_back(y);
		return best_point;
	}
};


int main(int argc, char ** argv) {
	
	
	

	ros::init(argc, argv, "pure_pursuit_node");
	PurePursuit pp;
	ros::spin();
	return 0;
}

