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


const float LOOK_AHEAD = 2.2;//1.3
const float SPEED_MULTIPLIER = 1.2;
const std::string WAYPOINT_FILEPATH1 = "/home/zan/wp-002.csv";///home/zan/wp.csv
const std::string WAYPOINT_FILEPATH2 = "/home/zan/wp0035.csv";
float map_resolution_ = 0.05;
double angle;
double yaw;
float map_origin_x_ = -51.224998;
float map_origin_y_ = -51.224998;
float curr_x, curr_y;



class LaneChanging {
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
    std::vector<std::vector<float>> waypoint1;
    std::vector<std::vector<float>> waypoint2;
    visualization_msgs::Marker marker;
	visualization_msgs::Marker chosen_point;
	ros::Publisher chosen_point_pub, free_point_pub_;
	visualization_msgs::Marker free_point;

	std::vector<std::vector<float>> submap;
    std::vector<std::vector<float>> map_global_;
	std::vector<float> distant;
	ros::Subscriber scan_sub_;
	
public:
	LaneChanging() {
		waypoint1 = getWaypoints(WAYPOINT_FILEPATH1);
		waypoint2 = getWaypoints(WAYPOINT_FILEPATH2);
		waypoint = waypoint2;
		n = ros::NodeHandle();

        // TO_DO: create ROS subscribers and publishers
		pose = n.subscribe(POSE_TOPIC, 1, &LaneChanging::pose_callback, this);
		drive = n.advertise<ackermann_msgs::AckermannDriveStamped>(DRIVE_TOPIC, 10);
		vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
		chosen_point_pub = n.advertise<visualization_msgs::Marker>("/chosen_point", 0);
	scan_sub_ = n.subscribe("/scan", 10, &LaneChanging::scan_callback, this);
	free_point_pub_ = n.advertise<visualization_msgs::Marker>("/free_point", 0);
	
	//read form map, global for now
    std::ifstream fin("/home/zan/zhenzan_ws/src/lane_changing/src/map.csv");
    std::string line;
    while(getline(fin, line)){
	std::istringstream sin(line);
	std::vector<float> row;
	std::string wp;
	while(getline(sin, wp, ',')){
	    if(atof(wp.c_str()) <= 0.86){//if it is wall
		row.push_back(0.0);
	    }else{
		row.push_back(1.0);
	    }
	}
	map_global_.push_back(row);
	
    }

	submap.clear();
    		for(int i=0; i<120; i++){
        		std::vector<float> row3;
			for(int j=0; j<120; j++){
	    			row3.push_back(map_global_[1800-60+j][1200-60+i]);
		//if 0,1
	    	//ROS_INFO("point %d, %d is %f", i, j, map_global_[map_y-60+j][map_x-60+i]);
			}
			submap.push_back(row3);
    		}
	}


	void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    
	    // update your occupancy grid
 	   	double min, max;//, angle
	   	int sample_time;
 	   	min = scan_msg -> angle_min;
 		max = scan_msg -> angle_max;
  	  	angle = scan_msg -> angle_increment;
    		sample_time = int((max-min)/angle);
    		//std::vector<float> distant;
    		distant.clear();
    		distant = scan_msg -> ranges;


    		ROS_INFO("yaw %f", yaw);
    		//transform it to the origin submap coordinate
    		Eigen::Matrix2f R;
    		Eigen::Vector2f scan_;
    		Eigen::Vector2f sub_co;

    		R << cos(-yaw), -sin(-yaw), sin(-yaw), cos(-yaw);
    
    		std::vector<Eigen::Vector2f> new_dist;
    		for(int i=0; i<distant.size(); i++){
			scan_ << cos(i*angle) * (distant[i]+0.3), sin(i*angle) * (distant[i]+0.3);
        		sub_co = R.inverse()*scan_;

			new_dist.push_back(sub_co);
   		}

    		ROS_INFO("%f,%f", cos(0) * distant[0], sin(0) * distant[0]);
    		ROS_INFO("%f,%f", new_dist[0](0),new_dist[0](1));
	
	/////////////////////////////////////////////////////////////////////////////////
	//create the submap
		int xx =0;
		int yy = 0;

    		float map_x, map_y;
    		map_x = (abs(map_origin_x_) + curr_x)/map_resolution_;
    		map_y = (abs(map_origin_y_) - curr_y)/map_resolution_;
    		submap.clear();
    		for(int i=0; i<120; i++){
        		std::vector<float> row3;
			for(int j=0; j<120; j++){
	    			row3.push_back(map_global_[map_y-60+j][map_x-60+i]);
		//if 0,1
	    	//ROS_INFO("point %d, %d is %f", i, j, map_global_[map_y-60+j][map_x-60+i]);
			}
			submap.push_back(row3);
    		}



    	for(int j=0; j< new_dist.size(); j++){
		int row = 60-new_dist[j](1)/map_resolution_;
		int col = 60+new_dist[j](0)/map_resolution_;

	//////////////////////////////////////////////////////////////////////
	//update submap
	//mn = 8, 4-115  -4
	//mn = 4, 2-118  -2
	//
		if((row <= 115 && row >= 4) && (col >= 4 && col <= 115)){
			for(int m=0;m<4;m++){
				for(int n=0;n<8;n++){
				//ROS_INFO("row col %d,%d,%d,%d", row,119-(row-2+n), col, 119-(col-2+m));
					if((119-(col-10+m) <= 119 && 119-(col-10+m) >= 0) && (119-(row-10+n) >= 0 && 119-(row-10+n) <= 119)){
						submap[119-(col-2+m)][119-(row-2+n)] = 0.0;
				//submap[row-2+m][col-2+n] = 0.0;
						xx = row;
						yy = col;
				//ROS_INFO("xx, yy %d, %d", xx, yy);
					}
				}
			}
		}
    	}
	ROS_INFO("updated the laser scan");
	//updated = true;

	////////////////////////////////////////////////////////////////////////
	//publish the free point
	free_point.points.clear();
	for(int m=0; m<120; m++){
		for(int n=0; n<120; n++){
				
				//ROS_INFO("submap %d, %d: %f ", m, n, submap[m][n]);
				//ROS_INFO("5");
//ROS_INFO("1111");
			if(submap[m][n]>0.96){
				free_point.id = 4;
				free_point.header.stamp = ros::Time::now();
				geometry_msgs::Point p;
				p.x = curr_x + (m - 60)*map_resolution_;
				p.y = curr_y - (n - 60)*map_resolution_;
				p.z = 0;
	//ROS_INFO("2222");

				free_point.points.push_back(p);
			    }
			}
	}
	free_point_pub_.publish(free_point);

	
}






	void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {

	

		curr_x = pose_msg->pose.pose.position.x;
		curr_y = pose_msg->pose.pose.position.y;
		
		std::vector<float> best_point = getBestPoint(curr_x, curr_y);
		
		float x = best_point[0];
		float y = best_point[1];

	
	/////////////////////////////////////////////////////////////////////
	//check the front point, if it is obstacle, switch waypoint

	int goal_in_sub_x = 60 + (x-curr_x)/map_resolution_;
	int goal_in_sub_y = 60 - (y-curr_y)/map_resolution_;
	ROS_INFO("goal x %d, y %d", goal_in_sub_x, goal_in_sub_y);
	int check = 10;
	ROS_INFO("start switch");
    if(goal_in_sub_x>=0 && goal_in_sub_x<=119 && goal_in_sub_y>=0 && goal_in_sub_y<= 119){

	ROS_INFO("1");
	if( submap[goal_in_sub_x][goal_in_sub_y]<0.96 ){
		ROS_INFO("1.1");
		if(waypoint == waypoint1){
			waypoint = waypoint2;
			ROS_INFO("using waypoint 1");
		}else{
			waypoint = waypoint1;
			ROS_INFO("using waypoint 2");
		}
		ROS_INFO("1.2");
	}
	ROS_INFO("1.3");
    }

/*
	for(int m = goal_in_sub_x-check; m<goal_in_sub_x+check; m++){
		for(int n=goal_in_sub_y-check; n<goal_in_sub_y+check; n++){
			if(m<=119 && m>=0 && n<=119 && n>=0){
				if(submap[m][n]<0.96){
					if(waypoint == waypoint1){
						waypoint = waypoint2;
						ROS_INFO("using waypoint 1");
					}else{
						waypoint = waypoint1;
						ROS_INFO("using waypoint 2");
					}
				}
				
			}
		}
	}
*/

		ROS_INFO("2");
		//recalibrate the goal point
		best_point = getBestPoint(curr_x, curr_y);

		x = best_point[0];
		y = best_point[1];
	
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
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;	
		for(int j=0; j<2; j++){
			for(int i=0;i<waypoint.size();i++){
			    if(i%20 == 0){
				marker.id = i;
				marker.header.stamp = ros::Time::now();
				marker.pose.position.x = waypoint[i][0];
				marker.pose.position.y = waypoint[i][1];
				vis_pub.publish(marker);
			    }
			}
		}
		

		chosen_point.header.frame_id = "map";
		chosen_point.ns = "waypoint_vis";
		chosen_point.type = visualization_msgs::Marker::SPHERE;
		chosen_point.action = visualization_msgs::Marker::ADD;
		chosen_point.pose.position.z = 0;
		chosen_point.pose.orientation.x = 0.0;	
		chosen_point.pose.orientation.y = 0.0;	
		chosen_point.pose.orientation.z = 0.0;	
		chosen_point.pose.orientation.w = 1.0;	
		chosen_point.scale.x = 0.2;	
		chosen_point.scale.y = 0.2;	
		chosen_point.scale.z = 0.2;	
		chosen_point.color.a = 1.0;	
		chosen_point.color.r = 1.0;	
		chosen_point.color.g = 0.0;
		chosen_point.color.b = 0.0;

		chosen_point.header.stamp = ros::Time::now();
		chosen_point.pose.position.x = x;
		chosen_point.pose.position.y = y;
		chosen_point_pub.publish(chosen_point);



		free_point.header.frame_id = "map";
	free_point.ns = "free_point_vis";
	free_point.type = visualization_msgs::Marker::POINTS;//SPHERE;
	free_point.action = visualization_msgs::Marker::ADD;
	//tree_node_point.pose.position.z = 0;
	//tree_node_point.pose.orientation.x = 0.0;	
	//tree_node_point.pose.orientation.y = 0.0;	
	//tree_node_point.pose.orientation.z = 0.0;	
	free_point.pose.orientation.w = 1.0;	
	free_point.scale.x = 0.1;	
	free_point.scale.y = 0.1;	
	free_point.scale.z = 0.1;	
	free_point.color.a = 1.0;	
	free_point.color.r = 0.0;	
	free_point.color.g = 1.0;
	free_point.color.b = 1.0;


    /////////////////////////////////////////////////////////////////
    /// Transform points

		double quatx= pose_msg->pose.pose.orientation.x;
		double quaty= pose_msg->pose.pose.orientation.y;
		double quatz= pose_msg->pose.pose.orientation.z;
		double quatw= pose_msg->pose.pose.orientation.w;

		tf::Quaternion q(quatx, quaty, quatz, quatw);
		tf::Matrix3x3 m(q);
		double roll, pitch;
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
	        // publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
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
		ROS_INFO("there are total %lu groups of waypoints", waypoints.size());
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
	LaneChanging lc;
	ros::spin();
	return 0;
}

