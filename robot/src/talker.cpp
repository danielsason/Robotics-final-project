#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// PCL specific includes
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
// Moveit specific includes
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#define armLength 0.68

ros::Publisher move_pub;

// [(speed > 0) => FWD] || [(speed < 0) => BWD]  
bool move(ros::Publisher publisher, float speed){
	geometry_msgs::Twist base_cmd;	
	base_cmd.linear.y = base_cmd.angular.z = 0;
	base_cmd.linear.x = speed;  
 	publisher.publish(base_cmd);
	return true;
}

// [(angle_speed > 0) => LEFT] || [(angle_speed < 0) => RIGHT]  
bool rotate(ros::Publisher publisher, float angle_speed){
	geometry_msgs::Twist base_cmd;	
	base_cmd.linear.x = base_cmd.linear.y = 0;
	base_cmd.angular.z = angle_speed; 
 	publisher.publish(base_cmd);
	return true;
}

bool rotate(ros::Publisher publisher, float angle_speed, int loops){
	int i = 0;
	while (i < loops){
		geometry_msgs::Twist base_cmd;	
		base_cmd.linear.x = base_cmd.linear.y = 0;
		base_cmd.angular.z = angle_speed; 
	 	publisher.publish(base_cmd);
	 	i++;
	 }
	return true;
}

// [(speed > 0) => FWD] || [(speed < 0) => BWD]
// [(angle_speed > 0) => LEFT] || [(angle_speed < 0) => RIGHT]   
bool moveAndRotate(ros::Publisher publisher, float speed, float angle_speed){
	geometry_msgs::Twist base_cmd;	
	base_cmd.linear.x = speed; 
	base_cmd.linear.y = 0;
	base_cmd.angular.z = angle_speed; 
 	publisher.publish(base_cmd);
	return true;
}

float absf(float x){
	if (x>0)
		return x;
	else
		return -x;
}

float rotateSpeedF(float xCenter){	// Calculates speed factor for rotation
	float xAbs = absf(xCenter);
	float speed;

	if (xAbs > 0.3) speed = 0.2;
	if (xAbs > 0.2 && xAbs <= 0.3) speed = 0.12;
	if (xAbs > 0.1 && xAbs <= 0.2) speed = 0.07;
	if (xAbs > 0.0 && xAbs <= 0.1) speed = 0.03;

	if (xCenter>0)
		return -speed;
	else
		return speed;
	
}

float moveSpeedF(float zCenter){	// Calculates speed factor for moving
	if (zCenter > 3.0) return 0.3;
	if (zCenter > 1.8 && zCenter <= 3.0) return 0.15;
	if (zCenter > 0.05 && zCenter <= 1.8) return 0.07;
}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud){
  int rMax = 255; 
  int rMin = 150; 
  int gMax = 60; 
  int gMin = 0; 
  int bMax = 60; 
  int bMin = 0; 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ()); 

  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax))); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin))); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax))); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin))); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax))); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin))); 

  // build the filter 
  pcl::ConditionalRemoval<pcl::PointXYZRGB>  condrem;
  condrem.setCondition(color_cond);
  condrem.setInputCloud(cloud); 
  condrem.setKeepOrganized(true); 

  // apply filter 
  condrem.filter (*cloud_filtered); 

  // save cloud after filtering 
  if (pcl::io::savePCDFile("filteredCloud.pcd", *cloud_filtered, true) == 0) 
  { 
     printf("Saved filteredCloud.pcd \n");
  } 
  else PCL_ERROR("Problem saving"); 

  printf("\n\n");

  // get range of red pixels in filtered cloud
  float xMax = -1.0 * (cloud->width); 
  float xMin = cloud->width; 
  float yMax = -1.0 * (cloud->height); 
  float yMin = cloud->height; 
  float zMax = 0; 
  float zMin = 161110;
  float xCenter = 7777777;		// Default value for speedF functions
  float yCenter = 0;
  float zCenter = 7777777;		// Default value for speedF functions
  float xFocus;
  float epsilon = 0.05;			// distance between red button and mid of photo
  float numOfRedPoints = 0.0;


  for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		if(cloud_filtered->points[i].x > xMax) xMax = cloud_filtered->points[i].x;
		if(cloud_filtered->points[i].x < xMin) xMin = cloud_filtered->points[i].x;
		if(cloud_filtered->points[i].y > yMax) yMax = cloud_filtered->points[i].y;
		if(cloud_filtered->points[i].y < yMin) yMin = cloud_filtered->points[i].y;
		if(cloud_filtered->points[i].z > zMax) zMax = cloud_filtered->points[i].z;
		if(cloud_filtered->points[i].z < zMin) zMin = cloud_filtered->points[i].z;

	}	

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
	   	 if (cloud_filtered->points[i].x == cloud_filtered->points[i].x)
	   	 	numOfRedPoints++;
   	
   	}
	
	if(numOfRedPoints>0){		//if there is reds points
	  // find center of red points in image
	  	xCenter = (xMax + xMin) / 2.0;
		yCenter = (yMax + yMin) / 2.0;
		zCenter = (zMax + zMin) / 2.0;
	}
	printf("Xmax %f\n", xMax);
	printf("Xmin %f\n", xMin);
	printf("RedPoints %f\n", numOfRedPoints);
    printf("xCenter: %f\n", xCenter);
	printf("zCenter: %f\n", zCenter);
	printf("movSpeed: %f\n", moveSpeedF(zCenter));
	printf("RotateSpeed :%f\n",rotateSpeedF(xCenter));
	xCenter = xCenter + 0.15;
	xFocus = absf(xCenter);

	if(numOfRedPoints > 0){								//if there are reds points
		if (xFocus > epsilon){					// Button is far from image center, rotating and move towards center							
			if (zCenter > armLength)					// Robot is far from the button moving towards button
				moveAndRotate(move_pub, moveSpeedF(zCenter), rotateSpeedF(xCenter));
			else
				rotate(move_pub, rotateSpeedF(xCenter));					// Rotate left
		}
		else {											// when the red vbutton is in the center
			if (zCenter > armLength)			// Robot is far from the button moving towards button
				move(move_pub, moveSpeedF(zCenter));
		}
	}
	else{												// there's no red points - sreaching for red points (rotate)
			rotate(move_pub, rotateSpeedF(xCenter));					// Rotate left
	}
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;
    move_pub = n.advertise<geometry_msgs::Twist>("//cmd_vel", 1);


// First Move the arm to center

	moveit::planning_interface::MoveGroup group("arm");
  	group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");
    group.setStartStateToCurrentState();

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id="base_footprint";
    target_pose.header.stamp=ros::Time::now()+ros::Duration(2.1);
    target_pose.pose.position.x = 0.5;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.9;
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0); ;
    group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    if(success) 
	    group.move();
    sleep(2);


		std::string topic = n.resolveName("kinect/qhd/points");
		uint32_t queue_size = 1;
		ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("kinect2/qhd/points", queue_size, callback);
		ros::spin();


  return 0;
}

