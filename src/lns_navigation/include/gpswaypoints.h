/*
Class to navigate the robot based on GPS points,
this class will read the GPS and orientation points
saved in a txt file and transform them to map points.
2 modes can be set for navigation, Rotation in place 
and Normal (Orientation is calculated based as half 
of the angle between the destination and next 
destination point).


Subscribe to: /odometry/filtered


Publish to: /waypoint_following_status

by Pablo
Last review: 2023/07/10

TODO: Find a better way to provide the orientation
      independent of the robot position when the 
      map is initialized.

*/


#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <tuple>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <math.h>


class GpsWaypoints
{
private:
	// ROS
	ros::NodeHandle n_;
	ros::Publisher pubWaypointNodeEnded_;
	ros::Subscriber odomfiltered_;
	ros::AsyncSpinner spinner_;

	//create a type definition for a client called MoveBaseClient
	typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient; 
	MoveBaseClient ac_;

	// Read file
	std::string path_abs_;
	int numWaypoints_ = -1;
	
	// Waypoint vector lat, long, orientation
	std::vector <std::tuple<double, double, double> > waypointVect_;
	// Current pose of the robot
	geometry_msgs::PointStamped map_pose_;
	
	// Methods
	void getWaypoints(void);
	void countWaypointsInFile(void);
	geometry_msgs::PointStamped latLongtoUTM(double, double);
	geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped, geometry_msgs::PointStamped*);
	geometry_msgs::PointStamped MaptoOdom(geometry_msgs::PointStamped, geometry_msgs::PointStamped*);
	move_base_msgs::MoveBaseGoal translation(geometry_msgs::PointStamped, double);
	move_base_msgs::MoveBaseGoal rotation(geometry_msgs::PointStamped, geometry_msgs::PointStamped);
	move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped, geometry_msgs::PointStamped, bool);

	// Callbacks
	void odomCB(const nav_msgs::Odometry::ConstPtr&);




public:
	GpsWaypoints(ros::NodeHandle);
	~GpsWaypoints();
	int loadFile(void);
	void RotateinPlace(void);
	void Normal(void);

};