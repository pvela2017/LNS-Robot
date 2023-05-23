#include "gpswaypoints.h"


GpsWaypoints::GpsWaypoints(ros::NodeHandle n) : 
    ac_("/move_base", true),
    spinner_(2)
{
	n_ = n;
    spinner_.start();
	// Initiate publisher to send end of node message
    pubWaypointNodeEnded_ = n_.advertise<std_msgs::Bool>("/waypoint_following_status", 100);
    // To get current position of the robot with respect of the map
    odomfiltered_ = n.subscribe("/odometry/filtered", 1, &GpsWaypoints::odomCB, this);

    // Timer
	int wait_count = 0;

    // Wait for the action server to come up
    while(!ac_.waitForServer(ros::Duration(5.0)))
    {
        wait_count++;
        if(wait_count > 3)
        {
            ROS_ERROR("move_base action server did not come up, killing gps_waypoint node...");
            // Notify joy_launch_control that waypoint following is complete
            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded_.publish(node_ended);
            ros::shutdown();
        }
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Get Path to coordinates file
    std::string path_local;
    ros::param::get("coordinates_file", path_local);
    path_abs_ = ros::package::getPath("lns_navigation") + path_local;
}


GpsWaypoints::~GpsWaypoints()
{
    spinner_.stop();
}

void GpsWaypoints::countWaypointsInFile()
{
	/*
		Gets the total number of waypoints to
		be send to the planner.

		latitude longitude heading
	*/

	int count = 0;

	// Open the file
    std::ifstream fileCount(path_abs_.c_str());
    // Count lines, each line is 1 waypoint
    if(fileCount.is_open())
    {
    	//TODO: este metodo vale callampa, cambiarlo
        double lati = 0;
        while(!fileCount.eof())
        {
            fileCount >> lati;
            ++count;
        }
        count = count - 1;
        numWaypoints_ = count / 3;
        ROS_INFO("%d GPS waypoints were read", numWaypoints_);
        fileCount.close();
    }
    else
    {
        std::cout << "Unable to open waypoint file" << std::endl;
        ROS_ERROR("Unable to open waypoint file");
    }
}


void GpsWaypoints::getWaypoints()
{
	/*
		Fill the waypoint vector with the information of the file
	*/

	// Initialize variables
    double lati = 0, longi = 0, heading = 0;

    // Stream the variables
    std::ifstream fileRead(path_abs_.c_str());
    for(int i = 0; i < (numWaypoints_ + 1); i++)
    {
        fileRead >> lati;
        fileRead >> longi;
        fileRead >> heading;
        waypointVect_.push_back(std::make_tuple(lati, longi, heading));
    }
    fileRead.close();

    // Outputting vector
    ROS_INFO("The following GPS Waypoints have been set:");
    for(std::vector <std::tuple<double, double, double >> ::iterator iterDisp = waypointVect_.begin(); iterDisp != waypointVect_.end(); iterDisp++)
    {
        ROS_INFO("%.9g %.9g %.3f",  std::get<0>(*iterDisp), std::get<1>(*iterDisp), std::get<2>(*iterDisp));
    }
}



geometry_msgs::PointStamped GpsWaypoints::latLongtoUTM(double lati_input, double longi_input)
{
	/*
		Transform GPS coordinates into UTM coordinates
	*/
    double utm_x = 0, utm_y = 0;
    // Should be 52S for korea
    // https://www.dmap.co.uk/utmworld.htm
    std::string utm_zone;
    geometry_msgs::PointStamped UTM_point_output;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = utm_x;
    UTM_point_output.point.y = utm_y;
    UTM_point_output.point.z = 0;

    return UTM_point_output;
}


geometry_msgs::PointStamped GpsWaypoints::UTMtoMapPoint(geometry_msgs::PointStamped UTM_input, geometry_msgs::PointStamped* UTM_point)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_point->header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
            listener.transformPoint("odom", UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map_point_output;
}


geometry_msgs::PointStamped GpsWaypoints::MaptoOdom(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped* odom_pose)
{
    geometry_msgs::PointStamped odom_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            map_point.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "map", time_now, ros::Duration(3.0));
            listener.transformPoint("odom", map_point, odom_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return odom_point_output;
}



move_base_msgs::MoveBaseGoal GpsWaypoints::translation(geometry_msgs::PointStamped map_point, double yaw)
{
    move_base_msgs::MoveBaseGoal goal;

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
    goal.target_pose.pose.position.y = map_point.point.y; //specify y goal

    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    tf::Matrix3x3 rot_euler;
    tf::Quaternion rot_quat;
    float  pitch_curr = 0, roll_curr = 0;
    
    // Specify quaternions
    rot_euler.setEulerYPR(yaw, pitch_curr, roll_curr);
    rot_euler.getRotation(rot_quat);

    goal.target_pose.pose.orientation.x = rot_quat.getX();
    goal.target_pose.pose.orientation.y = rot_quat.getY();
    goal.target_pose.pose.orientation.z = rot_quat.getZ();
    goal.target_pose.pose.orientation.w = rot_quat.getW();


    return goal;
}



move_base_msgs::MoveBaseGoal GpsWaypoints::rotation(geometry_msgs::PointStamped odom_point, geometry_msgs::PointStamped map_next)
{
    move_base_msgs::MoveBaseGoal goal;

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = odom_point.point.x; //specify x goal
    goal.target_pose.pose.position.y = odom_point.point.y; //specify y goal

    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    tf::Matrix3x3 rot_euler;
    tf::Quaternion rot_quat;
    float pitch_curr = 0, roll_curr = 0;
   
    // Calculate quaternion
    float x_curr = odom_point.point.x, y_curr = odom_point.point.y; // set current coords.
    float x_next = map_next.point.x, y_next = map_next.point.y; // set coords. of next waypoint
    float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
    float yaw_curr = atan2(delta_y, delta_x);

    // Specify quaternions
    rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
    rot_euler.getRotation(rot_quat);

    goal.target_pose.pose.orientation.x = rot_quat.getX();
    goal.target_pose.pose.orientation.y = rot_quat.getY();
    goal.target_pose.pose.orientation.z = rot_quat.getZ();
    goal.target_pose.pose.orientation.w = rot_quat.getW();

    return goal;
}


int GpsWaypoints::loadFile(void)
{
    /**/

    countWaypointsInFile();
    if (numWaypoints_ == -1)
    {
        ROS_ERROR("Couldn't get the waypoints");
        ros::shutdown();
        return -1;
    }
    getWaypoints();
    return 0;
}


void GpsWaypoints::RotateinPlace(void)
{
	/*
	*/

    // Auxiliary variables definition
	geometry_msgs::PointStamped UTM_goal, map_goal, UTM_next_goal, map_next_goal, odom_pose;
	double latiGoal, longiGoal, headingGoal, latiNextGoal, longiNextGoal, headingNextGoal;
    std_msgs::Bool node_ended;

	// Init. iterator
	std::vector<std::tuple<double, double, double> > ::iterator iter; 

	// Iterate through vector of waypoints for setting goals
    for(iter = waypointVect_.begin(); iter < waypointVect_.end(); iter++)
    {
        //Setting goal:
        latiGoal = std::get<0>(*iter);
        longiGoal = std::get<1>(*iter);
        headingGoal = std::get<2>(*iter);
        bool final_point = false;

        // Set next goal point if not at last waypoint
        if(iter < (waypointVect_.end() - 1))
        {
            iter++;
            latiNextGoal = std::get<0>(*iter);
            longiNextGoal = std::get<1>(*iter);
            headingNextGoal = std::get<2>(*iter);
            iter--;
        }
        else // Set to current
        {
            latiNextGoal = std::get<0>(*iter);
            longiNextGoal = std::get<1>(*iter);
            headingNextGoal = std::get<2>(*iter);
            final_point = true;
        }

        ROS_INFO("Received Latitude goal:%.8f", latiGoal);
        ROS_INFO("Received longitude goal:%.8f", longiGoal);
        ROS_INFO("Received orientation goal:%.3f", headingGoal);

        // Convert lat/long to utm:
        UTM_goal = latLongtoUTM(latiGoal, longiGoal);
        UTM_next_goal = latLongtoUTM(latiNextGoal, longiNextGoal);

        // Transform UTM to map point in odom frame
        map_goal = UTMtoMapPoint(UTM_goal, &map_goal);
        map_next_goal = UTMtoMapPoint(UTM_next_goal, &map_next_goal);

        // Build goal to send to move_base
        move_base_msgs::MoveBaseGoal goal = translation(map_goal, headingGoal); 

        // Send Goal
        ROS_INFO("Sending translation");
        ac_.sendGoal(goal); //push goal to move_base node

        //Wait for result
        ac_.waitForResult(); //waiting to see if move_base was able to reach goal

        if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Robot has reached its goal!");
            //switch to next waypoint and repeat
        }
        else
        {
            ROS_ERROR("Robot was unable to reach its goal. GPS Waypoint unreachable.");
            ROS_INFO("Exiting node...");
            // Notify joy_launch_control that waypoint following is complete
            node_ended.data = true;
            pubWaypointNodeEnded_.publish(node_ended);
            ros::shutdown();
        }


        // Rotate in place if it is not last point
        if (!final_point)
        {
            ros::spinOnce();
            odom_pose = MaptoOdom(map_pose_, &odom_pose);
            move_base_msgs::MoveBaseGoal rotate = rotation(odom_pose, map_next_goal);
            // Send Goal
            ROS_INFO("Sending rotation");
            ac_.sendGoal(rotate); //push goal to move_base node

            //Wait for result
            ac_.waitForResult(); //waiting to see if move_base was able to reach goal

            if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Robot has rotated!");
                //switch to next waypoint and repeat
            }
            else
            {
                ROS_ERROR("Robot was unable to rotate.");
                ROS_INFO("Exiting node...");
                // Notify joy_launch_control that waypoint following is complete
                node_ended.data = true;
                pubWaypointNodeEnded_.publish(node_ended);
                ros::shutdown();
            }
        }
    } // End for loop iterating through waypoint vector


    ROS_INFO("Robot has reached all of its goals!!!\n");
    ROS_INFO("Ending node...");

    // Notify joy_launch_control that waypoint following is complete
    node_ended.data = true;
    pubWaypointNodeEnded_.publish(node_ended);
}


void GpsWaypoints::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    map_pose_.header.frame_id = msg->header.frame_id;
    map_pose_.header.stamp = msg->header.stamp;
    map_pose_.point.x = msg->pose.pose.position.x;
    map_pose_.point.y = msg->pose.pose.position.y;
    map_pose_.point.z = msg->pose.pose.position.z;
}