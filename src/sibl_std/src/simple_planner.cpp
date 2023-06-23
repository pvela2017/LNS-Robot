#include <pluginlib/class_list_macros.h>
#include "simple_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(simple_planner::SimplePlanner, nav_core::BaseGlobalPlanner)

using namespace std;

 //Default Constructor
namespace simple_planner 
{


SimplePlanner::SimplePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}


void SimplePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

if(!initialized_)
{
  waypoints_per_meter_ = 100;
  initialized_ = true;
}

else
{
  ROS_WARN("This planner has already been initialized... doing nothing");
}

}

bool SimplePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan)
{
  plan.clear();
  // Start position
  double x1 = start.pose.position.x;
  double y1 = start.pose.position.y;

  // Goal position
  double x2 = goal.pose.position.x;
  double y2 = goal.pose.position.y;


  double dist =  hypot(x1-x2, y1-y2);
  double direction=atan2(y2-y1,x2-x1);



return true;
}

/*
// interpolate path
void SimplePlanner::interpolatePath(nav_msgs::Path& path)
{
  std::vector<geometry_msgs::PoseStamped> temp_path;
  for (int i = 0; i < static_cast<int>(path.poses.size()-2); i++)
  {
    // calculate distance between two consecutive waypoints
    double x1 = path.poses[i].pose.position.x;
    double y1 = path.poses[i].pose.position.y;
    double x2 = path.poses[i+1].pose.position.x;
    double y2 = path.poses[i+1].pose.position.y;
    double dist =  hypot(x1-x2, y1-y2);
    int num_wpts = dist * waypoints_per_meter;
    temp_path.push_back(path.poses[i]);
    geometry_msgs::PoseStamped p = path.poses[i];
    double A=0.0;
    double w=1.0/2.0;
    double direction=atan2(y2-y1,x2-x1);
    for (int j = 0; j < num_wpts; j++)
    {
      double xxx = w * 3.1415926 * static_cast<double>(j+1) / num_wpts * dist;
      double lateral = A*sin(xxx);
      //printf("%f\n",lateral);
      p.pose.position.x = x1 + static_cast<double>(j+1) / num_wpts * (x2 - x1) - lateral*sin(direction);
      p.pose.position.y = y1 + static_cast<double>(j+1) / num_wpts * (y2 - y1) + lateral*cos(direction);
      double dir = atan(3.1415926*w*A*cos(xxx)) + direction;
      p.pose.orientation = tf::createQuaternionMsgFromYaw(dir);
      temp_path.push_back(p);
    }
    temp_path.push_back(path.poses[i+1]);
    goal_index.data[i+1] = temp_path.size()-1;
    ROS_INFO("path number: %ld",temp_path.size()-1);
  }
  // update sequence of poses
  for (size_t i = 0; i < temp_path.size(); i++)
    temp_path[i].header.seq = static_cast<int>(i);
  path.poses = temp_path;
}*/

};