#include <ros/ros.h>
#include "gpswaypoints.h"




int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_waypoint"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    int load_file_status;
    GpsWaypoints Gps(n);
    load_file_status = Gps.loadFile();

    if(load_file_status != -1)
    {
        Gps.RotateinPlace();
    }
}
