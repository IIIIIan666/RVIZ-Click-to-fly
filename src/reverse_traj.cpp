// src/reverse_traj.cpp
#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <click_to_fly/file_utils.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "reverse_traj");
    ros::NodeHandle pnh("~");

    std::string input_file = "write_traj.yaml";
    std::string output_file = "read_traj.yaml";

    pnh.param<std::string>(
        "input_file",
        input_file,
        input_file
    );
    pnh.param<std::string>(
        "output_file",
        output_file,
        output_file
    );

    std::vector<Waypoint> waypoints;

    if(!loadTraj(input_file, waypoints)){
        ROS_FATAL("Failed to load trajectory");
        return -1;
    }

    std::reverse(
        waypoints.begin(),
        waypoints.end()
    );

    if(!saveTraj(output_file, waypoints)){
        ROS_ERROR("Failed to save reversed trajectory");
        return -1;
    }
    ROS_INFO("Trajectory reversed: %s -> %s",
            input_file.c_str(),
            output_file.c_str());

    return 0;
}
