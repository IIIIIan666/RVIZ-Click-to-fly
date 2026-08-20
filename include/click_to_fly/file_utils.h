// include/click_to_fly/file_utils.h
#pragma once
#include <string>
#include <vector>

struct Waypoint{
    double x;
    double y;
    double z;
    //double yaw;
};

std::string getFilePath(const std::string& filename);
bool initTraj(const std::string& filename);
bool appendWaypoint(const std::string& filename,
                const Waypoint& wp,
                int index);
bool loadTraj(const std::string& filename,
              std::vector<Waypoint>& waypoints);
bool saveTraj(const std::string& filename,
              const std::vector<Waypoint>& waypoints);
