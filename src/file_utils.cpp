#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <click_to_fly/file_utils.h>

std::string getFilePath(const std::string& filename){
    if(filename.empty()){
        ROS_ERROR("Trajectory filename is empty");
        return "";
    }
    std::string package_path = ros::package::getPath("click_to_fly");
    if (package_path.empty()){
        ROS_ERROR("Cannot find package 'click_to_fly'");
        return "";
    }
    return package_path + "/config/" + filename;
    // Return full filepath
}

bool initTraj(const std::string& filename){
    std::string file_path = getFilePath(filename);
    if(file_path.empty()){
        ROS_FATAL("Cannot find file: %s", filename.c_str());
        return false;
    }

    std::ofstream ofs(file_path, std::ios::trunc); //output file stream, truncate original content

    if(!ofs.is_open()){
        ROS_ERROR("Failed to initiate trajectory file: %s", file_path.c_str());
        return false;
    }

    // ofs << "# name x y z yaw" << std::endl;
    ofs << "# name x y z" << std::endl;
    ofs.close();
    if(!ofs){
        ROS_ERROR("Failed to write trajectory file: %s", file_path.c_str());
        return false;
    }
    return true;
}

bool appendWaypoint(
    const std::string& filename,
    const Waypoint& wp,
    int index){
    std::string file_path = getFilePath(filename);

    if(file_path.empty()){
        return false;
    }

    std::ofstream ofs(file_path, std::ios::app);

    if(!ofs.is_open()){
        ROS_ERROR(
            "Failed to open trajectory file: %s",
            file_path.c_str()
        );
        return false;
    }

    ofs << "p" << index << " "
        << wp.x << " "
        << wp.y << " "
        << wp.z
        << std::endl;

    if(!ofs){
        ROS_ERROR(
            "Failed to write trajectory file: %s",
            file_path.c_str()
        );
        return false;
    }
    return true;
}

bool loadTraj(
    const std::string& filename,
    std::vector<Waypoint>& waypoints
){
    std::string file_path = getFilePath(filename);
    if(file_path.empty()){
        return false;
    }
    std::ifstream ifs(file_path);
    if(!ifs.is_open()){
        ROS_ERROR(
            "Failed to open trajectory file: %s",
            file_path.c_str()
        );
        return false;
    }
    waypoints.clear();
    std::string line;
    int line_num = 0;

    auto trim = [](std::string& s){
        s.erase(
            s.begin(),
            std::find_if(
                s.begin(),
                s.end(),
                [](unsigned char ch){
                    return !std::isspace(ch);
                }
            )
        );

        s.erase(
            std::find_if(
                s.rbegin(),
                s.rend(),
                [](unsigned char ch){
                    return !std::isspace(ch);
                }
            ).base(),
            s.end()
        );
    };

    while(std::getline(ifs, line)){
        ++line_num;
        trim(line);
        if(line.empty()){
            continue;
        }
        if(line[0] == '#'){
            continue;
        }
        std::stringstream iss(line);
        std::string name;
        Waypoint wp;
        if(iss >> name >> wp.x >> wp.y >> wp.z){
            waypoints.push_back(wp);
            ROS_INFO(
                "[Waypoint Loaded] %s x=%.3f y=%.3f z=%.3f",
                name.c_str(),
                wp.x,
                wp.y,
                wp.z
            );
        }else{
            ROS_ERROR(
                "Invalid traj line %d: '%s'",
                line_num,
                line.c_str()
            );
        }
    }

    ROS_INFO(
        "Loaded %zu traj points",
        waypoints.size()
    );

    return !waypoints.empty();
}

bool saveTraj(const std::string& filename,
            const std::vector<Waypoint>& waypoints){
    std::string file_path = getFilePath(filename);

    if(file_path.empty()){
        return false;
    }

    std::ofstream ofs(file_path, std::ios::trunc); // open file, truncate original content

    if(!ofs.is_open()){
        ROS_FATAL("Failed to initiate trajectory file: %s", file_path.c_str());
        return false;
    }

    for(size_t i=0; i<waypoints.size(); ++i){
        ofs << "p" << i << " "
            << waypoints[i].x << " "
            << waypoints[i].y << " "
            << waypoints[i].z << " "
            // << yaw << std::endl; // yaw recording commented out
            << std::endl;

    }
    
    ofs.close();
    if(!ofs){
        ROS_ERROR("Failed to write trajectory file: %s", file_path.c_str());
        return false;
    }
    return true;
}
