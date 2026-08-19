#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ros/package.h>
#include <algorithm>
#include <cctype>

mavros_msgs::State current_state;
mavros_msgs::PositionTarget setpoint_raw;
int traj_index = 0;

struct Waypoint{
    double x;
    double y;
    double z;
    double yaw;
};
std::vector<Waypoint> waypoints;

tf::Quaternion quat;
double roll, pitch, yaw;
float init_position_x_takeoff = 0;
float init_position_y_takeoff = 0;
float init_position_z_takeoff = 0;
bool flag_init_position = false;
nav_msgs::Odometry local_pos;
ros::Time last_record; // initialized after ros::init() to avoid TimeNotInitializedException

void localPositionCallback(const nav_msgs::Odometry::ConstPtr& msg){
    local_pos = *msg;
    if(flag_init_position == false && (fabs(local_pos.pose.pose.position.z ) >= 0.05)){
        init_position_x_takeoff = local_pos.pose.pose.position.x;
        init_position_y_takeoff = local_pos.pose.pose.position.y;
        init_position_z_takeoff = local_pos.pose.pose.position.z;
        flag_init_position = true;
    }
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void stateCallback(const mavros_msgs::State::ConstPtr&msg){
    current_state = *msg;
}

void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr&msg){
    setpoint_raw.position.x = msg->point.x - init_position_x_takeoff;
    setpoint_raw.position.y = msg->point.y - init_position_y_takeoff;
    setpoint_raw.position.z = 2.0;

    ROS_INFO("New target: x=%.2f y=%.2f z=%.2f", 
        setpoint_raw.position.x, setpoint_raw.position.y, setpoint_raw.position.z);
}

std::string getFilePath(const std::string& filename){
    std::string package_path = ros::package::getPath("click_to_fly");
    if (package_path.empty()){
        ROS_ERROR("Cannot find package 'click_to_fly'");
        return "";
    }
    return package_path + "/config/" + filename;
}

bool initTraj(const std::string& filename){
    std::string file_path = getFilePath(filename);
    if(file_path.empty()){
        return false;
    }

    std::ofstream ofs(file_path, std::ios::trunc);

    if(!ofs.is_open()){
        ROS_ERROR("Failed to initiate trajectory file: %s", file_path.c_str());
        return false;
    }

    ofs << "# name x y z yaw" << std::endl;
    ofs.close();
    traj_index = 0;
    return true;
}

bool writeTraj(const std::string& filename){
    if(!flag_init_position){
        ROS_WARN_THROTTLE(5.0, "Skipping trajectory write: initial position not set");
        return false;
    }

    std::string file_path = getFilePath(filename);
    if(file_path.empty()){
        return false;
    }

    std::ofstream ofs(file_path, std::ios::app);

    if(!ofs.is_open()){
        ROS_ERROR("Failed to initiate trajectory file: %s", file_path.c_str());
        return false;
    }

    ofs << "p" << traj_index << " "
        << local_pos.pose.pose.position.x << " "
        << local_pos.pose.pose.position.y << " "
        << local_pos.pose.pose.position.z << " "
        << yaw << std::endl;

    ofs.close();
    traj_index ++;
    return true;
}

bool loadTraj(const std::string& filename){
    std::string file_path = getFilePath(filename);
    if(file_path.empty()){
        return false;
    }

    std::ifstream ifs(file_path);

    if(!ifs.is_open()){
        ROS_ERROR("Failed to initiate trajectory file: %s", file_path.c_str());
        return false;
    }

    waypoints.clear();
    std::string line;
    int line_num = 0;

    auto trim = [](std::string &s){
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch){ return !std::isspace(ch); }));
        s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end());
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
        if(iss >> name >> wp.x >> wp.y >> wp.z >> wp.yaw){
            waypoints.push_back(wp);
            ROS_INFO("[Waypoint Loaded] %s x=%.3f, y=%.3f, z=%.3f, k=%.3f",
                    name.c_str(), wp.x, wp.y, wp.z, wp.yaw);
        }
        else{
            ROS_ERROR("Invalid traj line %d: '%s'", line_num, line.c_str());
        }
    }
    ROS_INFO("Loaded %zu traj points", waypoints.size());
    ifs.close();
    return true;
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "raw_setpoint_click_control");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string mode;
    std::string traj_file_name;

    pnh.param<std::string>(
        "mode",
        mode,
        "WRITE"
    );

    pnh.param<std::string>(
        "traj_file_name",
        traj_file_name,
        "traj.yaml"
    );

    ROS_INFO("mode = %s, traj_file = %s",
        mode.c_str(),
        traj_file_name.c_str());

    double request_interval = 5.0;
    double init_wait_timeout = 10.0;
    pnh.param("request_interval", request_interval, request_interval);
    pnh.param("init_wait_timeout", init_wait_timeout, init_wait_timeout);

    ROS_INFO("request_interval=%.1f, init_wait_timeout=%.1f", request_interval, init_wait_timeout);

    // initialize last_record now that ROS time is available
    last_record = ros::Time::now();

    if(mode == "WRITE"){
        initTraj(traj_file_name);
        ROS_INFO("Current mode: %s", mode.c_str());
    }
    else if(mode == "READ"){
        loadTraj(traj_file_name);
        ROS_INFO("Current mode: %s", mode.c_str());
    }
    else{
        ROS_ERROR("Unknown mode: %s", mode.c_str());
        return -1;
    }

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("/mavros/state", 100, stateCallback);

    ros::Subscriber clicked_point_sub = nh.subscribe<geometry_msgs::PointStamped>
    ("/clicked_point", 100, clickedPointCallback);

    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>
    ("/mavros/local_position/odom", 100, localPositionCallback);

    ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
    ("/mavros/setpoint_raw/local", 100);

    ros::ServiceClient arming_cl = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient setmode_cl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    

    ros::Rate rate(20.0);
    ROS_INFO("Initializing...");
    while (ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_raw.type_mask = /*1 + 2 + 4 +*/ 8 + 16 + 32 + 64 + 128 + 256 /*+512 + 1024  */+ 2048;

    setpoint_raw.position.x = init_position_x_takeoff;
    setpoint_raw.position.y = init_position_y_takeoff;
    setpoint_raw.position.z = init_position_z_takeoff + 2.0;
    setpoint_raw.yaw = 0.0;

    ROS_INFO("Waiting up to %.1f s for initial local position...", init_wait_timeout);
    ros::Time init_wait_start = ros::Time::now();
    while (ros::ok() && !flag_init_position && ros::Time::now() - init_wait_start < ros::Duration(init_wait_timeout)){
        ros::spinOnce();
        rate.sleep();
    }
    if(!flag_init_position){
        ROS_WARN("Initial local position not received within %.1f s, continuing with current origin.", init_wait_timeout);
    }

    ROS_INFO("Sending initial setpoints...");

    for(int i=0; ros::ok() && i<100; i++){
        setpoint_raw.header.stamp = ros::Time::now();
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offboard_mode;
    offboard_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    ROS_INFO("First offboard/arm attempt after %.1f seconds", request_interval);



    while (ros::ok()){

        if (current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(request_interval)){
            if (setmode_cl.call(offboard_mode) && offboard_mode.response.mode_sent){
                ROS_INFO("OFFBOARD enabled");
            }
            else{
                ROS_INFO("OFFBOARD set mode failed");
            }
            
            last_request = ros::Time::now();
        }

        else if (!current_state.armed && ros::Time::now() - last_request > ros::Duration(request_interval)){
            if (arming_cl.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            else{
                ROS_INFO("Vehicle arm failed");
            }
            last_request = ros::Time::now();
        }
    


        setpoint_raw.header.stamp =
            ros::Time::now();
        
        mavros_setpoint_pos_pub.publish(setpoint_raw);

        if(ros::Time::now() - last_record > ros::Duration(1.0) && mode == "WRITE"){
            writeTraj(traj_file_name);
            last_record = ros::Time::now();
            
        }

        if (fabs(local_pos.pose.pose.position.x - setpoint_raw.position.x) < 0.05
            && fabs(local_pos.pose.pose.position.y - setpoint_raw.position.y) < 0.08){
            ROS_WARN_THROTTLE(
                1.0, 
                "Target position (x=%.2f y=%.2f z=%.2f) reached, waiting for next target...",
                local_pos.pose.pose.position.x,
                local_pos.pose.pose.position.y,
                local_pos.pose.pose.position.z
            );
        }
        else{
            ROS_INFO_THROTTLE(
                1.0,
                "Current: x=%.2f y=%.2f z=%.2f | Target: x=%.2f y=%.2f z=%.2f",
                local_pos.pose.pose.position.x,
                local_pos.pose.pose.position.y,
                local_pos.pose.pose.position.z,
                setpoint_raw.position.x,
                setpoint_raw.position.y,
                setpoint_raw.position.z
            );
        }
        


        ros::spinOnce();

        rate.sleep();
    }
    return 0;

}