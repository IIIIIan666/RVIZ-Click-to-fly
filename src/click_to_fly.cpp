// src/click_to_fly.cpp
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string>
#include <vector>
#include <click_to_fly/file_utils.h>

mavros_msgs::State current_state;
mavros_msgs::PositionTarget setpoint_raw;
int traj_index = 0;

std::vector<Waypoint> waypoints;

struct Missionpoint{
    double x;
    double y;
    double z;
};
std::vector<Missionpoint> missionpoints;

float init_position_x_takeoff = 0;
float init_position_y_takeoff = 0;
float init_position_z_takeoff = 0;
double takeoff_altitude = 10.0;
bool flag_init_position = false;
nav_msgs::Odometry local_pos;
ros::Time last_record; // initialized after ros::init() to avoid TimeNotInitializedException
size_t current_mission_index = 0;
double mission_tolerance = 0.5;


void localPositionCallback(const nav_msgs::Odometry::ConstPtr& msg){
    local_pos = *msg;
    if(flag_init_position == false){
        init_position_x_takeoff = local_pos.pose.pose.position.x;
        init_position_y_takeoff = local_pos.pose.pose.position.y;
        init_position_z_takeoff = local_pos.pose.pose.position.z;
        flag_init_position = true;
    }
}

void stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
    if(!flag_init_position){
        ROS_WARN("Ignoring clicked point: initial local position is not available");
        return;
    }
    if(!msg->header.frame_id.empty() && !local_pos.header.frame_id.empty()
        && msg->header.frame_id != local_pos.header.frame_id){
        ROS_ERROR("Ignoring clicked point: frame '%s' does not match local position frame '%s'",
            msg->header.frame_id.c_str(), local_pos.header.frame_id.c_str());
        return;
    }
    Missionpoint mp;
    mp.x = msg->point.x;
    mp.y = msg->point.y;
    mp.z = init_position_z_takeoff + takeoff_altitude;
    missionpoints.push_back(mp);
    ROS_INFO("New mission point added: x=%.2f y=%.2f z=%.2f", mp.x, mp.y, mp.z);
}

void setCurrentMissionpoint(){
    if(current_mission_index >= missionpoints.size()){
        ROS_WARN_THROTTLE(5.0, "No more mission points to set.");
        return;
    }
    const Missionpoint& mp = missionpoints[current_mission_index];
    setpoint_raw.position.x = mp.x;
    setpoint_raw.position.y = mp.y;
    setpoint_raw.position.z = mp.z;
    ROS_INFO_THROTTLE(1.0, "Setting mission point %zu: x=%.2f y=%.2f z=%.2f", current_mission_index, mp.x, mp.y, mp.z);
}

bool missionpointReached(const Missionpoint& mp){
    double distance = std::sqrt(
        std::pow(local_pos.pose.pose.position.x - mp.x, 2) +
        std::pow(local_pos.pose.pose.position.y - mp.y, 2) +
        std::pow(local_pos.pose.pose.position.z - mp.z, 2)
    );
    return distance <= mission_tolerance;
}







int main(int argc, char** argv){
    
    ros::init(argc, argv, "raw_setpoint_click_control");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string mode = "WRITE";
    std::string write_traj_file_name = "traj.yaml";
    std::string read_traj_file_name = "traj.yaml";
    double write_interval = 1;





    pnh.param<std::string>(
        "mode",
        mode,
        mode
    );

    pnh.param<std::string>(
        "write_traj_file_name",
        write_traj_file_name,
        write_traj_file_name
    );

    pnh.param<std::string>(
        "read_traj_file_name",
        read_traj_file_name,
        read_traj_file_name
    );

    pnh.param<double>(
        "takeoff_altitude",
        takeoff_altitude,
        takeoff_altitude
    );

    pnh.param<double>(
        "write_interval",
        write_interval,
        write_interval
    );

    pnh.param<double>(
        "mission_tolerance",
        mission_tolerance,
        mission_tolerance
    );



    ROS_INFO("mode = %s, write_traj_file = %s, read_traj_file = %s",
        mode.c_str(),
        write_traj_file_name.c_str(),
        read_traj_file_name.c_str());

    double request_interval = 5.0;
    double init_wait_timeout = 10.0;
    pnh.param("request_interval", request_interval, request_interval);
    pnh.param("init_wait_timeout", init_wait_timeout, init_wait_timeout);

    ROS_INFO("request_interval=%.1f, init_wait_timeout=%.1f", request_interval, init_wait_timeout);

    // initialize last_record now that ROS time is available
    last_record = ros::Time::now();

    if(write_interval <= 0 || mission_tolerance <= 0){
        ROS_FATAL("write_interval and mission_tolerance must be greater than zero");
        return -1;
    }

    if(mode == "WRITE"){
        if(!initTraj(write_traj_file_name)){
            ROS_FATAL("Failed to initialize trajectory file: %s", write_traj_file_name.c_str());
            return -1;
        }
        traj_index = 0;
        ROS_INFO("Current mode: %s", mode.c_str());
    }
    else if(mode == "READ"){
        if(!loadTraj(read_traj_file_name, waypoints)){
            ROS_FATAL("Failed to load trajectory file: %s", read_traj_file_name.c_str());
            return -1;
        }
        for(const Waypoint& wp : waypoints){
            Missionpoint mp;
            mp.x = wp.x;
            mp.y = wp.y;
            mp.z = wp.z;
            missionpoints.push_back(mp);
        }
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

    ROS_INFO("Waiting up to %.1f s for initial local position...", init_wait_timeout);
    ros::Time init_wait_start = ros::Time::now();
    while (ros::ok() && !flag_init_position && ros::Time::now() - init_wait_start < ros::Duration(init_wait_timeout)){
        ros::spinOnce();
        rate.sleep();
    }
    if(!flag_init_position){
        ROS_FATAL("Initial local position not received within %.1f s", init_wait_timeout);
        return -1;
    }

    setpoint_raw.position.x = init_position_x_takeoff;
    setpoint_raw.position.y = init_position_y_takeoff;
    setpoint_raw.position.z = init_position_z_takeoff + takeoff_altitude;
    setpoint_raw.yaw = 0.0;

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
    

        if(current_mission_index < missionpoints.size()){
            setCurrentMissionpoint();
            const Missionpoint& current_mp = missionpoints[current_mission_index];
            
            if(missionpointReached(current_mp)){
                ROS_INFO("Mission point %zu reached: x=%.2f y=%.2f z=%.2f", current_mission_index, current_mp.x, current_mp.y, current_mp.z);
                current_mission_index++;
                if(current_mission_index < missionpoints.size()){
                    ROS_INFO("Next mission point: x=%.2f y=%.2f z=%.2f", 
                        missionpoints[current_mission_index].x, missionpoints[current_mission_index].y, missionpoints[current_mission_index].z);
                }
                else{
                    ROS_INFO("All mission points completed.");
                }         
            }
        }

        setpoint_raw.header.stamp = ros::Time::now();
        mavros_setpoint_pos_pub.publish(setpoint_raw);

        if(ros::Time::now() - last_record > ros::Duration(write_interval) && mode == "WRITE"){
            if(flag_init_position){
                Waypoint wp;
                wp.x = local_pos.pose.pose.position.x;
                wp.y = local_pos.pose.pose.position.y;
                wp.z = local_pos.pose.pose.position.z;
                if(appendWaypoint(write_traj_file_name, wp, traj_index)){
                    ROS_INFO("Waypoint %d recorded: x=%.2f y=%.2f z=%.2f", traj_index, wp.x, wp.y, wp.z);
                    traj_index++;
                }
                else{
                    ROS_ERROR("Failed to record waypoint %d", traj_index);
                }
            }
            last_record = ros::Time::now();
            
        }

        double target_distance = std::sqrt(
            std::pow(local_pos.pose.pose.position.x - setpoint_raw.position.x, 2) +
            std::pow(local_pos.pose.pose.position.y - setpoint_raw.position.y, 2) +
            std::pow(local_pos.pose.pose.position.z - setpoint_raw.position.z, 2)
        );
        if(target_distance <= mission_tolerance){
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
