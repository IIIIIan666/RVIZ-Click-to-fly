#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

mavros_msgs::State current_state;
mavros_msgs::PositionTarget setpoint_raw;

void stateCallback(const mavros_msgs::State::ConstPtr&msg){
    current_state = *msg;
}

void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr&msg){
    setpoint_raw.position.x = msg->point.x;
    setpoint_raw.position.y = msg->point.y;
    setpoint_raw.position.z = 2.0;

    ROS_INFO("New target: x=%.2f y=%.2f z=%.2f", 
        setpoint_raw.position.x, setpoint_raw.position.y, setpoint_raw.position.z);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "raw_setpoint_click_control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("/mavros/state", 100, stateCallback);

    ros::Subscriber clicked_point_sub = nh.subscribe<geometry_msgs::PointStamped>
    ("/clicked_point", 100, clickedPointCallback);

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
    setpoint_raw.type_mask = /*1 + 2 + 4*/ + 8 + 16 + 32 + 64 + 128 + 256 /*+512 + 1024  */+ 2048;

    setpoint_raw.position.x = 0.0;
    setpoint_raw.position.y = 0.0;
    setpoint_raw.position.z = 2.0;
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



    while (ros::ok()){

        if (current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0)){
            if (setmode_cl.call(offboard_mode) && offboard_mode.response.mode_sent){
                ROS_INFO("OFFBOARD enabled");
            }
            else{
                ROS_INFO("OFFBOARD set mode failed");
            }
            
            last_request = ros::Time::now();
        }

        else if (!current_state.armed && ros::Time::now() - last_request > ros::Duration(5.0)){
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


        ROS_INFO_THROTTLE(
            1.0,
            "Target: x=%.2f y=%.2f z=%.2f",
            setpoint_raw.position.x,
            setpoint_raw.position.y,
            setpoint_raw.position.z
        );


        ros::spinOnce();

        rate.sleep();
    }
    return 0;

}       
