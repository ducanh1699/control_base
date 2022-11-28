/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <eigen3/Eigen/Dense>
using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped target_position;
void target_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& target_pos){
    target_position.pose.position.x = target_pos->pose.position.x;
    target_position.pose.position.y = target_pos->pose.position.y;
    target_position.pose.position.z = target_pos->pose.position.z;
}

// call back the current position
geometry_msgs::PoseStamped current_position;
void local_cb( const geometry_msgs::PoseStamped::ConstPtr & msg){
    current_position = *msg;
}

void inputPoints(int n, float fly_point[][3])
{
    for (size_t i = 0; i < n; i++)  
    {   
        cout << "Enter position x, y, z: ";
        for (size_t j = 0; j < 3; j++)
        {   
            cin >> fly_point[i][j];
        }
        cout << endl;
    }    
}

bool check_position(float error,geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target){
    Eigen::Vector3d stop;
    stop << target.pose.position.x - current.pose.position.x,
            target.pose.position.y - current.pose.position.y,
            target.pose.position.z - current.pose.position.z;
    double a = stop.norm();
    if (a <= error){
        cout << "Reach target!" << endl;
        cout << a << endl;
        return true;
    }
    else
        return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>("target_position", 10, target_pos_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber local_p = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    int n;
    cout << "Enter the number of points which you want to fly: ";
    cin >> n;
    while (n <=  0)
    {
        cout << "Please input again the number of points which you want to fly: ";
        cin >> n;
        cout << endl;
    }
    cout << endl;
    float fly_point [n][3];
    inputPoints(n, fly_point); 

    float error;
    // Error position
    cout << "Input error position: ";
    cin >> error;
    cout << endl;

    geometry_msgs::PoseStamped set_point;
    set_point.pose.position.x = fly_point[0][0];
    set_point.pose.position.y = fly_point[0][1];
    set_point.pose.position.z = fly_point[0][2];

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    std::cout << "[ INFO] ----- Waiting OFFBOARD switch \n";
    while (ros::ok() && !current_state.armed && (current_state.mode != "OFFBOARD"))
    {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] --------------- READY --------------- \n";
    // ros::Time last_request = ros::Time::now();
    bool check_target = false;
    for (int i = 0; i < n; i++)
    {
        set_point.pose.position.x = fly_point[i][0];
        set_point.pose.position.y = fly_point[i][1];
        set_point.pose.position.z = fly_point[i][2];
        cout << "Fly to the next point: ";
        cout << set_point.pose.position.x << " ";
        cout << set_point.pose.position.y << " ";
        cout << set_point.pose.position.z << " ";
        cout << endl;
        ros::Time last_time = ros::Time::now();
        while (!check_target && ros::ok())
        {   
            local_pos_pub.publish(set_point);
            ros::spinOnce();
            rate.sleep();
            check_target = check_position(error, current_position, set_point);
        }
        ros::Time t_check;
        t_check = ros::Time::now();
        while ((ros::Time::now()-t_check)<ros::Duration(3))
        {   
            local_pos_pub.publish(set_point);
            ros::spinOnce();
            rate.sleep();
        }
        check_target = false;
    }
   
    // ros::Time t_now = ros::Time::now();
    // while(ros::ok()){
    //     if( current_state.mode != "OFFBOARD" &&
    //         (ros::Time::now() - last_request > ros::Duration(5.0))){
    //         if( set_mode_client.call(offb_set_mode) &&
    //             offb_set_mode.response.mode_sent){
    //             ROS_INFO("Offboard enabled");
    //         }
    //         last_request = ros::Time::now();
    //     } else {
    //         if( !current_state.armed &&
    //             (ros::Time::now() - last_request > ros::Duration(5.0))){
    //             if( arming_client.call(arm_cmd) &&
    //                 arm_cmd.response.success){
    //                 ROS_INFO("Vehicle armed");
    //             }
    //             last_request = ros::Time::now();
    //         }
    //     }
        
    //     local_pos_pub.publish(pose);

    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // while (ros::Time::now()- t_now > ros::Duration(10.0))
    // {
    //     pose = target_position;
    //     ROS_INFO_STREAM("x " << pose.pose.position.x << "y" << pose.pose.position.y << "z" << pose.pose.position.z);
    //     ros::spinOnce();
    //     rate.sleep();
    // }        

    return 0;
}