#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

#include "turtlesim/Spawn.h"
#include "turtlesim/Pose.h"
#include <turtlesim/TeleportAbsolute.h>

#include <string>
#include "unistd.h"
#include<iostream>
#include <cmath>



ros::Publisher turtle1_commander;
ros::Publisher turtle2_commander;

ros::ServiceClient turtle1_pose_commander;
ros::ServiceClient turtle2_pose_commander;
ros::Publisher turtles_dist_pub;

turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;
geometry_msgs::Twist zero_twist;
turtlesim::TeleportAbsolute pose_command;
const double thresh = 1.0;

double get_turtles_distance()
{
    return pow((pow((turtle1_pose.x-turtle2_pose.x),2)+pow((turtle1_pose.y-turtle2_pose.y),2)),0.5);
}

void monitor_turtle_distances()
{   
    float x;
    float y;
    const float min_thresh = 1.0;
    const float max_thresh = 10.0;
    // checking boundries
    if(turtle1_pose.x>max_thresh || turtle1_pose.x<min_thresh || turtle1_pose.y >max_thresh || turtle1_pose.y<min_thresh)
    {
        turtle1_commander.publish(zero_twist);
        x = turtle1_pose.x;
        y = turtle1_pose.y;
        pose_command.request.x = std::max(std::min(max_thresh,x),min_thresh);
        pose_command.request.y = std::max(std::min(max_thresh,y),min_thresh);
        turtle1_pose_commander.call(pose_command);
    }
        
    // checking boundries
    if(turtle2_pose.x>max_thresh || turtle2_pose.x<min_thresh || turtle2_pose.y >max_thresh || turtle2_pose.y<min_thresh)
    {
        turtle2_commander.publish(zero_twist);
        float x = turtle2_pose.x;
        float y = turtle2_pose.y;
        pose_command.request.x = std::max(std::min(max_thresh,x),min_thresh);
        pose_command.request.y = std::max(std::min(max_thresh,y),min_thresh);
        turtle2_pose_commander.call(pose_command);
    }
        
    // calculating distance
    double dist = get_turtles_distance();
    // stop turtles if dist is less than thresh
    if(dist<thresh)
    {
        turtle1_commander.publish(zero_twist);
        turtle2_commander.publish(zero_twist);
        pose_command.request.x = turtle2_pose.x;
        if(turtle2_pose.y>turtle1_pose.y)
            pose_command.request.y = turtle2_pose.y+thresh;
        else
            pose_command.request.y = turtle2_pose.y-thresh;
        turtle2_pose_commander.call(pose_command);
    }
    // publish the distance
    std_msgs::Float32 dist_msg;
    dist_msg.data = dist;
    turtles_dist_pub.publish(dist_msg);
}

void monitor_turtle_pose(const turtlesim::Pose::ConstPtr& pose ,turtlesim::Pose &turtle_pose,ros::Publisher &turtle_pub)
{
    // updating the pose
    turtle_pose.x = pose->x;
    turtle_pose.y = pose->y;
    
    
    
}


void monitor_turtle1_pose(const turtlesim::Pose::ConstPtr& pose)
{
    monitor_turtle_pose(pose,turtle1_pose,turtle1_commander);
}
void monitor_turtle2_pose(const turtlesim::Pose::ConstPtr& pose)
{
    monitor_turtle_pose(pose,turtle2_pose,turtle2_commander);
}



int main(int argc, char **argv)
{
    
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS //system  
	ros::init(argc, argv, "Distance");  
	ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    turtle1_pose.x = 5;
    turtle2_pose.x = 2;
    turtle1_pose.y = 5;
    turtle2_pose.y = 2;

    zero_twist.linear.x = 0;
    zero_twist.linear.y = 0;
    zero_twist.angular.z = 0;
    // initialize pubs and subs
    turtle1_commander = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
    turtle2_commander = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",1);
    turtle1_pose_commander = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    turtle2_pose_commander = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle2/teleport_absolute");

    turtles_dist_pub = nh.advertise<std_msgs::Float32>("/distance",1);

    // prepare callback functions for the subscribers
    

    ros::Subscriber sub1 = nh.subscribe("/turtle1/pose",1,monitor_turtle1_pose);
    ros::Subscriber sub2 = nh.subscribe("/turtle2/pose",1,monitor_turtle2_pose);
    
    while (ros::ok()) {
        monitor_turtle_distances();

        // Handle any callbacks (if needed)
        ros::spinOnce();

        // Sleep for the remainder of the loop time to maintain the fixed interval
        loop_rate.sleep();
    }

    return 0;
}