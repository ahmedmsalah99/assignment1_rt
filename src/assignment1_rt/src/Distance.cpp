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
    turtlesim::TeleportAbsolute pose1_command;
    turtlesim::TeleportAbsolute pose2_command;
    
        
    // // calculating distance
    double dist = get_turtles_distance();
    // // stop turtles if dist is less than thresh
    bool teleport1 = false;
    bool teleport2 = false;
    if(dist<thresh)
    {
        teleport1 = true;
        teleport2 = true;
        turtle1_commander.publish(zero_twist);
        turtle2_commander.publish(zero_twist);
        // teleport both in oppoiste directions



        float y_diff = abs(turtle2_pose.y - turtle1_pose.y);
        float x_diff = abs(turtle2_pose.x - turtle1_pose.x);
        float signy1 = 0;
        float signy2 = 0;
        float signx1 = 0;
        float signx2 = 0;
       

        if(y_diff > x_diff)
        {
            // default turtle1 up and 2 down
            signy1 = 1;
            signy2 = -1;
            if(turtle2_pose.y>turtle1_pose.y)
            {
                signy1 = -1;
                signy2 = 1;
            }
        }else       
        {
            // default turtle1 right and 2 left
            signx1 = 1;
            signx2 = -1;
            if(turtle2_pose.x>turtle1_pose.x)
            {
                signx1 = -1;
                signx2 = 1;
            }
        } 
        pose1_command.request.x = turtle1_pose.x + signx1*thresh;
        pose1_command.request.y = turtle1_pose.y + signy1*thresh;

        pose2_command.request.x = turtle2_pose.x + signx2*thresh;
        pose2_command.request.y = turtle2_pose.y + signy2*thresh;

    }


    // checking boundries
    if(turtle1_pose.x>max_thresh || turtle1_pose.x<min_thresh || turtle1_pose.y >max_thresh || turtle1_pose.y<min_thresh)
    {
        teleport1 = true;
        turtle1_commander.publish(zero_twist);
        x = turtle1_pose.x;
        y = turtle1_pose.y;
        pose1_command.request.x = std::max(std::min(max_thresh,x),min_thresh);
        pose1_command.request.y = std::max(std::min(max_thresh,y),min_thresh);
    }
        
    // checking boundries
    if(turtle2_pose.x>max_thresh || turtle2_pose.x<min_thresh || turtle2_pose.y >max_thresh || turtle2_pose.y<min_thresh)
    {
        teleport2 = true;
        turtle2_commander.publish(zero_twist);
        x = turtle2_pose.x;
        y = turtle2_pose.y;
        pose2_command.request.x = std::max(std::min(max_thresh,x),min_thresh);
        pose2_command.request.y = std::max(std::min(max_thresh,y),min_thresh);
    }
    if(teleport1)
        turtle1_pose_commander.call(pose1_command);
    if(teleport2)
        turtle2_pose_commander.call(pose2_command);





    
    // publish the distance
    std_msgs::Float32 dist_msg;
    dist_msg.data = dist;
    turtles_dist_pub.publish(dist_msg);
}

void monitor_turtle_pose(const turtlesim::Pose::ConstPtr& pose ,turtlesim::Pose &turtle_pose)
{
    // updating the pose
    turtle_pose.x = pose->x;
    turtle_pose.y = pose->y;
}


void monitor_turtle1_pose(const turtlesim::Pose::ConstPtr& pose)
{
    monitor_turtle_pose(pose,turtle1_pose);
}
void monitor_turtle2_pose(const turtlesim::Pose::ConstPtr& pose)
{
    monitor_turtle_pose(pose,turtle2_pose);
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