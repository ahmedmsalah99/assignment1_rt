#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include <string>
#include "unistd.h"
#include<iostream>


ros::Publisher turtle1_commander;
ros::Publisher turtle2_commander;


bool is_number(const std::string& s)
{
    char* endl = nullptr;
    double val = strtod(s.c_str(), &endl);
    return endl != s.c_str() && *endl == '\0' && val != HUGE_VAL;
}




void UI()
{
    
   geometry_msgs::Twist twist;
    geometry_msgs::Twist zero_twist;
    std::vector<double> twist_vals{0,0,0};
    std::string turtle_name;
    std::string robot_twise;
    while(1){
        // initializations
        turtle_name="";
        robot_twise = "";
        
        
        // take the name of the turtle as input
        while (turtle_name!="turtle1" && turtle_name!="turtle2")
        {
            std::cout << "Enter turtle name: " << std::endl;
            std::cin >> turtle_name;
        }
        // take the twist of the turtle as input
        std::vector<std::string> user_msgs = std::vector<std::string>({"x: ","y: ","yaw: "});
        
        for (int i=0;i<3;i++)
        {
            while(!is_number(robot_twise))
            {
                std::cout << user_msgs[i] << std::endl;
                std::cin >> robot_twise;
            }
            twist_vals[i] = stod(robot_twise);
            robot_twise = "";

        }
        // assign the twist
        twist.linear.x = twist_vals[0];
        twist.linear.y = twist_vals[1];
        twist.angular.z = twist_vals[2];
        // start publishing
        if(turtle_name == "turtle1")
            turtle1_commander.publish(twist);
        else
            turtle2_commander.publish(twist);
        sleep(1);
        if(turtle_name == "turtle1")
            turtle1_commander.publish(zero_twist);
        else
            turtle2_commander.publish(zero_twist);

    }

}






int main (int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS //system  
	ros::init(argc, argv, "UI");  
	ros::NodeHandle nh;

    // Initialize the spawner
    ros::ServiceClient spawner =  nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtle1_commander = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
    turtle2_commander = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",1);

    // spawn turtle2
    turtlesim::Spawn srv;
    srv.request.x = 10.5;  
	srv.request.y = 11.0/2;
	srv.request.theta = 0.0;
	srv.request.name = "turtle2";
    spawner.call(srv);
    
    // make UI work
    UI();

    ros::spin();
    return 0;
}