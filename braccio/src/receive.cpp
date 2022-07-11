#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sstream>

#include <time.h>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"

#include <RF24/RF24.h>
#include <RF24Network/RF24Network.h>
#include <RF24/nRF24L01.h>
    
using namespace std;

#define deb0 3.14159  // 180°
#define deb1 2.0944   // 120°
#define deb2 2.0944   // 120°
#define deb3 1.74533  // 100°
#define deb4 3.14159  // 180°
#define deb5 1.5708   //  90°

#define sendPeriod 0.1 // 10Hz

struct data {
	short ID;
	short x;
	short y;
	short mode;
	short action;
	short file;
} receivedData;

int main(int argc, char const *argv[])
{
    ros::init(argc, argv, "receive_eggs");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("eggs_target_pos",10);

    RF24 radio(25,0);
    RF24Network network(radio);
    const uint16_t motherNode = 00;
    const uint16_t myNode = motherNode;
   
    //inits radio
    radio.begin();
    radio.startListening();
    network.begin(108, myNode);

    trajectory_msgs::JointTrajectory goal;

    goal.joint_names.resize(6); // set the number of joint names of the trajectory to 6

    // initialize joint names
    goal.joint_names.push_back("base");
    goal.joint_names.push_back("epaule");
    goal.joint_names.push_back("coude");
    goal.joint_names.push_back("poignet_incl");
    goal.joint_names.push_back("poignet_rot");
    goal.joint_names.push_back("pince_mot");

    goal.points.resize(1); // set the number of points of the trajectory to 1

    goal.points[0].positions.resize(6); // set the number of positions to 6

    // intialize the first position to 0
    goal.points[0].positions[0] = 0;
    goal.points[0].positions[1] = 0;
    goal.points[0].positions[2] = 0;
    goal.points[0].positions[3] = 0;
    goal.points[0].positions[4] = 0;
    goal.points[0].positions[5] = 0;

    ros::Time lastSentTime = ros::Time::now();

    while(ros::ok()){
        network.update();

        while(ros::ok() && network.available()){
            RF24NetworkHeader nHeader;
            
            network.read(nHeader, &receivedData, sizeof(receivedData));
            
            cout << receivedData.ID << endl;
            
            
            switch (receivedData.ID)
            {
            case 0:
                goal.points[0].positions[0] = (receivedData.x * deb0 * 2) / 256 - deb0;
                goal.points[0].positions[1] = (receivedData.y * deb1 * 2) / 256 - deb1;
                break;
            
            case 1:
                goal.points[0].positions[2] = (receivedData.x * deb2 * 2) / 256 - deb2;
                goal.points[0].positions[3] = (receivedData.y * deb3 * 2) / 256 - deb3;
                break;

            case 2:
                goal.points[0].positions[4] = (receivedData.x * deb4 * 2) / 256 - deb4;
                goal.points[0].positions[5] = (receivedData.y * deb5) / 256;
                break;
                
            default:
                ROS_ERROR("Unexpected ID : %d",receivedData.ID);
                break;
            }

            // Send goal at 1/sendPeriod Hz
            if(ros::Time::now() - lastSentTime >= ros::Duration(sendPeriod)){
                pub.publish(goal);
                lastSentTime = ros::Time::now();
            }            
   	    }
        ROS_ERROR("Egg connection lost");
    }
    return 0;
}
