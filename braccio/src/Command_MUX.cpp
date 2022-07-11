#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "trajectory_msgs/JointTrajectory.h"

bool eggControl = false; //egg control is desactivate by default

void eggChange(std_msgs::Bool msg);
void eggTargetUpdate(trajectory_msgs::JointTrajectory msg);
void IKTargetUpdate(trajectory_msgs::JointTrajectory msg);

ros::Publisher* pub;

int main(int argc, char* argv[]){
    ros::init(argc, argv, "Command_MUX");
    ros::NodeHandle n;
    ros::Publisher robotPos_pub = n.advertise<trajectory_msgs::JointTrajectory>("robot_target_pos", 5);
    pub = &robotPos_pub;
    ros::Subscriber command_sub = n.subscribe("command_source",5, eggChange);
    ros::Subscriber egg_target_pos_sub = n.subscribe("egg_target_pos",5,eggTargetUpdate);
    ros::Subscriber IK_target_pos_sub = n.subscribe("IK_target_pos",5,IKTargetUpdate);
    ROS_INFO("Command_MUX initialized");
    ros::spin();
}

void eggChange(std_msgs::Bool msg){
    eggControl = msg.data;
}

void eggTargetUpdate(trajectory_msgs::JointTrajectory msg){
    if(eggControl){
        pub->publish(msg);
    }
}

void IKTargetUpdate(trajectory_msgs::JointTrajectory msg){
    if(!eggControl){
        pub->publish(msg);
    }
}