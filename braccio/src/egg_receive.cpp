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
#include "braccio/egg_angles.h"

using namespace std;

#define PIPE_TRANSFER "/home/poppy/catkin_ws/src/braccio/receiver/angles.pipe"
#define DELAY 50000000

int resize(int x, int min, int max);

int main(int argc, char* argv[]){
    int receiver_PID = fork();
    if(receiver_PID == 0){
        int buf = system("sudo /home/poppy/catkin_ws/src/braccio/receiver/receive");
        return 0;
    }
    ros::init(argc, argv, "egg_receiver");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<braccio::egg_angles>("egg_angles",5);

    ROS_INFO("egg_receiver : Open pipe for reading");
    int fd = open(PIPE_TRANSFER, O_RDONLY);
    if(fd == -1){
        ROS_INFO("egg_receiver : Pipe opening error");
        exit(-1);
    }
    ROS_INFO("egg_receiver : Pipe opened for reading");

    /*
    int sX[3] = {0,0,0};
    int sY[3] = {0,0,0};
    int nX[3] = {0,0,0};
    int nY[3] = {0,0,0};
    */

    int S[6] = {0};
    int N[6] = {0};

    braccio::egg_angles angles;

    angles.m0 = 0;
    angles.m1 = 0;
    angles.m2 = 0;
    angles.m3 = 0;
    angles.m4 = 0;
    angles.m5 = 0;

    // Pointers are here used to make it iterable
    int16_t* ANGLES[6] = {&(angles.m0),&(angles.m1),&(angles.m2),&(angles.m3),&(angles.m4),&(angles.m5)};

    int currentTime;
    int lastTime = ros::Time::now().toNSec();

    int8_t ID;
    int16_t X;
    int16_t Y;

    while(ros::ok()){
        read(fd,&ID,1);
        read(fd,&X,2);
        read(fd,&Y,2);

        //ROS_INFO("egg_receiver :ID=%d   x=%3d   y=%3d",ID,X,Y);
        
        S[2*(ID-1)]   += X;
        S[2*(ID-1)+1] += Y;
        N[2*(ID-1)]++;
        N[2*(ID-1)+1]++;

        currentTime = ros::Time::now().toNSec();
        if(currentTime < lastTime){
            lastTime = currentTime;
        }
        if(currentTime - lastTime > DELAY){
            lastTime += DELAY;
            for(int k = 0; k<6; k++){
                if(N[k]){
                    *(ANGLES[k]) = S[k]/N[k];
                }
                N[k] = 0;
                S[k] = 0;
            }
            pub.publish(angles);

            //ROS_INFO("egg_receiver : %4d %4d %4d %4d %4d %4d",angles.m0,angles.m1,angles.m2,angles.m3,angles.m4,angles.m5);
        }
    }

    char cmd[256];
    ROS_INFO("egg_receiver : shutdown receive");
    sprintf(cmd,"sudo kill -9 %d",receiver_PID);
    system(cmd);
    ROS_INFO("egg_receiver : receive shutdown");
    return 0;
}

int resize(int x, int min, int max){
    float a = (max - min) / 360;
    float b = min + a * 180;
    return (int)(a*x + b);
}
