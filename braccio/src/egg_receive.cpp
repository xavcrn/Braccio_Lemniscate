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

#define PIPE_TRANSFER "/home/poppy/catkin_ws/src/braccio/receiver/temp/transferToPython"
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
    ros::Publisher pub = n.advertise<braccio::egg_angles>("eggs_angles",5);
    char buff[10];
    
    ROS_INFO("Open pipe for reading");
    int fd = open(PIPE_TRANSFER, O_RDONLY);
    if(fd == -1){
        ROS_INFO("Pipe opening error");
        exit(-1);
    }
    ROS_INFO("Pipe opened for reading");

    int sX[3] = {0,0,0};
    int sY[3] = {0,0,0};
    int nX[3] = {1,1,1};
    int nY[3] = {1,1,1};

    braccio::egg_angles angles;

    int currentTime;
    int lastTime = ros::Time::now().toNSec();

    int ID;
    int X;
    int Y;

    while(ros::ok()){
        read(fd,buff,10);
        ID = (0xFF00 & (buff[0] <<  8)) | (0xFF & buff[1]);
        X  = (0xFF000000 & (buff[2] << 24)) | (0xFF0000 & (buff[3] << 16)) | (0xFF00 & (buff[4] << 8)) | (0xFF & buff[5]);
        X  = (0xFF000000 & (buff[6] << 24)) | (0xFF0000 & (buff[7] << 16)) | (0xFF00 & (buff[8] << 8)) | (0xFF & buff[9]);
        
        ROS_INFO("%c %c %c %c %c %c %c %c %c %c",buff[0],buff[1],buff[2],buff[3],buff[4],buff[5],buff[6],buff[7],buff[8],buff[9]);
        ROS_INFO("ID=%d   x=%3d   y=%3d",ID,X,Y);

        switch(ID){
            case 0:
                sX[0] += X;
                sY[0] += Y;
    			nX[0]++;
	    		nY[0]++;
                break;
            case 1:
                sX[1] += X;
                sY[1] += Y;
    			nX[1]++;
	    		nY[1]++;
                break;
            case 3:
                sX[2] += X;
                sY[2] += Y;
    			nX[2]++;
	    		nY[2]++;
                break;
            default: 
                break;
        }
        
        currentTime = ros::Time::now().toNSec();
        if(currentTime < lastTime){
            lastTime = currentTime;
        }
        if(currentTime - lastTime > DELAY){
            lastTime += DELAY;

            for(int k = 0; k<3; k++){
                sX[k] = sX[k]/nX[k];
                sY[k] = sY[k]/nX[k];
                nX[k] = 1;
                nY[k] = 1;
            }

            angles.m0 = resize(sX[0], -180, 180);
			angles.m1 = resize(sY[0], -120, 120);
			angles.m2 = resize(sX[1], -120, 120);
			angles.m3 = resize(sY[1], -100, 100);
			angles.m4 = resize(sX[2], -180, 180);
			angles.m5 = resize(sY[2],  -90,   0);

            pub.publish(angles);

            ROS_INFO("angles : %4d %4d %4d %4d %4d %4d",angles.m0,angles.m1,angles.m2,angles.m3,angles.m4,angles.m5);
        }       
    }

    char cmd[256];
    ROS_INFO("shutdown receive");
    sprintf(cmd,"sudo kill -9 %d",receiver_PID);
    system(cmd);
    ROS_INFO("receive shutdown");
    return 0;
}

int resize(int x, int min, int max){
    if(x < 255){
        x = 255;
    } else if(x > 512){
        x = 512;
    }
    int a = (max - min) / 255;
    int b = min - a * 255;
    
    return a*x + b;
}