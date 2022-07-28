#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cstdlib>

#include "RF24/RF24.h"
#include "RF24Network/RF24Network.h"
#include "RF24/nRF24L01.h"
    
using namespace std;

#define PIPE_TRANSFER "/home/poppy/catkin_ws/src/braccio/receiver/angles.pipe"

struct data {
    uint8_t ID;
    uint8_t x;
    uint8_t y;
    uint8_t z;
} receivedData;


int main(int argc, char const *argv[]){
		cout << "receive_from_egg : Compilation date : " << __DATE__ << endl;
		cout << "receive_from_egg : Compilation hour : " << __TIME__ << endl;
    RF24 radio(25,0);
    RF24Network network(radio);
    const uint16_t motherNode = 00;
    const uint16_t myNode = motherNode;
   
    //inits radio
    radio.begin();
    radio.startListening();
    network.begin(108, myNode);
    sleep(1);

    #ifndef DEBUG
    int fd = open(PIPE_TRANSFER, O_WRONLY);
		cout << "receive_from_egg : Pipe opened for writing" << endl;
    char buff[5];
    #endif
    
    

    while(true){
				network.update();

        while(network.available()){
            RF24NetworkHeader nHeader;
            
            network.read(nHeader, &receivedData, sizeof(receivedData));

            #ifdef DEBUG
            printf("receive_from_egg : received : ID = %d, x = %3d, y = %3d, z = %3d\n",receivedData.ID,receivedData.x,receivedData.y,receivedData.z);
            #endif

            int x = receivedData.x - 128;
            int y = receivedData.y - 128;
            int z = receivedData.z - 128;

            // Compute the x and y angle from the x, y and z component
            float OAx = sqrt(y*y+z*z);            
            float sina = z/OAx;
            float cosa = y/OAx;
            float ax; // x angle (rad)
            // Determine the angle
            // Use the mean value between the one from the cosinus and the one from the sinus
            if(cosa >= 0){
              if(sina >=0){
                ax = (asin(sina) + acos(cosa))/2;
              } else {
                ax = (asin(sina) - acos(cosa))/2;
              }
            } else {
              if(sina >=0){
                ax = (M_PI - asin(sina) + acos(cosa))/2;
              } else {
                ax = (asin(-sina) + acos(-cosa))/2 - M_PI;
              }
            }
            float OAy = sqrt(x*x+z*z);
            float ay;
            sina = z/OAy;
            cosa = x/OAy;
            if(cosa >= 0){
              if(sina >=0){
                ay = (asin(sina) + acos(cosa))/2;
              } else {
                ay = (asin(sina) - acos(cosa))/2;
              }
            } else {
              if(sina >=0){
                ay = (M_PI - asin(sina) + acos(cosa))/2;
                //cout << "a = " << asin(sina)*180/M_PI << endl << "a = " << acos(cosa)*180/M_PI << endl;
              } else {
                ay = (-asin(sina) + acos(cosa))/2 - M_PI;
              }
            }
            int16_t axd = ax*180/M_PI - 90; // x angle (°)
            int16_t ayd = ay*180/M_PI - 90;

            if(axd < -180){
              axd += 360;
            } else if (axd > 180){
              axd -= 360;
            }
            if(ayd < -180){
              ayd += 360;
            } else if (ayd > 180){
              ayd -= 360;
            }

            #ifdef DEBUG
            printf("x = %4d°   y = %4d°\n",axd, ayd);
            #else         
            /*  
            snprintf(buff, 5, "%d", (int16_t)receivedData.ID);
            write(fd, buff, 2);
            
            snprintf(buff, 5, "%d", axd);
            write(fd, buff, 4);

            snprintf(buff, 5, "%d", ayd);
            write(fd, buff, 4);
            */
            write(fd,&(receivedData.ID),sizeof(receivedData.ID));
            write(fd,&axd,sizeof(axd));
            write(fd,&ayd,sizeof(ayd));
            #endif

            #ifdef VERBOSE
            cout << "ID:" << (int16_t)receivedData.ID << " x:" << axd << " y:" << ayd << endl;
            #endif
            
   	    }
    }  
    #ifndef DEBUG
    close(fd);
    #endif
    return 0;
}
