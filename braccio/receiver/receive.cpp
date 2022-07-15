#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cstdlib>

#include "../../rf24libs/RF24/RF24.h"
#include "../../rf24libs/RF24Network/RF24Network.h"
#include "../../rf24libs/RF24/nRF24L01.h"
    
using namespace std;

#define PIPE_TRANSFER "/home/poppy/catkin_ws/src/braccio/receiver/temp/transferToPython"

struct data {
    short ID;
    short x;
    short y;
    short mode;
    short action;
    short file;
} receivedData;


int main(int argc, char const *argv[]){
		cout << "Compilation date : " << __DATE__ << endl;
		cout << "Compilation hour : " << __TIME__ << endl;
    RF24 radio(25,0);
    RF24Network network(radio);
    const uint16_t motherNode = 00;
    const uint16_t myNode = motherNode;
   
    //inits radio
    radio.begin();
    radio.startListening();
    radio.printDetails();
    network.begin(108, myNode);
 
    //cout << "Debut radio " << endl; 
 		
/*
    if(mkfifo(PIPE_TRANSFER, S_IRUSR | S_IWUSR)== -1){
				cout << "pipe creation failed" << endl;
				exit(-1);
		}

    cout << "pipe initialized" << endl;
*/
		cout << "opening pipe file" << endl;
    int fd = open(PIPE_TRANSFER, O_WRONLY);

		cout << "pipe opened" << endl;

    char buff[10];
    while(true){
				network.update();

        while(network.available()){
            RF24NetworkHeader nHeader;
            
            network.read(nHeader, &receivedData, sizeof(receivedData));
            
            snprintf(buff, 10, "%d", receivedData.ID);
            write(fd, buff, 2);
            
            snprintf(buff, 10, "%d", receivedData.x);
            write(fd, buff, 4);
            
            snprintf(buff, 10, "%d", receivedData.y);
            write(fd, buff, 4);
						
						//cout << "ID : " << receivedData.ID << " X : " << receivedData.x << " Y : " << receivedData.y << endl;
   	    }
    }  
    
    close(fd);
    return 0;
}
