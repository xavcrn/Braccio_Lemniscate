//#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24Network.h>

#define DEBUG

/*
      Number of emitter to change :
      EMITTER 1 : 1 (base / shoudler)
      EMITTER 2 : 2 (elbow / poignet ver)
      EMITTER 3 : 3 (poignet rot / gripper)
*/
#define EMITTER 1

#define targetedNode 0
#define xPin A0
#define yPin A1
#define zPin A2

#define x0 334
#define y0 330
#define z0 332

#define xmax 405
#define ymax 404
#define zmax 407

#define xmin 275
#define ymin 268
#define zmin 275


#define lengthFilter 16
// frequency of the sending is of 1/PERIOD kHz
#define PERIOD 100


struct eggData {
  uint8_t id = EMITTER;
  int8_t x;
  int8_t y;
  int8_t z;
}data;

RF24 radio(9,10); //emission with Arduino Nano-rf
RF24Network network(radio);

int X[lengthFilter] = {0};
int Y[lengthFilter] = {0};
int Z[lengthFilter] = {0};

void setup() {
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);  
  radio.begin();
  radio.stopListening();
  network.begin(108, EMITTER);
  delay(100);

  #ifdef DEBUG
  Serial.begin(115200);
  #endif  
}

void loop() {
  static long int temps = millis();
  static int k = 0;
  X[k] = analogRead(xPin);
  Y[k] = analogRead(yPin);
  Z[k] = analogRead(zPin);
  
  if(k==lengthFilter-1){
    k=0;
  } else {
    k++;
  }
  
  if(millis() - temps > PERIOD){
    int mX = 0;
    int mY = 0;
    int mZ = 0;
    for(int k = 0; k<lengthFilter; k++){
      mX += constrain(X[k],xmin,xmax);
      mY += constrain(Y[k],ymin,ymax);
      mZ += constrain(Z[k],zmin,zmax);
    }
    mX /= lengthFilter;
    mY /= lengthFilter;
    mZ /= lengthFilter;

    data.x = mX - x0; //map(mX-x0, xmin-x0, xmax-x0, 0, 255);
    data.y = mY - y0; //map(mY-y0, ymin-y0, ymax-y0, 0, 255);
    data.z = mZ - z0; //map(mZz0, zmin-z0, zmax-z0, 0, 255);

    #ifdef DEBUG
    Serial.print("X : ");
    Serial.println(data.x);
    Serial.print("Y : ");
    Serial.println(data.y);
    Serial.print("Z : ");
    Serial.println(data.z);
    Serial.println("");
    #endif
    
    network.update();
    RF24NetworkHeader nHeader(targetedNode);
    network.write(nHeader, &data, sizeof(data));

    temps = millis();
  }
}
