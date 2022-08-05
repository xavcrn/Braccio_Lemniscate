#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24Network.h>

// Choose the number of the emitter (between 0 and 5) (the ID is EMITTER+1)

#define EMITTER 5

#define targetedNode 0
#define xPin A0
#define yPin A1
#define zPin A2

// enable the DEBUG mode by uncommenting this line
//#define DEBUG

// these values were measured using DEBUG mode
int16_t x0[6]   = {344, 346, 346, 334, 340, 340};
int16_t xmax[6] = {409, 412, 413, 401, 405, 405};
int16_t xmin[6] = {274, 275, 264, 267, 270, 269};

int16_t y0[6]   = {330, 339, 333, 333, 338, 334};
int16_t ymax[6] = {400, 410, 402, 402, 405, 404};
int16_t ymin[6] = {264, 275, 275, 265, 268, 267};

int16_t z0[6]   = {344, 339, 341, 339, 336, 341};
int16_t zmax[6] = {413, 413, 407, 407, 408, 408};
int16_t zmin[6] = {277, 275, 275, 274, 275, 274};


#define lengthFilter 16
// frequency of the sending is of 1/PERIOD kHz
#define PERIOD 10


struct eggData {
  uint8_t id = EMITTER + 1;
  uint8_t x;
  uint8_t y;
  uint8_t z;
} data;

RF24 radio(9, 10); //emission with Arduino Nano-rf
RF24Network network(radio);

int X[lengthFilter] = {0};
int Y[lengthFilter] = {0};
int Z[lengthFilter] = {0};

// Adresses of the nodes
// The Network Layer can only adress 5 child per parent
// To use the 6 eggs, we have to use a child node as a relay to reach the mother
// Here, the 05 node is used to do so
// The 6th emitter has the adress octal(15) = (1<<3)|5 to comunicate by to the 5th node
const uint16_t nodes[6] = {1, 2, 3, 4, 5, (1<<3)|5};

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  radio.begin();
  delay(200);
#if EMITTER != 4
  // Only the node 04 need to listen to another node
  radio.stopListening();
#endif
  delay(200);
  network.begin(108, nodes[EMITTER]);
  delay(200);
#ifdef DEBUG
  Serial.println("setup done");
#endif
}

void loop() {
  static long int temps = millis();
  static int k = 0;
  X[k] = analogRead(xPin);
  Y[k] = analogRead(yPin);
  Z[k] = analogRead(zPin);

  if (k == lengthFilter - 1) {
    k = 0;
  } else {
    k++;
  }

  if (millis() - temps > PERIOD) {
    network.update();
    RF24NetworkHeader nHeader(targetedNode);
    int mX = 0;
    int mY = 0;
    int mZ = 0;
    for (int k = 0; k < lengthFilter; k++) {
      mX += constrain(X[k], xmin[EMITTER], xmax[EMITTER]);
      mY += constrain(Y[k], ymin[EMITTER], ymax[EMITTER]);
      mZ += constrain(Z[k], zmin[EMITTER], zmax[EMITTER]);
    }
    mX /= lengthFilter;
    mY /= lengthFilter;
    mZ /= lengthFilter;

    data.x = constrain(map(mX, xmin[EMITTER], x0[EMITTER], -128, 0), -128, 127) + 128;
    data.y = constrain(map(mY, ymin[EMITTER], y0[EMITTER], -128, 0), -128, 127) + 128;
    data.z = constrain(map(mZ, zmin[EMITTER], z0[EMITTER], -128, 0), -128, 127) + 128;

#ifdef DEBUG
    Serial.print("ID : ");
    Serial.print(data.id);
    Serial.print(" adress : ");
    Serial.println(nodes[EMITTER], OCT);
    Serial.print("X : ");
    Serial.print(analogRead(xPin));
    Serial.print("  dX = ");
    Serial.println(data.x);
    Serial.print("Y : ");
    Serial.print(analogRead(yPin));
    Serial.print("  dY = ");
    Serial.println(data.y);
    Serial.print("Z : ");
    Serial.print(analogRead(zPin));
    Serial.print("  dZ = ");
    Serial.println(data.z);
    Serial.println("");
#endif


    network.write(nHeader, &data, sizeof(data));

    temps = millis();
  }
}
