#include <nRF24L01.h>
#include <RF24.h>
#include <RF24Network.h>

const uint16_t motherNode =   0;
const uint16_t childNode  =   1;
const uint8_t  channel    = 108;

struct data {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  uint8_t d = 0;
}donnees;

RF24        radio(9, 10);
RF24Network network(radio);

#define RCV

void setup(){
  radio.begin();
  #ifdef RCV
    // RECVEIVER
    radio.startListening();
    network.begin(channel, motherNode);
  #else
    // EMITTER  
    radio.stopListening();
    network.begin(channel, childNode);
  #endif
  Serial.begin(115200);
  delay(100);
}

void loop(){
  network.update();
   #ifdef RCV
    // RECEIVER
    int k;
    RF24NetworkHeader nHeader(childNode);
    if(network.available()){
      network.read(nHeader, &donnees, sizeof(donnees));
      Serial.print("Received :");
      Serial.print(" a=");
      Serial.print(donnees.a);
      Serial.print(" b=");
      Serial.print(donnees.b);
      Serial.print(" c=");
      Serial.print(donnees.c);
      Serial.print(" d=");
      Serial.println(donnees.d);            
    }
  #else
    // EMITTER
    RF24NetworkHeader nHeader(motherNode);
    network.write(nHeader, &donnees, sizeof(donnees));
    Serial.print("Send : ");
    Serial.print(" a=");
    Serial.print(donnees.a++);
    Serial.print(" b=");
    Serial.print(donnees.b--);
    Serial.print(" c=");
    Serial.print(donnees.c++);
    Serial.print(" d=");
    Serial.println(donnees.d--);
    Serial.println();
    delay(100);
  #endif
}
