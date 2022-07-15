#define leftSpeed 6
#define leftDirection 7
#define rightSpeed 5
#define rightDirection 4

#define FORWARD LOW
#define BACKWARD HIGH

inline void stop(){
  analogWrite(leftSpeed, 0);
  analogWrite(rightSpeed, 0);
  digitalWrite(leftDirection, FORWARD);
  digitalWrite(rightDirection, BACKWARD);
}

void setup(){
  // Declare output pins as output
  pinMode(leftSpeed,      OUTPUT);
  pinMode(leftDirection,  OUTPUT);
  pinMode(rightSpeed,     OUTPUT);
  pinMode(rightDirection, OUTPUT);
  
  // Set initial speed to 0 and initial direction to forward
  stop();
  
  Serial.begin(115200);
}

void loop(){
  char buf[4];
  char n = Serial.readBytes(buf, 4);
  if(n != 4){
    stop();
    return;
  }
  if(buf[0]){
    digitalWrite(leftDirection, HIGH);
  } else {
    digitalWrite(leftDirection, LOW);
  }
  analogWrite(leftSpeed, buf[1]);
  
  if(buf[2]){
    digitalWrite(rightDirection, HIGH);
  } else {
    digitalWrite(rightDirection, LOW);
  }
  analogWrite(rightSpeed, buf[3]);
}