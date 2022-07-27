#define xPin A0
#define yPin A1
#define zPin A2

#define xmax 425
#define ymax 425
#define zmax 425
#define xmin 280
#define ymin 280
#define zmin 283

void setup(){
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(zPin, INPUT);

  Serial.begin(115200);
}

void loop(){
  static int pin[3] = { xPin,  yPin,  zPin};
  static int MAX[3] = {-1024, -1024, -1024};
  static int MIN[3] = { 1024,  1024,  1024};
  static long int temp = millis();
  int mes[3];
  for(int k = 0; k<3; k++){
    mes[k] = analogRead(pin[k]);
    if(mes[k] > MAX[k]){
      MAX[k] = mes[k];
    } else if(mes[k] < MIN[k]){
      MIN[k] = mes[k];
    }
  }
  if(millis() - temp > 250){
    Serial.print("X : ");
    Serial.print(mes[0]);
    Serial.print("   ");
    Serial.print(MIN[0]);
    Serial.print("   ");
    Serial.println(MAX[0]);
    Serial.print("Y : ");
    Serial.print(mes[1]);
    Serial.print("   ");
    Serial.print(MIN[1]);
    Serial.print("   ");
    Serial.println(MAX[1]);
    Serial.print("Z : ");
    Serial.print(mes[2]);
    Serial.print("   ");
    Serial.print(MIN[2]);
    Serial.print("   ");
    Serial.println(MAX[2]);
    Serial.println("");
    temp = millis();
  }
}
