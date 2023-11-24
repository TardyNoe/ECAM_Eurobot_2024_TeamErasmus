#include <Wire.h>
#include <Adafruit_MotorShield.h>

bool Ldir = true;
bool Rdir = true;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorR = AFMS.getMotor(1);
Adafruit_DCMotor *motorL = AFMS.getMotor(2);

int RateCalibrationNumber;
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float angle = 0;

float xpos = 0;
float ypos = 0;

int datax = 0;
int datay = 0;
int dataa = 0;
int datadist = 0;
int counterR=0;
int counterL=0;
float PrevCounter = 0;
float DeltaT = 0;
bool hasMoved = false;
const int trigPin = 9;
const int echoPin = 10;
int angleGoal = 0;
int power = 0;

void sendData(int posx,int posy, int angle,int distance){
  Wire.beginTransmission(8);
  int dataToSend[4] = {posx, posy, angle,distance}; 
  for (int i = 0; i < 4; i++) {
    byte highByte = highByte(dataToSend[i]); 
    byte lowByte = lowByte(dataToSend[i]);  
    Wire.write(highByte);
    Wire.write(lowByte);
  }
  Wire.endTransmission();
}
void requestData(){
  Wire.requestFrom(8, 4); 
  int receivedData[2];
  int index = 0;
  while (Wire.available()) {
    byte high = Wire.read();
    byte low = Wire.read();
    receivedData[index++] = word(high, low); 
  }
  power = receivedData[0];
  angleGoal = receivedData[1];

}
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 10000);
  float distance = duration * 0.034 / 2;
  return distance;
}

void Rshaft_moved(){
  if(Rdir){
    counterR++;    
  }else{
    counterR--;
  }

  hasMoved = true;
  delayMicroseconds(50);
}
void Lshaft_moved(){
  if(Ldir){
    counterL++;
  }else{
    counterL--;
  }
  hasMoved = true;
  delayMicroseconds(50);
}
void gyro_signals(void) {     
  Serial.println("1");                                 
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  byte error = Wire.endTransmission();
  Serial.println("2");   
  delay(1);
  Wire.requestFrom(0x68,6);
  Serial.println("3");   
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RatePitch=((float)GyroX/65.5)*1.55;
  RateRoll=((float)GyroY/65.5)*1.5;
  RateYaw=-((float)GyroZ/65.5);
  Serial.println("4"); 
}

void control_motor(float L,float R){
  if(L>200){
    L = 200;
  }
  if(R>200){
    R = 200;
  }
  if(L<-200){
    L = -200;
  }
  if(R<-200){
    R = -200;
  }
  motorR->setSpeed(abs(R));
  motorL->setSpeed(abs(L));
  if(L>0){
    Ldir = true;
    motorL->run(FORWARD);
  }else{
    Ldir = false;
    motorL->run(BACKWARD);
  }
  if(R>0){
    Rdir = true;
    motorR->run(FORWARD);
  }else{
    Rdir = false;
    motorR->run(BACKWARD);
  }
  
}
void setupIMU(){
  Wire.setClock(300000);
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1D);
  Wire.write(0b00000110);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();             
  for (RateCalibrationNumber=0;RateCalibrationNumber<2000;RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000; 
}
void setupMotor(){
  AFMS.begin();
  motorR->setSpeed(100);
  motorL->setSpeed(100);
  motorR->run(FORWARD);
  motorL->run(FORWARD);
  attachInterrupt(digitalPinToInterrupt(2),  Rshaft_moved, FALLING);
  attachInterrupt(digitalPinToInterrupt(3),  Lshaft_moved, FALLING);  
}
void setupUltrasonic(){
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
}
void setup() {  
  Serial.begin(115200);
  setupIMU();
  setupUltrasonic();
  setupMotor();
  LoopTimer=micros();
}
void loop() {
  requestData();
  
  gyro_signals();
  
  RateRoll = RateRoll - RateCalibrationRoll;
  RatePitch = RatePitch - RateCalibrationPitch;
  RateYaw = RateYaw - RateCalibrationYaw;
  Serial.println("5");
  int dt = micros() - LoopTimer;
  angle = angle + RateYaw*(dt)*0.000001;
  
  LoopTimer=micros();
  float Error = angle+angleGoal;
  float rotSpeed = 4*(Error);
  control_motor(power+rotSpeed,power-rotSpeed);
  
  if(hasMoved){
    DeltaT = (counterR+counterL)/2 - PrevCounter;
    PrevCounter = (counterR+counterL)/2;
    hasMoved = false;
    xpos = xpos + DeltaT*cos(radians(angle));
    ypos = ypos + DeltaT*sin(radians(angle));
  }
   datax = xpos*10+32768;
   datay = ypos*10+32768;
   dataa = angle*10+32768;
   
   //datadist = getDistance();
   datadist = 0;
  
  if (datadist>100){
    datadist = 0;
 }
  sendData(datax,datay,dataa,datadist+32768);
  
  
}
