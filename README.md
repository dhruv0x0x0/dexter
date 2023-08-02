# dexter
#include <Wire.h>
#define CH1 11
#define CH2 10
#define CH3 9
#define CH4 8
#define CH5 7
#define CH6 6
#define CH7 5
#define CH8 4
#define CH9 3
#define CH10 2

int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue)
{
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100)
    return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

void setup()
{
  
  Wire.begin();
  Serial.begin(9600);
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  pinMode(CH7, INPUT);
  pinMode(CH8, INPUT);
  pinMode(CH9, INPUT);
  pinMode(CH10, INPUT);

}

void loop()
{
   //Serial.print("aa");
  int a[10];
  //Serial.println("45");
  for (int i = 0; i < 4; i++)
  {
    a[i] = readChannel(11 - i, 20, 220, 120);
  }
  
  for (int i = 4; i < 6; i++)
  {
    int x = readChannel(11 - i, 0, 1000, 0);
    if (x > 500)
      a[i] = 1;
    else
      a[i] = 0;

  }
  
  for (int i = 6; i < 8; i++)
  {
    a[i] = readChannel(11 - i, 10, 170, 90);
  }
  
  for (int i = 8; i < 10; i++)
  {
    int x = readChannel(11 - i, 0, 1000, 0);
    if (x > 500)
      a[i] = 1;
    else
      a[i] = 0;
  }
  
  if (a[8] == 0)
  {
    Wire.beginTransmission(10);
    Wire.write(a[8]);
    Wire.write(a[0]);
    Wire.write(a[1]);
    Wire.write(a[2]);
    Wire.write(a[3]);
    Wire.endTransmission();
    Wire.beginTransmission(12);
    Wire.write(a[8]);
    Wire.write(a[0]);
    Wire.write(a[1]);
    Wire.write(a[2]);
    Wire.write(a[3]);
    Wire.endTransmission();
    Wire.beginTransmission(8);
    Wire.write(a[8]);
    Wire.write(a[6]);
    Wire.write(a[7]);
    Wire.write(a[4]);
    Wire.write(a[5]);
    Wire.endTransmission();
  }
  
  if (a[8] == 1)
  {
    Wire.beginTransmission(10);
    Wire.write(a[8]);
    Wire.endTransmission();
    Wire.beginTransmission(12);
    Wire.write(a[8]);
    Wire.endTransmission();
    Wire.beginTransmission(8);
    Wire.write(a[8]);
    Wire.endTransmission();
  }
  
  for (int i = 0; i < 10; i++)
  {
    Serial.print(a[i]);
    Serial.print(" ");
    Serial.print(":");
    Serial.print(i);
    Serial.print(" | ");
  }
  
  Serial.println();
  delay(10);
}
//////////
#include <ArduinoHardware.h>
#include<Wire.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
const int pwm_1 = 10;
const int pwm_2 = 11;
const int dir_1 = 6;
const int dir_2 = 5;
float w_r = 0, w_l = 0;

float wheel_rad = 0.0325, wheel_sep = 0.295; //wheel_rad is the wheel radius ,wheel_sep is seperation
ros::NodeHandle nh;
float vel_x = 0, vel_y = 0, ang_z = 0;
int a[] = {0, 0, 0, 0, 0};
float spd = 1.0;
float turn = 1.0;
// Rotary Encoder Inputs
#define Clock 9 //Clock pin connected to D9
#define Data 8 //Data pin connected to D8
//#define Push 10 //Push button pin connected to D10
#define CLK 2
#define DT 3

int c = 0;
int currentStateCLK;
int lastStateCLK;
int p_c=0;
//String currentDir ="";
int counter = 0; //Use this variable to store "steps"
int previous_counter = 0; //Use this variable to store previous "steps" value
int currentStateClock; //Store the status of the clock pin (HIGH or LOW)
int StateData; //Store the status of the data pin (HIGH or LOW)
int lastStateClock; //Store the PREVIOUS status of the clock pin (HIGH or LOW)
String currentDir =""; //Use this to print text
//unsigned long lastButtonPress = 0; //Use this to store if the push button was pressed or not
void messageCb( const geometry_msgs::Twist& msg)
{

  vel_x = msg.linear.x;
  vel_y = msg.linear.y;
  ang_z = msg.angular.z;
  give_vel();

}

void give_vel()
{

  if (ang_z != 0)
  {
    w_l = ang_z / wheel_rad;
    w_r = -ang_z / wheel_rad;
  }
  else if (vel_x > 0 && vel_y > 0)
  {
    w_l = 0;
    w_r = vel_x / wheel_rad;
  }
  else if (vel_x > 0 && vel_y == 0)
  {
    w_l = vel_x / wheel_rad;
    w_r = vel_x / wheel_rad;
  }
  else if (vel_x > 0 && vel_y < 0)
  {
    w_l = vel_x / wheel_rad;
    w_r = 0;
  }
  else if (vel_x == 0 && vel_y > 0)
  {
    w_l = -vel_y / wheel_rad;
    w_r = vel_y / wheel_rad;
  }
  else if (vel_x == 0 && vel_y == 0)
  {
    w_l = 0;
    w_r = 0;
  }
  else if (vel_x == 0 && vel_y < 0)
  {
    w_l = -vel_y / wheel_rad;
    w_r = vel_y / wheel_rad;
  }
  else if (vel_x < 0 && vel_y > 0)
  {
    w_l = vel_x / wheel_rad;
    w_r = 0;
  }
  else if (vel_x < 0 && vel_y == 0)
  {
    w_l = vel_x / wheel_rad;
    w_r = vel_x / wheel_rad;
  }
  else if (vel_x < 0 && vel_y < 0)
  {
    w_l = 0;
    w_r = vel_x / wheel_rad;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

float y(int n)
{
  if (n > 180)
    return 1.0;
  if (n < 10)
    return 0.0;
  if (n < 50)
    return -1.0;
  return 0.0;
}

void rc_vel()
{
  vel_x = y(a[2]) * spd;
  vel_y = y(a[1]) * spd;
  ang_z = y(a[4]) * spd;
}

void Motors_init()
{
  pinMode(dir_1, OUTPUT);
  pinMode(pwm_1, OUTPUT);
  pinMode(dir_2, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  digitalWrite(dir_1, LOW);
  digitalWrite(pwm_1, LOW);
  digitalWrite(dir_2, LOW);
  digitalWrite(pwm_2, LOW);
}

void receiveEvent(int howMany)
{
  if (Wire.available())
  {
    int x = Wire.read();
    if (x == 0)
    {
      a[0] = x;
      for (int i = 1; i < 5; i++)
      {
        a[i] = Wire.read();
      }
      rc_vel();
      give_vel();

      for (int i = 0; i < 5; i++)
      {
        Serial.print(" : ");
        Serial.print(a[i]);
      }
    }
    if (x == 1)
    {
      a[0] = x;
      Serial.print("Auto Mode : ");
      Serial.print(x);
    }
    Serial.println();

  }

}

void MotorL(int Pulse_Width1)
{
  if (Pulse_Width1 > 0)
  {
    digitalWrite(dir_1, HIGH);
    analogWrite(pwm_1, Pulse_Width1);
  }
  if (Pulse_Width1 < 0)
  {
    Pulse_Width1 = abs(Pulse_Width1);
    digitalWrite(dir_1, LOW);
    analogWrite(pwm_1, Pulse_Width1);
  }
  if (Pulse_Width1 == 0)
  {
    digitalWrite(dir_1, LOW);
    digitalWrite(pwm_1, 0);
  }
}

void MotorR(int Pulse_Width2)
{
  if (Pulse_Width2 > 0)
  {
    digitalWrite(dir_2, LOW);
    analogWrite(pwm_2, Pulse_Width2);
  }
  if (Pulse_Width2 < 0)
  {
    Pulse_Width2 = abs(Pulse_Width2);
    digitalWrite(dir_2, HIGH);
    analogWrite(pwm_2, Pulse_Width2);
  }
  if (Pulse_Width2 == 0)
  {
    digitalWrite(dir_2, LOW);
    digitalWrite(pwm_2, 0);
  }
}

void setup()
{
  Motors_init();
  nh.initNode();
  nh.subscribe(sub);
  Wire.begin(10);
  Wire.onReceive(receiveEvent);
  Serial.begin(57600);
  pinMode(Clock,INPUT_PULLUP);
pinMode(Data,INPUT_PULLUP);
pinMode(CLK,INPUT_PULLUP);
pinMode(DT,INPUT_PULLUP);
//pinMode(Push, INPUT_PULLUP);

//Here we activate pin change interruptions on pin D8 and D9 with PCINT0 and PCINT1
PCICR |= (1 << PCIE0); //enable PCMSK0 scan
PCMSK0 |= (1 << PCINT0); //Pin 8 (Data) interrupt. Set pin D8 to trigger an interrupt on state change.
PCMSK0 |= (1 << PCINT1); //Pin 9 (Clock) interrupt. Set pin D9 to trigger an interrupt on state change.
// Read the initial state of Clock pin (it could be HIGH or LOW)
lastStateClock = digitalRead(Clock);
lastStateCLK = digitalRead(CLK);

// Call updateEncoder() when any high/low changed seen
// on interrupt 0 (pin 2), or interrupt 1 (pin 3)
attachInterrupt(0, updateEncoder, CHANGE);
attachInterrupt(1, updateEncoder, CHANGE);


}

void loop() {
  if (a[0] == 0)
  {
    rc_vel();
    give_vel();
//    for (int i = 0; i < 5; i++)
//    {
//      Serial.print(" : ");
//      Serial.print(a[i]);
//    }
  }
  //Serial.println();
  Serial.print(w_l);
  Serial.print(" : ");
  Serial.println(w_r);
  MotorR(w_l * 10);
  MotorL(w_r * 10);
  nh.spinOnce();
  //delay(50);
  if(counter != previous_counter || c!=p_c)
{
Serial.print("Right Counter: ");
Serial.print(counter);
Serial.print(" |:| Left Counter: ");
Serial.println(c);
}
previous_counter=counter;
p_c=c;
delay(1);
}
ISR (PCINT0_vect){
cli(); //We pause interrupts happening before we read pin values
currentStateClock = digitalRead(Clock); //Check pin D9 state? Clock
StateData = digitalRead(Data); //Check pin D8 state? Data
if (currentStateClock != lastStateClock && currentStateClock == 1){
// If "clock" state is different "data" state, the encoder is rotating clockwise
if (StateData != currentStateClock){
counter --; // We increment
//lastStateClock = currentStateClock; // Updates the previous state of the clock with the current state
//sei(); //restart interrupts
}
//Else, the encoder is rotating counter-clockwise
else {
counter ++; // We decrement
//lastStateClock = currentStateClock; // Updates previous state of the clock with the current state
//sei(); //restart interrupts
}

}
lastStateClock = currentStateClock;
sei();
}
void updateEncoder(){
// Read the current state of CLK
currentStateCLK = digitalRead(CLK);

// If last and current state of CLK are different, then pulse occurred
// React to only 1 state change to avoid double count
if (currentStateCLK != lastStateCLK && currentStateCLK == 1){

// If the DT state is different than the CLK state then
// the encoder is rotating CCW so decrement
if (digitalRead(DT) != currentStateCLK) {
c--;
//currentDir ="CCW";
} else {
// Encoder is rotating CW so increment
c++;
//currentDir ="CW";
}

//Serial.print("Direction: ");
//Serial.print(currentDir);
//Serial.print(" | Counter: ");
//Serial.println(counter);
}

// Remember last CLK state
lastStateCLK = currentStateCLK;
}

////////
#include <ArduinoHardware.h>
#include<Wire.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
const int pwm_1 = 10;
const int pwm_2 = 11;
const int dir_1 = 6;
const int dir_2 = 5;
float w_r = 0, w_l = 0;

float wheel_rad = 0.0325, wheel_sep = 0.295; //wheel_rad is the wheel radius ,wheel_sep is seperation
ros::NodeHandle nh;
float vel_x = 0, vel_y = 0, ang_z = 0;
int a[] = {0, 0, 0, 0, 0};
float spd = 1;
float turn = 1.0;
#define Clock 9 //Clock pin connected to D9
#define Data 8 //Data pin connected to D8
//#define Push 10 //Push button pin connected to D10
#define CLK 2
#define DT 3

int c = 0;
int currentStateCLK;
int lastStateCLK;
int p_c=0;
//String currentDir ="";
int counter = 0; //Use this variable to store "steps"
int previous_counter = 0; //Use this variable to store previous "steps" value
int currentStateClock; //Store the status of the clock pin (HIGH or LOW)
int StateData; //Store the status of the data pin (HIGH or LOW)
int lastStateClock; //Store the PREVIOUS status of the clock pin (HIGH or LOW)
String currentDir =""; //Use this to print text
void messageCb( const geometry_msgs::Twist& msg)
{

  vel_x = msg.linear.x;
  vel_y = msg.linear.y;
  ang_z = msg.angular.z;
  give_vel();

}

void give_vel(){
  
  if(ang_z!=0)
  {
    w_l = -ang_z/wheel_rad;
    w_r = ang_z/wheel_rad;
  }
  else if(vel_x>0 && vel_y>0)
  {
    w_l = 0;
    w_r = vel_x/wheel_rad;
  }
  else if(vel_x>0 && vel_y==0)
  {
    w_l = vel_x/wheel_rad;
    w_r = vel_x/wheel_rad;
  }
  else if(vel_x>0 && vel_y<0)
  {
    w_l = vel_x/wheel_rad;
    w_r = 0;
  }
  else if(vel_x==0 && vel_y>0)
  {
    w_l = -vel_y/wheel_rad;
    w_r = vel_y/wheel_rad;
  }
  else if(vel_x==0 && vel_y==0)
  {
    w_l = 0;
    w_r = 0;
  }
  else if(vel_x==0 && vel_y<0)
  {
    w_l = -vel_y/wheel_rad;
    w_r = vel_y/wheel_rad;
  }
  else if(vel_x<0 && vel_y>0)
  {
    w_l = vel_x/wheel_rad;
    w_r = 0;
  }
  else if(vel_x<0 && vel_y==0)
  {
    w_l = vel_x/wheel_rad;
    w_r = vel_x/wheel_rad;
  }
  else if(vel_x<0 && vel_y<0)
  {
    w_l = 0;
    w_r = vel_x/wheel_rad;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

float y(int n)
{
  if (n > 150)
    return 1.0;
  if (n > 100 && n<140)
   return 0.0;
  if (n < 100)
    return -1.0;
  return 0.0;
}

void rc_vel()
{
  vel_x = y(a[2]) * spd;
  vel_y = y(a[1]) * spd;
    Serial.println(vel_y);

  ang_z = y(a[4]) * spd;
}

void Motors_init()
{
  pinMode(dir_1, OUTPUT);
  pinMode(pwm_1, OUTPUT);
  pinMode(dir_2, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  digitalWrite(dir_1, LOW);
  digitalWrite(pwm_1, LOW);
  digitalWrite(dir_2, LOW);
  digitalWrite(pwm_2, LOW);
}

void receiveEvent(int howMany)
{
  if (Wire.available())
  {
    int x = Wire.read();
    if (x == 0)
    {
      a[0] = x;
      for (int i = 1; i < 5; i++)
      {
        a[i] = Wire.read();
      }
      rc_vel();
      give_vel();

      for (int i = 0; i < 5; i++)
      {
        Serial.print(" : ");
        Serial.print(a[i]);
      }
    }
    if (x == 1)
    {
      a[0] = x;
      Serial.print("Auto Mode : ");
      Serial.print(x);
    }
    Serial.println();

  }

}

void MotorL(int Pulse_Width1)
{
  if (Pulse_Width1 > 0)
  {
    digitalWrite(dir_1, HIGH);
    analogWrite(pwm_1, Pulse_Width1);
  }
  if (Pulse_Width1 < 0)
  {
    Pulse_Width1 = abs(Pulse_Width1);
    digitalWrite(dir_1, LOW);
    analogWrite(pwm_1, Pulse_Width1);
  }
  if (Pulse_Width1 == 0)
  {
    digitalWrite(dir_1, LOW);
    digitalWrite(pwm_1, 0);
  }
}

void MotorR(int Pulse_Width2)
{
  if (Pulse_Width2 > 0)
  {
    digitalWrite(dir_2, LOW);
    analogWrite(pwm_2, Pulse_Width2);
  }
  if (Pulse_Width2 < 0)
  {
    Pulse_Width2 = abs(Pulse_Width2);
    digitalWrite(dir_2, HIGH);
    analogWrite(pwm_2, Pulse_Width2);
  }
  if (Pulse_Width2 == 0)
  {
    digitalWrite(dir_2, LOW);
    digitalWrite(pwm_2, 0);
  }
}

void setup()
{
  Motors_init();
  nh.initNode();
  nh.subscribe(sub);
  Wire.begin(12);
  Wire.onReceive(receiveEvent);
  Serial.begin(57600);
   pinMode(Clock,INPUT_PULLUP);
pinMode(Data,INPUT_PULLUP);
pinMode(CLK,INPUT_PULLUP);
pinMode(DT,INPUT_PULLUP);
//pinMode(Push, INPUT_PULLUP);

//Here we activate pin change interruptions on pin D8 and D9 with PCINT0 and PCINT1
PCICR |= (1 << PCIE0); //enable PCMSK0 scan
PCMSK0 |= (1 << PCINT0); //Pin 8 (Data) interrupt. Set pin D8 to trigger an interrupt on state change.
PCMSK0 |= (1 << PCINT1); //Pin 9 (Clock) interrupt. Set pin D9 to trigger an interrupt on state change.
// Read the initial state of Clock pin (it could be HIGH or LOW)
lastStateClock = digitalRead(Clock);
lastStateCLK = digitalRead(CLK);

// Call updateEncoder() when any high/low changed seen
// on interrupt 0 (pin 2), or interrupt 1 (pin 3)
attachInterrupt(0, updateEncoder, CHANGE);
attachInterrupt(1, updateEncoder, CHANGE);
if(counter != previous_counter || c!=p_c)
{
Serial.print("Right Counter: ");
Serial.print(counter);
Serial.print(" |:| Left Counter: ");
Serial.println(c);
}
previous_counter=counter;
p_c=c;
delay(1);
}

void loop() {
  if (a[0] == 0)
  {
    rc_vel();
    give_vel();
//    for (int i = 0; i < 5; i++)
//    {
//      Serial.print(" : ");
//      Serial.print(a[i]);
//    }
  }
  //Serial.println();
  //Serial.print(w_l);
  //Serial.print(" : ");
  //Serial.println(w_r);
  MotorR(w_l * 10);
  MotorL(w_r * 10);
  nh.spinOnce();
  //delay(50);
  if(counter != previous_counter || c!=p_c)
{
Serial.print("Right Counter: ");
Serial.print(counter);
Serial.print(" |:| Left Counter: ");
Serial.println(c);
}
previous_counter=counter;
p_c=c;
delay(1);
}
ISR (PCINT0_vect){
cli(); //We pause interrupts happening before we read pin values
currentStateClock = digitalRead(Clock); //Check pin D9 state? Clock
StateData = digitalRead(Data); //Check pin D8 state? Data
if (currentStateClock != lastStateClock && currentStateClock == 1){
// If "clock" state is different "data" state, the encoder is rotating clockwise
if (StateData != currentStateClock){
counter --; // We increment
//lastStateClock = currentStateClock; // Updates the previous state of the clock with the current state
//sei(); //restart interrupts
}
//Else, the encoder is rotating counter-clockwise
else {
counter ++; // We decrement
//lastStateClock = currentStateClock; // Updates previous state of the clock with the current state
//sei(); //restart interrupts
}

}
lastStateClock = currentStateClock;
sei();
}
void updateEncoder(){
// Read the current state of CLK
currentStateCLK = digitalRead(CLK);

// If last and current state of CLK are different, then pulse occurred
// React to only 1 state change to avoid double count
if (currentStateCLK != lastStateCLK && currentStateCLK == 1){

// If the DT state is different than the CLK state then
// the encoder is rotating CCW so decrement
if (digitalRead(DT) != currentStateCLK) {
c--;
//currentDir ="CCW";
} else {
// Encoder is rotating CW so increment
c++;
//currentDir ="CW";
}

//Serial.print("Direction: ");
//Serial.print(currentDir);
//Serial.print(" | Counter: ");
//Serial.println(counter);
}

// Remember last CLK state
lastStateCLK = currentStateCLK;
}
