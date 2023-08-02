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
