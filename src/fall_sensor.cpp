#include <stdio.h>
#include <time.h>
#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>

#define trig 3 //pin 15
#define echo 6 //pin 16
long travelTime, startTime;
void setup()
{
  wiringPiSetup();
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  digitalWrite(trig, LOW);
  delay(30);
}

int calculate()
{
  digitalWrite(trig, HIGH);
  delay(20);
  digitalWrite(trig, LOW);

  while(digitalRead(echo) == LOW)
    {
      startTime = micros();
    }

  while(digitalRead(echo) == HIGH)
    {
      travelTime = micros() - startTime;
    }

  int distance = travelTime/58;  
  return distance;
}

int main(int argc, char *argv[])
{
  setup();   
  while(1)
  {   
    printf("distance = %d cm \n", calculate());
    delay(500);
  }
  return 0;
}


