#include <Servo.h>
Servo myservo;
#include "Adafruit_APDS9960.h"
Adafruit_APDS9960 apds;
#define srvPin 9
#define btnPin 12
#define mPin1 8
#define mPin2 11

int s = 0;
long prev = 5000;
int first_line = 1;
String first_color = "";
String color = "white";
String rec = "";
int ang;
int in = 0; // stores incoming data from serial port
int dir = 0;
long ultraprev = 0;
long duration = 0;
long a0;
int cen = 101;
int rot = 35;
int hsp = 110, lsp = 105;
long dis;
int turnDir;
long stop_prev = 0;
int stop_flag = 0;
int ultra_delay = 15;
uint16_t r, g, b, c;
int d[8], maxd;
int mmax = 3;

void setup() {
  Serial.begin(115200);
  pinMode(btnPin, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  if (!apds.begin())
  {
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Device initialized!");
  apds.enableColor(true);

  myservo.attach(srvPin);

  pinMode(mPin1, OUTPUT);
  pinMode(mPin2, OUTPUT);


  myservo.write(cen);
  digitalWrite(13, LOW);

  while (digitalRead(btnPin) == HIGH)
  {
    delay(10);
  }

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  analogWrite(mPin2, 0);

  for (int i = 0; i < 8; i++)
  {
    d[i] = 100;
  }
}


//================ LOOP =========================
void loop() {

  if (apds.colorDataReady())
  {
    apds.getColorData(&r, &g, &b, &c);
    if (c < 40)
    {

      if (r > g && r > b && first_line == 1)
      {
        prev = millis();
        s++;
        Serial.print("s= "); Serial.println(s);
        Serial.println("red");
        first_color = "red";
        color = "red";
        first_line = 0;
      }
      else if (b > g && b > r && first_line == 1)
      {
        prev = millis();
        s++;
        Serial.print("s= "); Serial.println(s);
        Serial.println("blue");
        first_color = "blue";
        color = "blue";
        first_line = 0;
      }

      if (r > g && r > b && millis() - prev > 4000 && first_line == 0 && first_color == "red")
      {
        s++;
        if (s == 8 || s == 4)
        {
          duration += millis() - prev;
        }
        prev = millis();
        Serial.print("s= "); Serial.println(s);
        Serial.println("red");
        color = "red";
      }
      else if (b > g && b > r && millis() - prev > 4000 && first_line == 0 && first_color == "blue")
      {
        s++;
        if (s == 8 || s == 4)
        {
          duration += millis() - prev;
        }
        prev = millis();
        Serial.print("s= "); Serial.println(s);
        Serial.println("blue");
        color = "blue";
      }
      else
      {
        color = "white";
      }
    }

  }

  if (millis() - ultraprev > ultra_delay)
  {
    a0 = analogRead(A0);
    dis = a0 * 520  / 1023;

    maxd = 0;
    for (int i = 0; i < mmax; i++)
    {
      d[i] = d[i + 1];
      if (d[i] > maxd) maxd = d[i];
    }
    d[mmax] = dis;
    if (d[mmax] > maxd) maxd = d[mmax];
    dis = maxd;
  }




  if (s == 11) //stop after completing 3 rounds
  {
    stop_flag = 1;
    stop_prev = millis();    
  }

  if ( stop_flag == 1 && millis() - stop_prev > 2500)
  {
    analogWrite(mPin2, 0);
    digitalWrite(2, LOW);
    while (true)
    {
      delay(5);
    }
  }


  if (digitalRead(btnPin) == LOW)
  {
    analogWrite(mPin2, lsp);
  }

  if (Serial.available() > 0)
  {
    // read the incoming byte:
    in = Serial.read();
    if (in != 115)
    {
      rec = rec + (char)in;
    }
    else
    {
     
      if (rec[0] == char(108)) dir = -1;
      else if (rec[0] == char(114)) dir = 1;
      else if (rec[0] == char(110)) dir = 0;
      
      rec.remove(0, 1);
      ang = rec.toInt();
      
      rec = "";
    }
  }
  
  {
    if (dir == -1)
    {
      myservo.write(cen - ang);
    }
    else if (dir == 1)
    {
      myservo.write(cen + ang);
    }
    else if (dir == 0)
    {

      if (first_color == "blue" && millis() - prev < 2000)
      {
        myservo.write(cen - rot);
      }
      else if (first_color == "red"  && millis() - prev < 2000)
      {
        myservo.write(cen + rot);
      }

      else
      {
        if (dis < 40)
        {
          if (first_color == "red")
          {
            myservo.write(cen + rot);
          }
          else if (first_color == "blue")
          {
            myservo.write(cen - rot);
          }
        }
        else
          myservo.write(cen);
      }
    }
  }
  
}
