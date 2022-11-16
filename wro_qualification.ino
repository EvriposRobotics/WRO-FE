#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define INTERRUPT_PIN 2
MPU6050 mpu;
#include <Servo.h>
Servo myservo;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

#define mPin1 8
#define mPin2 11
#define srvPin 9
#define btnPin 12

//long duration;
long distance;
long a0, a1, a2;
int disL = 45;
int parkL = 110, parkH = 160;
int cen = 101;
int rot = 35;
int rotc = 3;
int hsp = 160, lsp = 140, ssp = 255, initsp = 95; //normal hsp=130 lsp=100
int d[8], dl[5], dr[5], maxd;
int mmax = 3;
int disLf = 100, disRt = 100;
long dis = 100;
//float mo;
long prev = 0;
int dir = 1;
int turn = 0;
float ang, nang, diff;
int turnn = 0, trnm;
int turnDir;
int clr = 0;
long cprev, ultraprev = 0, turnprev;
int ultra_delay = 5, queue = 1;
int add_angle = 2;
int protect = 0;
long prot_pr = 0;

void setup() {

  pinMode(btnPin, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Serial.begin(115200);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  Serial.println(devStatus);



  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  myservo.attach(srvPin);

  pinMode(mPin1, OUTPUT);
  pinMode(mPin2, OUTPUT);


  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  myservo.write(cen);
  for (int i = 0; i < 5; i++)
  {
    d[i] = 100;
    dl[i] = 100;
    dr[i] = 100;
  }
  digitalWrite(13, LOW);


  while (digitalRead(btnPin) == HIGH)
  {
    delay(10);
  }
  analogWrite(mPin2, hsp);
  digitalWrite(mPin1, LOW);
  delay(500);
  analogWrite(mPin2, initsp);

  nang = add_angle;
}


//================ LOOP =========================
void loop() {

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Read from the gyroscope/accelerometer sensor
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ang = ypr[0] * 180 / M_PI;
    //    Serial.print("ypr\t");
    //    Serial.println(ang);
  }

  if (millis() - ultraprev > ultra_delay)
  {
    if (queue == 1) //front ultrasonic
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

      Serial.print("dis="); Serial.println(dis);
      queue = 2;
    }
    else if (queue == 2) //read left sharp infrared sensor
    {
      disLf = analogRead(A1); //0.5v -> 60cm  //  2.3v->10cm // 0.4->80cm
      queue = 3;
    }
    else if (queue == 3) //read right sharp infrared sensor
    {
      disRt = analogRead(A2); //0.5v -> 60cm  //  2.3v->10cm // 0.4->80cm
      queue = 1;
    }
    ultraprev = millis();
  }

  
  if ((turn == 0 || turn == 2) && dis > 5  && dis < disL && millis() - turnprev > 2000 && (disLf < 110 || disRt < 110))
  {
    disL = 55;
    if (turnn == 0)
    {
      digitalWrite(mPin1, LOW);
      digitalWrite(mPin2, LOW);
      delay(1000);
    }
    disLf = analogRead(A1); //0.5v -> 60cm  //  2.3v->10cm // 0.4->80cm
    disRt = analogRead(A2); //0.5v -> 60cm  //  2.3v->10cm // 0.4->80cm

    if (turnn == 0 && disLf < disRt)
    {
      turnDir = 1; //turn left
    }
    else if (turnn == 0 && disLf > disRt)
    {
      turnDir = 2; //turn right
    }
    else if (turnn == 0 && disLf == disRt)
    {
      digitalWrite(mPin1, LOW);
      digitalWrite(mPin2, LOW);
    }

    trnm = turnn % 4;
    if (turnDir == 1) //turn left
    {
      if (trnm == 0) nang = 90 + add_angle;
      else if (trnm == 1) nang = 180;
      else if (trnm == 2) nang = -90 + add_angle;
      else if (trnm == 3) nang = add_angle;
      myservo.write(cen - rot);
    }
    else //turn right
    {
      if (trnm == 0) nang = -90;
      else if (trnm == 1) nang = 180;
      else if (trnm == 2) nang = 90 + add_angle;
      else if (trnm == 3) nang = add_angle;
      myservo.write(cen + rot);
    }

    analogWrite(mPin2, hsp);
    prev = millis();
    turn = 1;
    //delay(1000);
  }

  if (trnm == 1) //convert negative angle to a negative one
  {
    if (ang <= -180)
      ang = -ang;
  }
  

  Serial.print("dir= "); Serial.print(turnDir);
  Serial.print("trnm= "); Serial.print(trnm);
  Serial.print("nang= "); Serial.print(nang);
  Serial.print("ang= "); Serial.print(ang);

  diff = abs(nang - ang);
  if (turn == 0 && diff > 2) //automatic direction correction
  {
    turn = 2;
    rotc = diff + 1;
    if (rotc > rot)
      rotc = rot;
    if (nang > ang)
    {
      cprev = millis();
      myservo.write(cen - rotc);
    }
    else
    {
      cprev = millis();
      myservo.write(cen + rotc);
    }
  }

  if (turn == 2 && millis() - cprev > 200)
  {
    myservo.write(cen + rotc);
    turn = 0;
  }

  if (turn == 1 && abs(nang - ang) < 9) //ready to go straight
  {
    myservo.write(cen);
    analogWrite(mPin2, lsp);
    turn = 0;
    turnn += 1; //number of turns counter
    for (int i = 0; i < 5; i++)
    {
      d[i] = 150;
    }
    dis = 150;
    turnprev = millis();
    delay(500);
    if (turnDir == 2 && trnm == 1 && protect == 0)
    {      
      protect = 0;
      prot_pr = millis();      
    }
    if (turnDir == 1 && trnm == 1  && protect == 0)
    {
      protect = 0;
      prot_pr = millis();
    }   
  }

  if (protect == 1 && millis() - prot_pr >= 2000)
  {
    protect = 0;    
    digitalWrite(mPin2, HIGH);
    delay(400);
    analogWrite(mPin2, lsp);
  }

  diff = abs(nang - ang);
  if (turnn == 12 && diff < 2) //after 12 turns enter into parking mode
  {
    analogWrite(mPin2, initsp);
    delay(1500);
    digitalWrite(mPin1, HIGH);
    digitalWrite(mPin2, HIGH);
    prot_pr = millis();
    a0 = analogRead(A0);
    distance = a0 * 520  / 1023;

    while (true)
    {
      if (distance > parkH)
      {
        digitalWrite(mPin1, LOW);
        digitalWrite(mPin2, HIGH);
        delay(100);
        digitalWrite(mPin1, LOW);
        digitalWrite(mPin2, LOW);
        delay(500);
      }
      if (distance < parkL)
      {
        digitalWrite(mPin1, HIGH);
        digitalWrite(mPin2, LOW);
        delay(100);
        digitalWrite(mPin1, LOW);
        digitalWrite(mPin2, LOW);
        delay(500);
      }

      if (distance < parkH && distance > parkL && millis() - prot_pr > 1000)
      {
        digitalWrite(13, HIGH);
        digitalWrite(2, LOW);
        while (true);
      }
      a0 = analogRead(A0);
      distance = a0 * 520  / 1023;

      Serial.print("turn=12  dis= ");
      Serial.println(distance);
    }

  }

  Serial.print("dis= ");
  Serial.println(dis);
}
