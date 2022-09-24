// *****  EVRIPOS ROBOTICS TEAM *****

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define INTERRUPT_PIN 2

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



MPU6050 mpu;
#include <Servo.h>
Servo myservo;

int mPin1 = 8;
int mPin2 = 11;
long duration; 
int distance; 
#define ePinFr 2 
#define tPinFr 3
#define ePinLf 5 
#define tPinLf 4
#define ePinRt 7 
#define tPinRt 6
#define srvPin 9

int a0,a1,a2;
 
int cen=101;
int rot=35;
int rotc=3;
int hsp=200,lsp=180,ssp=255;
int d[5],dl[5],dr[5],maxd;
int mmax=3;
int dis=1000,disLf=100,disRt=100;
float mo;
long prev=0;
int dir=1;
int turn=0;
float ang,nang,diff;
int turnn=0,trnm;
int turnDir;
int clr=0;
long cprev,ultraprev=0,turnprev;
int ultra_delay=15,queue=1;
int add_angle=2;

void setup() {
   pinMode(13,OUTPUT);
   digitalWrite(13,HIGH);
//  lcd.begin();
//  lcd.backlight();
//  lcd.clear();
//  lcd.setCursor(1,0);
//  lcd.print("Calibrating..."); 
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
  
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));


    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    Serial.println(devStatus);
    

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
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
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }  
   
  myservo.attach(srvPin);
  
  pinMode(mPin1, OUTPUT);
  pinMode(mPin2, OUTPUT);
  
  pinMode(tPinFr, OUTPUT); 
  pinMode(ePinFr, INPUT);
  pinMode(tPinLf, OUTPUT); 
  pinMode(ePinLf, INPUT); 
  pinMode(tPinRt, OUTPUT); 
  pinMode(ePinRt, INPUT); 
   
  analogWrite(mPin2, ssp);
  digitalWrite(mPin1, LOW);
  
  myservo.write(cen);
  for(int i=0;i<5;i++)
  {
    d[i]=100;
    dl[i]=100;
    dr[i]=100;
  }

  delay(500);
  analogWrite(mPin2, lsp);
  digitalWrite(13,LOW); 
  nang=add_angle; 
 }
 

//================ LOOP =========================
void loop() {

  a0=analogRead(A0);
  //a1=analogRead(A1);
  a2=analogRead(A2);
  analogWrite(mPin2, lsp);
    
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Read from the gyroscope/accelerometer sensor
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ang=ypr[0] * 180/M_PI;
  }
  
  if(turnn==0 && millis()-ultraprev > ultra_delay)
  {
    if(queue==1) //front ultrasonic
    {
      digitalWrite(tPinFr, LOW);
      delayMicroseconds(2);
      digitalWrite(tPinFr, HIGH);
      delayMicroseconds(10);
      digitalWrite(tPinFr, LOW);
      duration = pulseIn(ePinFr, HIGH, 15000);  
      distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
      dis=distance;
      
      maxd=0;
      for(int i=0;i<mmax;i++)
      {
        d[i]=d[i+1];
        if(d[i]>maxd) maxd=d[i];
      } 
      d[mmax]=dis;
      if(d[mmax]>maxd) maxd=d[mmax];      
      dis=maxd;
      
      queue=2;
//      queue=1;
    }
    else if (queue==2) //left ultrasonic
    {
      digitalWrite(tPinLf, LOW);
      delayMicroseconds(2);
      digitalWrite(tPinLf, HIGH);
      delayMicroseconds(10);
      digitalWrite(tPinLf, LOW);
      duration = pulseIn(ePinLf, HIGH, 15000);  
      distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)      
      disLf=distance;
      
      maxd=0;
      for(int i=0;i<mmax;i++)
      {
        dl[i]=dl[i+1];
        if(dl[i]>maxd) maxd=dl[i];
      } 
      dl[mmax]=disLf;
      if(dl[mmax]>maxd) maxd=dl[mmax];      
      disLf=maxd; 
          
      queue=3;
    }
    else if (queue==3) //right ultrasonic
    {
      digitalWrite(tPinRt, LOW);
      delayMicroseconds(2);
      digitalWrite(tPinRt, HIGH);
      delayMicroseconds(10);
      digitalWrite(tPinRt, LOW);
      duration = pulseIn(ePinRt, HIGH, 15000);  
      distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
      disRt=distance;  

      maxd=0;
      for(int i=0;i<mmax;i++)
      {
        dr[i]=dr[i+1];
        if(dr[i]>maxd) maxd=dr[i];
      } 
      dr[mmax]=disRt;
      if(dr[mmax]>maxd) maxd=dr[mmax];      
      disRt=maxd;
       
      queue=1; 
    }
    ultraprev=millis();
  }
  else if(turnn>=1 && millis()-ultraprev > ultra_delay)
  {
    digitalWrite(tPinFr, LOW);
    delayMicroseconds(2);
    digitalWrite(tPinFr, HIGH);
    delayMicroseconds(10);
    digitalWrite(tPinFr, LOW);
    duration = pulseIn(ePinFr, HIGH, 15000);  
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    if(distance>50) dis=distance;
    if(distance<50 && millis()-turnprev>2500)
       dis=distance;
    else
       dis=100;    
    
    maxd=0;
    for(int i=0;i<mmax;i++)
    {
      d[i]=d[i+1];
      if(d[i]>maxd) maxd=d[i];
    } 
    d[mmax]=dis;
    if(d[mmax]>maxd) maxd=d[mmax];      
    dis=maxd;
    ultraprev=millis();
  }

  if(a0>300 && turn!=1) 
  {
    turn=3;
    if(a2>300)
    {
      myservo.write(cen+rot);      
    }
    else
    {
      myservo.write(cen-rot);        
    }
  }

  if(turn==3)
  {
    if(a0<300)
    {
      turn=4;
    }
  }

  if(turn==4 && diff>2)
   {    
    rotc=diff+1;
    if(rotc>rot)
       rotc=rot;
    if(nang>ang)
    {      
      cprev=millis();
      myservo.write(cen-rotc);
    }
    else
    {      
      cprev=millis();
      myservo.write(cen+rotc);      
    }
   }
   else if (turn==4 && diff<=2)
   {
    turn=0;
   }
  
  if((turn==0 || turn==2) && dis>20 && dis<50 && millis()-turnprev>2000)
  {
        
      if(turnn==0 && disLf>disRt)
      {
         turnDir=1; //turn left         
      }
      else if (turnn==0 && disLf<disRt)
      {
        turnDir=2; //turn right        
      }
      else if (turnn==0 && disLf==disRt)
      {
        digitalWrite(mPin1, HIGH);
        digitalWrite(mPin2, HIGH); 
      }
        
      trnm=turnn % 4;
      if(turnDir==1) //turn left
      {
        if(trnm==0) nang=90+add_angle;
        else if (trnm==1) nang=180;
        else if (trnm==2) nang=-90+add_angle;
        else if (trnm==3) nang=add_angle;
        myservo.write(cen-rot);
      }
      else //turn right
      {
        if(trnm==0) nang=-90;
        else if (trnm==1) nang=180;
        else if (trnm==2) nang=90+add_angle;
        else if (trnm==3) nang=add_angle;
        myservo.write(cen+rot);
      }     
        
      analogWrite(mPin2, hsp);
      prev=millis();
      turn=1;    
      //delay(1000);          
  }
  
   if(trnm==1) //an vrisketai se gwnia 180(-180) metatrepei thn arnhtikh se 8etikh
   {
    if(ang<=-180)
    ang=-ang;
   }


   Serial.print("dir= "); Serial.print(turnDir);
   Serial.print("trnm= "); Serial.print(trnm);
   Serial.print("nang= "); Serial.print(nang);
   Serial.print("ang= "); Serial.print(ang); 
   diff=abs(nang-ang);
   if(turn==0 && diff>2)
   {
    turn=2;
    rotc=diff+1;
    if(rotc>rot)
       rotc=rot;
    if(nang>ang)
    {      
      cprev=millis();
      myservo.write(cen-rotc);
    }
    else
    {      
      cprev=millis();
      myservo.write(cen+rotc);      
    }
   }

   if(turn==2 && millis()-cprev>200)
   {
      myservo.write(cen+rotc);
      turn=0;
   }
   
   if(turn==1 && abs(nang-ang)<9) //an vrisketai se strofh kai einai etoimo na isiwsei
    {
      myservo.write(cen);
      analogWrite(mPin2, lsp);
      turn=0;
      turnn+=1; //otan teleiwnei h strofh auksanetai kata 1
      for(int i=0;i<5;i++)
      {
        d[i]=150;  
      }
      dis=150;
      turnprev=millis();
      delay(1000);
      if(turnDir==2 && trnm==1)
      {
        digitalWrite(mPin1, LOW);
        digitalWrite(mPin2, LOW);
        delay(3000);
        digitalWrite(mPin2, HIGH);
        delay(300);
        analogWrite(mPin2, lsp);
      }
      if(turnDir==1 && trnm==1)
      {
        digitalWrite(mPin1, LOW);
        digitalWrite(mPin2, LOW);
        delay(3000);
        digitalWrite(mPin2, HIGH);
        delay(300);
        analogWrite(mPin2, lsp);
      }
    }
    
    if(turnn==12) //an exei kanei kai thn 12h strofh stamataei
    {
      digitalWrite(13,HIGH);
      delay(400);
      digitalWrite(mPin1, HIGH);
      digitalWrite(mPin2, HIGH);       
      while(true);
    }

  Serial.print("dis= ");
  Serial.println(dis);

}
