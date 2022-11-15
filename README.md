**Evripos Robotics General Project Description (Greece Team, Future Engineers)**

The robot consists of a chassis that is entirely designed by our team, excluding the rear axle mounted differential for which we used the design by "Luciano Ricardo de Faria", taken from the thingiverse.com repository :
 ([https://www.thingiverse.com/thing:3270976]).
 
The chassis was 3D printed using the "Creality Ender 3 v2" and "Anet A6" printers with the use of PLA type plastic. Some sanding may need after 3D printing on some parts in order to have a perfect fit with others parts or to rotate freely depending on the case.

Not printed parts are:
  •	Τhe wheels rotation axles, which are LEGO axles. On the front wheels, 2 LEGO axles with a length of 39mm have been used using a clamp (LEGO bush) on each wheel. For the    rear wheels we used 2 LEGO axles of 72mm length. A small amount of silicone-teflon type lubricating grease has been placed at each friction-contact point of the axles      with other components.
  •	One rubber O-ring on each wheel, outer diameter 68mm and thickness 5mm. It is critical that the o-rings are of a good quality in order to have a good grip with the 
   track.
   
**Robot movement**

For the drive of the robot, a "tt dc gear motor" type motor was used operating at 3-6V voltage with maximum rotation speed of 130 rpm. The motor is powered by a battery pack of 4 AA NiMH batteries connected in series.

Speed control of the robot is performed by an L298n dc motor controller which receives signal through the digital ports 8 & 11 of the Arduino. Port 11 provides a PWM type signal for speed control. For better control of the surrounding environment through the robot's sensors, the motor speed has not been set to the motor’s maximum speed, as the PWM high-low pulse ratio does not exceed 50% (50% PWM).

The steering control is done by a micro servo type motor (MG90s) with metal gears. The Arduino controls the rotation of the servomotor via digital port 9. The servomotor rotates at most of 35⁰ left or right relative to the straight line of movement resulting to the turn of the front wheels to the relevant angle/direction.

**Sensors**

Three (3) distance sensors, a color sensor, an accelerometer-gyroscope(IMU) and a wide camera are used for the needs of the robot's navigation inside the track:

  •	The front distance sensor: "DFRobot URM09" type ultrasonic distance sensor with power supply tolerance 3.3-5V, detection range 2-500cm and resolution of 1cm. It is   
    powered by the Arduino Uno's 5V output.
  •	The left and right distance sensors: "Sharp GP2Y0A21YK" type infrared distance sensors with 4.5-5.5V power supply tolerance and 10-80cm distance detection range.
  •	The color sensor: "APDS9960" type sensor with voltage supply of 3.3V which communicates with the Arduino using the I2C protocol.
  •	Accelerometer-gyroscope sensor: “MPU6500” IMU type sensor with 3V-5V operating voltage which communicates with the Arduino using the I2C protocol.
  •	Camera: “Waveshare Raspberry Pi camera module fisheye lens(m)” camera type sensor with 5MP resolution and 200° viewing angle.

(The two sensors that communicate using the I2C protocol are connected to the Arduino through the analog input ports A4 & A5 which have the role of the SDA and SCL ports of the protocol respectively).

**In details**

The front distance sensor (ultrasonic) is used to detect the walls of the track ahead, while the side ones are used to detect the direction το which the vehicle should turn at each corner.

When the robot gets close to a wall then the Arduino compares the 2 inputs it receives from the 2 distance sensors (left/right), compares the two distance inputs and then turns towards the direction of the sensor providing the higher distance (left/right).

Through the built-in motion processor (DMP-Digital Motion Processor) contained in the “MPU6500” sensor, which combines data from the accelerometer and the gyroscope, we continuously identify the orientation of the robot (in degrees) versus its initial position on the track. The sensor orientation error is less than 1⁰. When initializing the vehicle position the orientation is set to 0⁰. When the vehicle turns left the degrees increase up to 180⁰. When the robot turns right the degrees are reduced up to -180°. It also utilizes an algorithm which constantly corrects the path of the robot according to the continuously calculated desired path.

The color sensor “APDS9960” detects the blue and orange lines of the track in order to accurately measure the number of turns the robot has performed but also to detect the direction to which the robot should turn to. If, for example, the robot meets the blue line first, then it understands that it should follow a counterclockwise trajectory on the track and thus must turn left on every at each subsequent corner.

The detection of the green & red signals on the track is done through the raspberry pi model b+ which receives images through a 200⁰ fisheye camera with 5MP. At first the image is cropped, discarding the upper part of it as this is not needed for the obstacle detection and could create deviations if it detects off-track objects (keep only the useful track related image). In like manner, the left and right edges of the image is cropped reducing by a small degree the viewing angle of 200o that is not needed in our case. The raspberry pi processes the images by first cropping part of the field of view and then applying (via the opencv computer vision library) an algorithm that recognizes the red and green signals (obstacles) on the track. The walls of the track, which are black in color, are also detected for the robot to avoid them. The color system used to identify colors is HSV (Hue-Saturation-Value).

Finally, these data are processed appropriately (check how many obstacles exist, which obstacle is the closest so that it is avoided immediately) and through the serial port (USB cable) connecting the raspberry pi to the Arduino, the data are sent in the form of string messages to the Arduino. The Arduino decodes these messages and then turns accordingly to avoid obstacles, taking also into account the inputs from the other sensors it manages. One message for example is “ns” which means no obstacle is detected ahead and another message is “l12s” which means turn the wheels to the left by 12⁰.

When there are no obstacles, the Arduino moves considering only the data from the other sensors (except the camera). 

**Light source**

We use a "cool white" color Light Emmitting Diode (LED), mounted at the bottom of the robot, next to the APDS9960 sensor. The purpose of this LED is to provide a strong light for the color sensor to make precise measurements of the track bottom colors. The led is connected to the power source using a 330 Ohm 1/4 Watt resistor in series as a protection. 


**Electric power sources**

Two power sources are used:
1. A battery pack of 4 AA NiMH batteries in series (capacity 2450mAh) producing a total output voltage of 4.8V and is used for:
  •	the dc motor controller which powers the robot dc motor 
  •	the servo motor that helps the robot turn right or left
  •	the ultrasonic sensor
  •	the two (2) infrared sensors
  •	the white led
  
For more efficient energy consumption management of the electronic components powered by the above battery array, a relay is used. This relay receives a logic level (5V) activation signal from the digital output port 2 of the Arduino Uno as soon as it starts, thus powering all connected devices to the battery pack. When the robot completes the activity on the track, the Arduino Uno is turned off by the user disconnecting its power cable, so that the relay stops powering these devices.

(Caution: Alkaline batteries should not be used for the array as the total voltage they produce (6V) is greater than the maximum allowable operating voltage of some electrical elements powered by the battery array).

2. A Χiaomi Powerbank (Mi powerbank 3 – 18W) with a total capacity of 10000mAh which exclusively powers the raspberry pi with an operating voltage of 5V via a USB type C cable. 

Notes about the power supply:
  •	The camera connected to the raspberry pi is powered through the CSI type cable connected to the corresponding port of the raspberry pi.
  •	The Arduino is powered through the raspberry pi's USB port via a Male-A to Male-B USB cable.
  •	The MPU6500 and APDS9960 sensors are powered by the 3.3V output of the Arduino Uno.

**SBC and SCM** 

The Single Board Computer used in our setup is the Raspberry Pi 3 model B+. The SBC in our case collects and processes the data (images) from the wide angle camera, at a rate of about 15-18 frames per second. Through this setup our robot is able to detect whether it has a red or a green sign ahead or a black wall, deciding this way the angle the wheels must turn to avoid them the right way. The Raspberry Pi runs the official Raspberry Pi OS installed on a 32GB microsd card.

The Single Board Microcontroller used is the Arduino Uno rev2. The SBM receives data from various sensors and from the SBC via a serial (USB) cable. The SBM makes the final decisions for the robots movement (when to start, when to stop and when to turn) as it gives priority to the appropriate sensor according to the circumstances.

**Programming languages**

The Arduino Uno rev2  SBM used in our robot was programmed using the Arduino IDE 1.8.19 integrated programming environment with C++ language. The libraries used for Arduino are:
  •	I2Cdev.h
  •	MPU6050_6Axis_MotionApps20.h
  •	Wire.h
  •	Servo.h
  •	Adafruit_APDS9960.h

Raspberry pi 3 SBC programming was done through Thonny integrated programming environment over Raspberry OS operating system. The programming language used was Python version 3.9.2, using extensively the opencv computer vision library. Other Python used:
  •	numpy
  •	cv2 (opencv)
  •	time
  •	serial
