import numpy as np
import cv2
import time
import RPi.GPIO as GPIO
import serial

exs=0
exs_g=0
exs_r=0
h_g=0
h_r=0
maxh=0
cen=0
clr="r"
trn=0
prev_trn=0
trn_v=0




p1=23
p2=18
p3=25

GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
GPIO.setup(p1, GPIO.OUT)
GPIO.setup(p2, GPIO.OUT)
GPIO.setup(p3, GPIO.OUT)



prev_frame_time = 0
new_frame_time = 0
fc=0
font = cv2.FONT_HERSHEY_SIMPLEX

a=np.zeros(5)
print (a)

cam = cv2.VideoCapture(0)

cv2.namedWindow("test")

img_counter = 0

while True:
    ret, frame = cam.read()
    if not ret:
        print("failed to grab frame")
        break
    
    new_frame_time = time.time()
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time 
    fps = int(fps)    
    fps = str(fps)
    fc+=1
    fcs=str(fc)
    
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    
    
    
    img=frame[240:380,120:520]  #crop image
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
     
    
#     GREEN color mask evripos robotics
    mask=cv2.inRange(hsv, np.array([35, 80, 80]), np.array([80, 255, 255]));
    
    res_g = cv2.bitwise_and(img, img, mask = mask)
    gray = cv2.cvtColor(res_g, cv2.COLOR_BGR2GRAY)

    contours = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    
    exs=0
    exs_r=0
    exs_g=0
    
    maxh=0
    
    for i in contours: #green
        x,y,w,h = cv2.boundingRect(i)
        if (h>maxh):
            maxh=h
            cen=x+w/2
            clr="g"
            
    
    for i in contours: #green color
        x,y,w,h = cv2.boundingRect(i)
        if (h==maxh):
            cv2.rectangle(img, (x, y), (x + w, y + h), (255,0,0), 2)
            cv2.putText(img, str(cen-200), (x, y+20), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            exs+=1
            exs_g=1
            h_g=h

#     RED color mask evripos robotics    
    mask1=cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]));
    mask2=cv2.inRange(hsv, np.array([170, 100, 50]), np.array([180, 255, 255]));
    mask = mask1 | mask2;
    
    res_r = cv2.bitwise_and(img, img, mask = mask)
    gray = cv2.cvtColor(res_r, cv2.COLOR_BGR2GRAY)

    contours = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    
    
    for i in contours: #red
        x,y,w,h = cv2.boundingRect(i)
        if (h>maxh and h>w and y<50):
            maxh=h
            cen=x+w/2
            clr="r"
                        
    
    for i in contours: #red colors
        x,y,w,h = cv2.boundingRect(i)
        if (h==maxh and h>w and y<50):
            cv2.rectangle(img, (x, y), (x + w, y + h), (255,0,0), 2)
            cv2.putText(img, str(cen-200), (x, y+20), font, 0.5, (0, 0 ,255), 1, cv2.LINE_AA)
            exs+=1
            exs_r=1
            h_r=h

    
    

        
    if(exs>=1):
        GPIO.output(p1, GPIO.HIGH)
        if(maxh>60):
            trn=1
            prev_trn=time.time()
            if(clr=="r"):
                GPIO.output(p2, GPIO.HIGH)                
                trn_v=0
            elif(clr=="g"):
                GPIO.output(p2, GPIO.LOW)                
                trn_v=1
        else:
            if(cen>200):
                GPIO.output(p2, GPIO.HIGH)                
            elif(cen<200):
                GPIO.output(p2, GPIO.LOW)                
            else:
                GPIO.output(p1, GPIO.LOW)           
    else:
        GPIO.output(p1, GPIO.LOW)
    
    if(exs==0 and trn==1 and time.time()-prev_trn>=1.5):
        trn=0
    elif(exs==0 and trn==1 and time.time()-prev_trn<1):
        if(trn_v==0):
            GPIO.output(p1, GPIO.HIGH)
            GPIO.output(p2, GPIO.LOW)
        else:
            GPIO.output(p1, GPIO.HIGH)
            GPIO.output(p2, GPIO.HIGH)           
        
            

    cv2.putText(img, fps, (7, 27), font, 1, (100, 255, 0), 2, cv2.LINE_AA)
    
    up_width = 800
    up_height = 600
    up_points = (up_width, up_height)
    resized_up = cv2.resize(img, up_points, interpolation= cv2.INTER_LINEAR)
    
    cv2.imshow("test", resized_up)

    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        up_width = 800
        up_height = 600
        up_points = (up_width, up_height)
        resized_up = cv2.resize(img, up_points, interpolation= cv2.INTER_LINEAR)

        img_name = "opencv_frame_{}.png".format(img_counter)
        cv2.imwrite(img_name, resized_up)
        print("{} written!".format(img_name))
        img_counter += 1

cam.release()

cv2.destroyAllWindows()

GPIO.cleanup()
print("cleanup ok")




