import numpy as np
import cv2
import time
import serial
import serial.tools.list_ports

ports = serial.tools.list_ports.comports()

for port, desc, hwid in sorted(ports):
     print (port,desc)
     if(desc=="Arduino Uno"):
         myport=port

ser=serial.Serial(myport,115200,timeout=0.050)

exs=0
maxh=0
cen=0
clr="r"


centers=np.zeros(150)

prev_frame_time = 0
new_frame_time = 0
fc=0
font = cv2.FONT_HERSHEY_SIMPLEX

cam = cv2.VideoCapture(0)


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
    
    img=frame[235:380,80:560]
    
    bnwl=cv2.cvtColor(img[:60,60:110],cv2.COLOR_BGR2GRAY)
    bnwr=cv2.cvtColor(img[:60,430:],cv2.COLOR_BGR2GRAY)
    #change the second parameter value to increase or decrease the black wall cutoff value
    #default value = 55
    retl,thresl=cv2.threshold(bnwl,55,255,cv2.THRESH_BINARY_INV)
    retr,thresr=cv2.threshold(bnwr,55,255,cv2.THRESH_BINARY_INV)
    
        
    #process left part of the image to detect walls ahead
    contours,hierarchy=cv2.findContours(thresl,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    thresl= cv2.cvtColor(thresl, cv2.COLOR_GRAY2BGR)
    thresl= cv2.cvtColor(thresl, cv2.COLOR_BGR2HSV)
    exs_bl_l=0
    for i in contours: 
        x,y,w,h = cv2.boundingRect(i)                
        if(h>50):
            exs_bl_l=1
            cv2.rectangle(thresl, (x, y), (x + w, y + h), (255,0,0), 12)            
            
    
    
    #process right part of the image to detect walls ahead
    contours,hierarchy=cv2.findContours(thresr,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    thresr= cv2.cvtColor(thresr, cv2.COLOR_GRAY2BGR)
    thresr= cv2.cvtColor(thresr, cv2.COLOR_BGR2HSV)
    exs_bl_r=0
    for i in contours: 
        x,y,w,h = cv2.boundingRect(i)                
        if(h>50):
            exs_bl_r=1            
            cv2.rectangle(thresr, (x, y), (x + w, y + h), (255,0,0), 12)   
    
    up_points = (200, 240)
    resl = cv2.resize(thresl, up_points, interpolation= cv2.INTER_LINEAR)
    resr = cv2.resize(thresr, up_points, interpolation= cv2.INTER_LINEAR)    
    cv2.imshow("thresl",resl)
    cv2.imshow("thresr",resr)
  
    cntr=252 
    img=frame[235:380,80:560]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
     
    cv2.line(img,(int(cntr),0), (int(cntr),15), (255,0,0), 3)
#==============================================    
#     GREEN color mask
# green color mask initial values:
#  mask=cv2.inRange(hsv, np.array([35, 80, 80]), np.array([80, 255, 255]));

    mask=cv2.inRange(hsv, np.array([35, 80, 80]), np.array([80, 255, 255]));
    
    res_g = cv2.bitwise_and(img, img, mask = mask)
    gray = cv2.cvtColor(res_g, cv2.COLOR_BGR2GRAY)

    contours = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    
    exs=0
    maxh=0
    
    for i in contours: #find height of tallest green object
        x,y,w,h = cv2.boundingRect(i)
        if (h>maxh):
            maxh=h
            cen=x+w/2
            clr="g"
            nowy=y+h/2
            
    
    for i in contours: #green color
        x,y,w,h = cv2.boundingRect(i)
        if (h==maxh):
            cv2.rectangle(img, (x, y), (x + w, y + h), (255,0,0), 2)
            cv2.putText(img, str(cen-cntr), (x, y+10), font, 0.35, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(img, str(h), (x, y+30), font, 0.35, (0, 255, 0), 1, cv2.LINE_AA)
            if(maxh>0):
                cv2.putText(img, str((cen-cntr)/maxh), (x, y+20), font, 0.35, (0, 255, 0), 1, cv2.LINE_AA)
            exs+=1 
            
#==============================================
#     RED color mask
# green color mask initial values:
# mask1=cv2.inRange(hsv, np.array([0, 130, 80]), np.array([10, 255, 255]));
# mask2=cv2.inRange(hsv, np.array([170, 130, 80]), np.array([180, 255, 255]));

    mask1=cv2.inRange(hsv, np.array([0, 130, 80]), np.array([10, 255, 255]));
    mask2=cv2.inRange(hsv, np.array([170, 130, 80]), np.array([180, 255, 255]));
    mask = mask1 | mask2;
    
    res_r = cv2.bitwise_and(img, img, mask = mask)
    gray = cv2.cvtColor(res_r, cv2.COLOR_BGR2GRAY)

    contours = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    
    for i in contours: #find height of tallest red object
        x,y,w,h = cv2.boundingRect(i)
        if (h>maxh and h>w and y<40):
            maxh=h
            cen=x+w/2
            clr="r"
            nowy=y+h/2  
    
    for i in contours: #red colors
        x,y,w,h = cv2.boundingRect(i)
        if (h==maxh and h>w and y<72):
            cv2.rectangle(img, (x, y), (x + w, y + h), (255,0,0), 2)
            cv2.putText(img, str(cntr-cen), (x, y+10), font, 0.35, (0, 0 ,255), 1, cv2.LINE_AA)
            cv2.putText(img, str(h), (x, y+30), font, 0.35, (0, 0 ,255), 1, cv2.LINE_AA)
            if(maxh>0):
                cv2.putText(img, str((cntr-cen)/maxh), (x, y+20), font, 0.35, (0, 0, 255), 1, cv2.LINE_AA)
            exs+=1            
            
#   ============================          
#   ========== AI ==============
    
    if(exs>=1): #colored sign exist in image
        dist=abs(cen-cntr)        
        if(clr=="g"):
            #print("green")
            dist2=(cen-cntr)
        elif(clr=="r"):
            #print("red")
            dist2=(cntr-cen)
        
        mult=10        
        if(clr=="g"):
            angle=-(1.9-(dist2/maxh))*mult            
        else:
            angle=(1.6-(dist2/maxh))*mult            
            
        max_angle=25    
        angle=int(angle)
        if(angle>max_angle):
            angle=max_angle
        if(angle<-max_angle):
            angle=-max_angle
        
        if(maxh<30):
            if(angle)>13:
                angle=13           
            
        if(exs_bl_r>0 and maxh<55):
            message="l"+str(25)+"s"
            print(message)
            ser.write(message.encode())
        elif(exs_bl_l>0 and maxh<55):
            message="r"+str(25)+"s"
            print(message)
            ser.write(message.encode())
        else:
            if(angle<0):
                message="l"+str(abs(angle))+"s"
                print(message)
                ser.write(message.encode())
            elif(angle>0):
                message="r"+str(angle)+"s"
                print(message)
                ser.write(message.encode())
    else: # no red/green color object detected
        ser.write("ns".encode())
        print("no traffic sign")

    cv2.putText(img, fps, (7, 27), font, 0.6, (100, 255, 0), 1, cv2.LINE_AA)
        
    #resized_up = cv2.resize(img, up_points, interpolation= cv2.INTER_LINEAR)    
    cv2.imshow("wro", img)

    while ser.in_waiting:
        receive=ser.readline().rstrip()
        print(receive)

    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        img_name = "colored_image_{}.png".format(img_counter)
        cv2.imwrite(img_name, img)
        print("{} written!".format(img_name))
        img_name = "left_black_{}.png".format(img_counter)
        cv2.imwrite(img_name, thresl)
        print("{} written!".format(img_name))
        img_name = "right_black_{}.png".format(img_counter)
        cv2.imwrite(img_name, thresr)
        print("{} written!".format(img_name))
        img_counter += 1

cam.release()
cv2.destroyAllWindows()
ser.close()
print("cleanup ok")
