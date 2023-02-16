# This is the latest version 6/13  19:41



import cv2

import numpy as np
import copy
import time


All_start_time = time.time()

car_cascade = cv2.CascadeClassifier('cars.xml')
crash_delay = 0

#---------- thresholds --------------------------                   |  #config  |
L_th = -1
R_th = 0.5 # is_lane

canny_min_threshold = 180
canny_max_threshold = 200 # Canny edge

minLineLength=40
maxLineGap=5 # houghLineP
#------------------------------------------------




y_end_point = 539
y_twothird_point = 350 # 2/3 in picture




#------------- perspective---------------------
y_perspective_point = 250

L_perspective_point = 0
L_perspective_padding = 20

R_perspective_point = 0
R_perspective_padding = 20
#----------------------------------------------



mask = cv2.imread('lanemask1920_3.png', 0)
mask = cv2.bitwise_not(mask)
mask = cv2.pyrDown(mask) #sub sampling - mask


Error_count = 0


cap = cv2.VideoCapture('hD5T.mp4')
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
down_width = width//2
down_height = height//2


chek = 0


kernel = np.ones((5,5),np.uint8)
kernel_closing = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))


pts2 = np.array([[0,0],[0,400],[70,0],[70,400]], np.float32)


#-------- variables for make line continus --------------
remember_size = 10      # Remember before lane near N frames        |  #config   |

Lx_before_status = 0
Lx_before_x = 0        #y_twothird_point 
Lx_before_x_bottom = 0 #picture's bottom == (y=539)

Rx_before_status = 0
Rx_before_x = 0        #y_twothird_point 
Rx_before_x_bottom = 0 #picture's bottom == (y=539)
#--------------------------------------------------------


before_frame = []



before_velocity = []







while(cap.isOpened()):

    Null_img = np.zeros((down_height,down_width),dtype=np.uint8)
    result =[]

    chek = chek + 1

    is_lane_detect = False
    is_lane_detect_left = False
    is_lane_detect_right = False

    is_velocity_detect = False

    is_crash = False
    is_out = False

    time_start = time.time()
    
    line_img = np.zeros((height,width,3),dtype=np.uint8)
    line_img = cv2.pyrDown(line_img)
    
    

    temp = 0
    temp4 = 0

    

    L_delta = [] # gradient before if
    R_delta = [] # gradient before if

    L_point = [] # [x1,y1]
    R_point = [] # [x1,y1]

    Lx_point_group = [] #[x1,y_end_point=539], [x2,y_end_point=539] ...
    Rx_point_group = [] #[x1,y_end_point=539], [x2,y_end_point=539] ...

    Lx = 0 # (Lx, y_end_point=539) Lx = np.average(Lx_point_group)
    Rx = 0 # (Rx, y_end_point=539) Rx = np.average(Rx_point_group)

    twothird_Lx = 0
    twothird_Rx = 0 # x point that is 2/3 in picture 

    L_count = 0
    R_count = 0

    ret, frame = cap.read()
    frame = cv2.pyrDown(frame) #sub sampling
    img = copy.deepcopy(frame)
    copy1 = copy.deepcopy(frame) # for velocity

    copy1 = cv2.inRange(copy1, (200,200,200), (255,255,255))
    
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    masked_hsv = cv2.inRange(hsv, (20,30,30), (32,255,255)) # for inhance yellow color in HSV
    added = cv2.add(gray, masked_hsv)
    canny_img = cv2.Canny(added, canny_min_threshold, canny_max_threshold)
    bit_img = cv2.bitwise_and(canny_img, mask)



    








    # haar cascade per 2frames
    if chek % 2 == 0:
        cascade_img = copy.deepcopy(gray)
        #cascade_img = cv2.pyrDown(cascade_img) # important now 480x270   1/16 (1/4, 1/4)scale
        cascade_img = cv2.pyrDown(cascade_img) # important now 240x135   1/64 (1/8, 1/8)scale
        cascade_data = car_cascade.detectMultiScale(cascade_img, 1.1, 5)
        for x,y,w,h in cascade_data:
            x = x*2
            y = y*2
            w = w*2
            h = h*2
            h = h * (3/4)
            w = w/2
            #cv2.circle(img,(np.int32(x+w),np.int32(y+h)), 10, (0,255,0), -1)
            if (385<x+w<600) and (250<y+h<400):     #you need resize when downpyr
                is_crash=True
                crash_delay = 10
        
        
        
    
    

        

        

        
            
        
    #cv2.rectangle(img,(385,250), (600, 400), (0,255,255), 3)     # You neeed resize when downpyr
    if crash_delay > 0:
        is_crash=True
        crash_delay = crash_delay-1

    
    
    

    lines = cv2.HoughLinesP(bit_img,1,np.pi/360,100,minLineLength,maxLineGap)


    if lines is not None:
        for i in range(len(lines)):
            for x1,y1,x2,y2 in lines[i]: # x1, y1, x2, y2.dtype=int32
                if (x2-x1)==0:  # if divide 0
                    continue;
                delta = (y2-y1)/(x2-x1)
                if delta < L_th:
                    L_count = L_count+1
                    L_point.append([x1,y1])
                    L_delta.append(delta)
                if delta > R_th:
                    R_count = R_count+1
                    R_point.append([x1,y1])
                    R_delta.append(delta)


        if L_count > 0:
            L_gradient = np.average(L_delta)  # dtype=float64 or (nan: no nan using L_count)
            for x1, y1 in L_point:
                temp = ((y_end_point-y1)/L_gradient) + x1 #temp.dtype=float64->int32
                Lx_point_group.append(temp)
            if Lx_point_group != []:
                Lx = np.int32(np.average(Lx_point_group)) #Lx.dtype=float64->int32
                twothird_Lx = np.int32((y_twothird_point-y_end_point)/L_gradient + Lx) #temp.dtype=float64->int32
                #L_perspective_point = np.int32((y_perspective_point-y_end_point)/L_gradient + Lx) # perspective
                cv2.line(img, (twothird_Lx, y_twothird_point), (Lx, y_end_point), (0,0,255),3)


                Lx_before_status = 10
                Lx_before_x = twothird_Lx
                Lx_before_x_bottom = Lx
                is_lane_detect_left=True

                
        if R_count > 0:
            R_gradient = np.average(R_delta)  # dtype=float64 or (nan: no nan using R_count)
            for x1, y1 in R_point:
                temp = ((y_end_point-y1)/R_gradient) + x1 #temp.dtype=float64->int32
                Rx_point_group.append(temp)
            if Rx_point_group != []:
                Rx = np.int32(np.average(Rx_point_group)) #Rx.dtype=float64->int32
                twothird_Rx = np.int32((y_twothird_point-y_end_point)/R_gradient + Rx) #temp.dtype=float63->int32
                R_perspective_point = np.int32((y_perspective_point-y_end_point)/R_gradient + Rx) # perspective
                cv2.line(img, (twothird_Rx, y_twothird_point), (Rx, y_end_point), (0,0,255),3)
                is_lane_detect_right=True
                    
                
                Rx_before_status = 10
                Rx_before_x = twothird_Rx
                Rx_before_x_bottom = Rx
                '''pts1 = np.float32([[R_perspective_point-30,y_perspective_point-20],
                           [Rx-30,y_end_point-20],
                           [R_perspective_point+30,y_perspective_point-20],
                           [Rx+30,y_end_point-20]])'''

    if L_count == 0 and Lx_before_status > 0:
        cv2.line(img, (Lx_before_x, y_twothird_point), (Lx_before_x_bottom, y_end_point), (255,0,0),3)
        Lx_before_status = Lx_before_status - 1
        is_lane_detect_left=True

    if R_count == 0 and Rx_before_status > 0:
        cv2.line(img, (Rx_before_x, y_twothird_point), (Rx_before_x_bottom, y_end_point), (255,0,0),3)
        Rx_before_status = Rx_before_status - 1
        is_lane_detect_right=True







    if is_lane_detect_left==True and is_lane_detect_right==True:
        is_lane_detect=True



    #velocity--------------------------------------------------------------
    #if chek % 2 == 0 :
    pts1 = np.array([[R_perspective_point-20,y_perspective_point],
                    [Rx_before_x_bottom-105,y_end_point],
                    [R_perspective_point+20,y_perspective_point],
                    [Rx_before_x_bottom+50,y_end_point]], np.float32)


    pts3 = np.array([[R_perspective_point-20,y_perspective_point],
                    [R_perspective_point+15,y_perspective_point],
                     [Rx_before_x_bottom+45,y_end_point],
                     [Rx_before_x_bottom-55,y_end_point]])                      #need resize when downpyr 35-> 17


    cv2.fillPoly(Null_img, [pts3], 255) # must be clock way



    result = cv2.bitwise_and(Null_img, copy1)

    

    if chek % 2 == 1:
        before_frame = copy.deepcopy(result)
        

    if chek>5 and chek % 2 == 0:
        v_count = 0
        v_count_list = []
        M = cv2.getPerspectiveTransform(pts1, pts2)


        result = cv2.warpPerspective(result, M, (70, 400))
        before_frame = cv2.warpPerspective(before_frame, M, (70, 400))


        before_sum = before_frame.sum(axis=1)
        after_sum = result.sum(axis=1)
        col_sum = np.int32(after_sum) - np.int32(before_sum)

        for i in range(len(col_sum)):
            #print(col_sum[i])
            if 3400 > col_sum[i] > 1000:
                v_count = v_count+1# pixel gasu
                
            elif col_sum[i] < 1000 and v_count != 0:
                v_count_list.append(v_count)
        if v_count_list != []:
            velocity = ((v_count_list[0]*0.06)/(1/15)) * 3.6 # 3.6km/h = 1m/s
            is_velocity_detect = True
            if is_lane_detect_right == True:
                cv2.putText(img,('velocity :'+str(round(velocity,2))), (3,62),cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255),2)
                if before_velocity != []:
                    acel = (velocity - before_velocity)
                    if abs(acel) > 9.8:
                        cv2.putText(img, ('acel: WARNING'), (3,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255),2)

            else:
                cv2.putText(img,('velocity :'), (3,62),cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255),2)

            before_velocity=velocity
        cv2.imshow('sub_img',result)
        cv2.imshow('warp', before_frame)

    if chek>5 and chek % 2 == 1 and is_lane_detect_right == True:
        cv2.putText(img,('velocity :'+str(round(before_velocity,2))), (3,62),cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255),2)









    #------- is_out? --------------------------------------
    if is_lane_detect==True and R_count == 0 and L_count == 0: # ?x_before_x_bottoom
        temp4 = (Rx_before_x_bottom + Lx_before_x_bottom)/2
        temp4 = temp4/480
        

    elif is_lane_detect==True and R_count > 0 and L_count == 0: # Rx , Lx_before_x_bottom
        temp4 = (Rx + Lx_before_x_bottom)/2
        temp4 = temp4/480
        


    elif is_lane_detect==True and R_count == 0 and L_count > 0:
        temp4 = (Rx_before_x_bottom + Lx)/2
        temp4 = temp4/480
        


    elif is_lane_detect==True and R_count > 0 and L_count > 0:
        temp4 = (Rx + Lx) / 2
        temp4 = temp4/480    # 480 is bottom middle in pyrDown           you need resize = 480 -> 240


    if temp4 != 0 and (0.7 > temp4 or 1.3 < temp4):
        is_out = True
        

    time_out = time.time()
    
    frame_time = time_out - time_start

    if is_crash == True:
        cv2.putText(img,('CRASH'), (450,62),cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255),2)
    
    if is_out == True:
        cv2.putText(img,('Lane Out'), (450,100),cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255),2)
    
    print('frame: '+str(chek)+'\ttime: ' +str(frame_time)+'\nis_lane_detect: '+str(is_lane_detect)+'\t\tis_crash : '+ str(is_crash)+'\tis_out : '+str(is_out)+ '\tis_velocity_detect :'+str(is_velocity_detect))
    print('\n\n')
    if chek >1799:
        break


    cv2.circle(img,(480, 539),5,(0,255,0),-1) # you need resize -> 240, 269



    #cv2.circle(img,(Rx_before_x-30, y_twothird_point),5,(255,0,255),-1)
    #cv2.circle(img,(Rx_before_x+30, y_twothird_point),5,(255,0,255),-1)


        


    cv2.imshow('asde',img)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break 
        
        
        
All_end_time = time.time()
        


        
total_time = All_end_time - All_start_time

spf = total_time / 1800
        
fps = 1/spf



print('\n\n\n\n\n')
print(Error_count)

print('Total_time : '+str(total_time)+'\tSPF :' + str(spf)+'\tFPS :'+str(fps))



cap.release()
cv2.destroyAllWindows()





