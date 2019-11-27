#!/usr/bin/env python

import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2

from sensor_msgs.msg import Image
from uchile_srvs.srv import PersonDetection, PersonDetectionRequest
import numpy as np

# HEAD:  0 -> nose | 1 -> l_eye | 2 -> r_eye | 3 -> l_ear  | 4 -> r_ear
# UPPER BODY: 5 -> l_shoulder | 6 -> r_shoulder | 7 -> l_elbow | 8 -> r_elbow | 9 -> l_wrist | 10 -> r_wrist
# LOW BODY: 11 -> l_hip | 12 -> r_hip

class PoseServerClient:

    def __init__(self):
        self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        self.rgb_image = None
        self.img_pub = rospy.Publisher('/skeleton_image', Image, queue_size=1)
        self.bridge = CvBridge()
        self.Font= cv2.FONT_HERSHEY_SIMPLEX
        self.FontColor=(0, 0, 255) 
        now = rospy.get_rostime()
        rospy.loginfo("Initial time %i %i", now.secs, now.nsecs)
        self.InitialTime=now.secs
        self.last_zone=0

    def img_callback(self,img_data):
        self.rgb_image = img_data

    def draw_separation_lines(self,img_data):
        width=img_data.shape[1]
        heigth=img_data.shape[0]




        p11=(width/3,heigth)
        p12=((width/3),0)
        p21=((width/3)*2,heigth)
        p22=((width/3)*2,0)

        rospy.loginfo("p11: {} p12: {}".format(p11,p12))

        cv2.line(img_data, p11, p12, (0,0,255), 2)

        cv2.line(img_data, p21, p22, (0,0,255), 2) 

        return img_data

    def reset_time_by_zone(self,img_data,y):
        actual_zone=self.get_zone(img_data,y)
        rospy.loginfo("actual zone:{}".format(actual_zone))
        rospy.loginfo("last zone:{}".format(self.last_zone))
        if self.last_zone!=actual_zone:
            rospy.loginfo("Actualice el tiempo inicial")
            self.InitialTime=rospy.get_rostime().secs
        self.last_zone=actual_zone
        return

    def draw_skeleton(self,img_data,data):
        
        #x,y = detections.skeletons.data[2*j], detections.skeletons.data[2*j+1]

        # Left Arm
        for i in range(5,9):
            index=[i,i+2]
            img_data=self.draw_lines(img_data,data,index)

        img_data=self.draw_lines(img_data,data,[11,12])
        img_data=self.draw_lines(img_data,data,[5,11])
        img_data=self.draw_lines(img_data,data,[6,12])
        img_data=self.draw_lines(img_data,data,[5,6])

        return  img_data

    def draw_lines(self,img_data,data,indexs):
        color=(0,0,0)
        img_data=cv2.line(img_data, (data[2*indexs[0]],data[2*indexs[0]+1]), (data[2*indexs[1]],data[2*indexs[1]+1]), color, 2)
        return img_data


    def get_zone(self,img_data,y):
        width=img_data.shape[1]

        if(y<width/3):
            return 0
        elif (y<(width/3)*2):
            return 1
        else:
            return 2


    def create_image(self,img_data,image_path='/home/nmarticorena/Downloads/chiluwu(1).jpg'):
        src1 = cv2.imread(image_path)

        width=img_data.shape[1]
        heigth=img_data.shape[0]
        blank_image = np.zeros((heigth,width,3), np.uint8)
        blank_image[:,0:width/3] = (255,255,0)      # (B, G, R)
        blank_image[:,width/3:(width/3)*2] = (200,255,0)
        blank_image[:,(width/3)*2:(width)] = (155,255,0)
        blank_image = cv2.addWeighted(src1, 1, blank_image, 1, 0.0)
        return blank_image


    def call_service(self):
        print("asdasda")
        if self.rgb_image is None:
            return

        rospy.wait_for_service('/pose_detector/detect')
        try:
            server_client = rospy.ServiceProxy('/pose_detector/detect', PersonDetection)
            request = PersonDetectionRequest()
            request.image = self.rgb_image
            detections = server_client(request)
            print(detections)
            
            print(str(rospy.Time.now()))
            ############################################
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.rgb_image, "bgr8")
            except CvBridgeError as e:
                print(e)

            rospy.loginfo(cv_image.shape)

            cv_image=self.create_image(cv_image)

            cv_image=self.draw_separation_lines(cv_image)
            cv_image=self.draw_skeleton(cv_image,detections.skeletons.data)

            if len(detections.labels)>0:
                rospy.loginfo("Encontre una weaita")
                p_size = detections.skeletons.layout.data_offset
            
                for j in range(0,p_size/2):
                    x,y = detections.skeletons.data[2*j], detections.skeletons.data[2*j+1]

                    if j<5:
                        if j==0:
                            x1,y1,x2,y2=detections.skeletons.data[2*1], detections.skeletons.data[2*1+1],detections.skeletons.data[2*2], detections.skeletons.data[2*2+1]
                            radius=np.sqrt((x2-x1)**2+(y2-y1)**2)
                            cv2.circle(cv_image,(x,y), int(radius*2) , (0,0,0), 2)
                        elif j<=2:
                            cv2.circle(cv_image,(x,y), 12, (0,255,0), -1)
                    elif j==10:
                        self.reset_time_by_zone(cv_image,x)

                        time=rospy.get_rostime().secs-self.InitialTime
                        cv2.putText(cv_image,str(time) , (x,y), self.Font, 1,  
             self.FontColor, 1, cv2.LINE_AA, False)
                        cv2.circle(cv_image,(x,y), 6, (0,255,0), -1)


                    else:
                        cv2.circle(cv_image,(x,y), 6, (0,255,0), -1)

            
            try:
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            except CvBridgeError as e:
                print(e)


            self.img_pub.publish(ros_image)
            ############################################
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e







if __name__ == "__main__":

    rospy.init_node('pose_detector_client')
    server_client = PoseServerClient()

    
    while not rospy.is_shutdown():

        server_client.call_service()
        rospy.sleep(0.2)