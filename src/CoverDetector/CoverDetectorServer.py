#!/usr/bin/env python
# -*- coding: utf8 -*-
import rospy
import numpy as np
import matplotlib.pyplot as plt 
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters
import image_geometry
import tf

from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Int32MultiArray

from uchile_srvs.srv import DepthDetection, DepthDetectionResponse
from uchile_msgs.msg import Rect



class CoverDetectorServer:

    def __init__(self):
        rospy.loginfo("Detector created with name: ")
        self.bridge = CvBridge()

        rospy.Service('/cover_detector/detect', DepthDetection, self.process_frame)

        """
        self.pub_rgb=rospy.Publisher("Disk",Image,queue_size=20)
        self.pub_depth=rospy.Publisher("Disk_depth",Image,queue_size=20)
        self.pub_crop=rospy.Publisher("Disk_crop",Image,queue_size=20)
        self.image_sub = message_filters.Subscriber("/maqui/camera/front/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/maqui/camera/depth/image_raw", Image)
        self.depth_info = message_filters.Subscriber("/maqui/camera/depth/camera_info", CameraInfo)
        self.image_info= message_filters.Subscriber("/maqui/camera/front/camera_info", CameraInfo)

        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub,self.depth_sub,self.depth_info,self.image_info],30,1)
        #ts = message_filters.TimeSynchronizer([self.image_sub,self.depth_sub,self.depth_info,self.image_info],20)
        ts.registerCallback(self.image_callback)
        self.listener = tf.TransformListener()
        """

    #def process_frame(self,image,depth_image,depth_info,image_info):
    def process_frame(self,req):

        resp= DepthDetectionResponse()

        rospy.loginfo("Getting the cover")
        image=req.rgb_image
        depth_image=req.depth_image
        depth_info=req.depth_camera_info
        image_info=req.rgb_camera_info


        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        try:
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image,"passthrough")
        except CvBridgeError as e:
            print(e)

        Model_depth=image_geometry.PinholeCameraModel()
        Model_depth.fromCameraInfo(depth_info)

        Model_rgb=image_geometry.PinholeCameraModel()
        Model_rgb.fromCameraInfo(image_info)
        self.K=depth_info.K
        self.depth_image=cv_depth_image
        self.image=cv_image
        
        mask = self.depth_image.copy()
        mask.fill(0)
        mask[(self.depth_image<=200)]=8000
        depth_image_2=self.depth_image+mask
        

        Model_depth.rectifyImage(depth_image_2,depth_image_2)

        minimo=np.min(depth_image_2)
        print(minimo)
        cv_depth_image_norm = depth_image_2.copy()
        cv2.normalize(cv_depth_image_norm, cv_depth_image_norm, 0, 255, cv2.NORM_MINMAX)
        data = np.float32(cv_depth_image_norm)
        mask.fill(0)
        mask[(depth_image_2<=minimo+40)]=255
        mask=np.uint8(mask)

        contours, hierachy ,tmp = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        x,y,w,h=cv2.boundingRect(contours)



        x2 = x+w
        y2 = y+h

        
        vector1 = Model_depth.projectPixelTo3dRay((x,y)) #sobre imagen rect
        vector2=[0,0,0]

        vector2[0]=vector1[0]*minimo*0.001 - 0.025
        vector2[1]=vector1[1]*minimo*0.001 + 0.035
        vector2[2]=vector1[2]*minimo*0.001
        
        pixel1 = Model_rgb.project3dToPixel(vector2)

        vector12 = Model_depth.projectPixelTo3dRay((x2,y2)) #sobre imagen rect
        vector22=[0,0,0]

        vector22[0]=vector12[0]*minimo*0.001 - 0.025
        vector22[1]=vector12[1]*minimo*0.001 + 0.035
        vector22[2]=vector12[2]*minimo*0.001
        
        pixel2 = Model_rgb.project3dToPixel(vector22)

        bbox=Rect()
        bbox.x=pixel1[1]
        bbox.y=pixel1[0]
        bbox.width=pixel2[1]-pixel1[1]
        bbox.height=pixel2[0]-pixel1[0]

        resp.bboxes.append(bbox)
        resp.labels.append("Cover")
        
        return resp
        
        
       
        
   # def Process(self):
        
#rospy.Subscriber("/camera/depth/image_raw", Image, image_callback, queue_size = 1, buff_size = 16777216)
#cv2.destroyAllWindows()
  
def main():
    rospy.init_node('disk_detector_server')
    detector = CoverDetectorServer()
    rospy.loginfo("Cover Detector Server up!")
    rospy.spin()


if __name__ == '__main__':
    main()