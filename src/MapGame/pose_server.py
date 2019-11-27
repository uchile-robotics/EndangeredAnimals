#!/usr/bin/env python

import rospy
import rospkg

import os
import time

import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Int32MultiArray

from uchile_srvs.srv import Onoff,PersonDetection, PersonDetectionResponse
from uchile_msgs.msg import Rect
from centernet.src.clothes_detector import Clothes_detector


class PoseDetectorServer:

    def __init__(self):


        self.get_params()
        self.bridge = CvBridge()
        self.rospack = rospkg.RosPack()
        self.model_path=self.rospack.get_path('centernet') + '/src/centernet/models/'+self.weights_name
        rospy.Service('/pose_detector/activate', Onoff, self.activate)
        rospy.Service('/pose_detector/debug', Onoff, self.debug)
        rospy.Service('/pose_detector/detect', PersonDetection, self.process_frame)
        self.save_img = False



    def get_params(self):
        #self.weights_name = rospy.get_param('/pose_server/weights_name')
        self.weights_name = "multi_pose_dla_3x.pth"

    def debug(self,req):
        self.save_img = True
        rospy.logwarn("Images will be save now!")
        return []


    def activate(self,req):

        if (req.select==True):
            if not os.path.exists(self.model_path):
                print("Bad PATH weights")

            rospy.loginfo("Loading model...")
            start=time.time()

            self.detector=Clothes_detector(self.model_path)

            end=time.time()

            rospy.loginfo("Model loaded in " + str(end-start))
            rospy.loginfo('-----------------------------')
            rospy.logwarn("Ready for detection")
            rospy.loginfo('-----------------------------')
        else:
            self.model=None
            torch.cuda.empty_cache()
            rospy.loginfo('-----------------------------')
            rospy.loginfo("GPU FREE")
            rospy.loginfo('-----------------------------')


        return []

    def process_frame(self,req):

    	resp = PersonDetectionResponse()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(req.image, "bgr8")
        except CvBridgeError as e:
            print(e)
            return resp

        rospy.loginfo("Detecting...")

        results = self.detector.detect(cv_image)

        

        if(len(results)>0):

            skt = Int32MultiArray()
            skt.layout.data_offset=len(results[0])-5
            skt.data = []

            
            for result in results:
                bbox_points = result[:4]
                points = result[5:]
                
                #points=map(float,points)

                bbox = Rect()
                bbox.x = min(bbox_points[0],bbox_points[2])
                bbox.y = min(bbox_points[3],bbox_points[1])
                bbox.width = max(bbox_points[0],bbox_points[2])-min(bbox_points[0],bbox_points[2])
                bbox.height = max(bbox_points[3],bbox_points[1])-min(bbox_points[3],bbox_points[1])

                resp.bboxes.append(bbox)
                resp.labels.append("person")
                skt.data.extend(points)

                if self.save_img:
                    p_size = len(points)
                    for j in range(0,p_size/2):
                        x,y = points[2*j], points[2*j+1]

                        if j<5:
                            cv2.circle(cv_image,(x,y), 6, (0,0,255), -1)
                        else:
                            cv2.circle(cv_image,(x,y), 6, (0,255,0), -1)
                #print("type of points: {}".format(type(points[0])))

            #print("type of first skt: {}".format(type(skt.data[0])))
            #print("type of skt: {}".format(type(skt.data)))
            resp.skeletons = skt
            if self.save_img:
                cv2.imwrite("pose_debug.png",cv_image)
        #print(resp)
        return resp

        


if __name__ == "__main__":

    rospy.init_node('pose_detector_server')
    detector = PoseDetectorServer()
    rospy.loginfo("Pose Detector server up!")
    rospy.spin()



