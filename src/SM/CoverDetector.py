#!/usr/bin/env python
import rospy
import smach
from smach_ros import IntrospectionServer
import numpy as np
import time


from maqui_skills import robot_factory

from uchile_states.interaction.states import Speak
from uchile_states.interaction.tablet_states import ShowWebpage, WaitTouchScreen


from uchile_srvs.srv import DepthDetection, DepthDetectionRequest
from uchile_srvs.srv import PersonDetection, PersonDetectionRequest
from uchile_msgs.msg import SkeletonWeb

from sensor_msgs.msg import Image,CameraInfo

from cv_bridge import CvBridge, CvBridgeError
import cv2


class Setup(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded","aborted"]) # Aca se ponen todas las salidas que pueden tener este estado
		self.robot=robot
		self.audition=self.robot.get("audition")
		self.tts=self.robot.get("tts")
		#self.skill=self.robot.get("skill")

	def execute(self,userdata): # Userdata es informacion que se puede mover entre estados
		self.tts.set_language("Spanish")
		#self.tts.say("Setup detector de CARATULAS")
		self.tts.say("Me gustaria guardar una imagen del animal para la ficha")
		self.robot.tts.wait_until_done()
		rospy.sleep(10)


		#return "preemted" 
		#OJO esto genera un error debido a que en los outcomes definidos de las clase no se tiene el preemted , recomiendo ejecutar para ver el error
		return "succeeded"

class Selector(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded","aborted"],input_keys=['animal_info'],output_keys=['animal_info']) # Aca se ponen todas las salidas que pueden tener este estado
		self.robot=robot
		self.audition=self.robot.get("audition")
		self.tts=self.robot.get("tts")
		#self.skill=self.robot.get("skill")
		self.img_pub = rospy.Publisher('/skeleton_image', Image, queue_size=1)
		self.skeleton_pub = rospy.Publisher('/SkeletonWeb', SkeletonWeb, queue_size=1)
		self.bridge = CvBridge()
		self.Font= cv2.FONT_HERSHEY_SIMPLEX
		self.FontColor=(0, 0, 0) 
		now = rospy.get_rostime()
		rospy.loginfo("Initial time %i %i", now.secs, now.nsecs)
		self.InitialTime=now.secs
		self.last_zone=0

		self.ZonesNames={0:'Primera imagen',1:'Segunda imagen',2:'Tercera Imagen',3:'Cuarta Imagen'}
		#self.skill=self.robot.get("skill")
	
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

	def reset_time_by_zone(self,img_data,x,y):
		actual_zone=self.get_zone(img_data,x,y)
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

	def get_zone(self,img_data,x,y):
		width=640
		heigth=480

		if(x<width/2 and y<heigth/2):
			return 0
		if(x<width/2 and y>=heigth/2):
			return 1
		if(x>width/2 and y<heigth/2):
			return 2
		else:
			return 3

	def draw_lines(self,img_data,data,indexs):
		color=(0,0,0)
		img_data=cv2.line(img_data, (data[2*indexs[0]],data[2*indexs[0]+1]), (data[2*indexs[1]],data[2*indexs[1]+1]), color, 2)
		return img_data

	def create_image(self,image_path='/home/nmarticorena/Downloads/chiluwu(1).jpg'):
		src1 = cv2.imread(image_path)

		width=640
		heigth=480
		blank_image = np.zeros((heigth,width,3), np.uint8)
		blank_image[:,0:width/3] = (255,255,0)      # (B, G, R)
		blank_image[:,width/3:(width/3)*2] = (200,255,0)
		blank_image[:,(width/3)*2:(width)] = (155,255,0)
		blank_image = cv2.addWeighted(src1, 1, blank_image, 1, 0.0)
		return blank_image


	def remap(self,x,in_min,in_max,out_min,out_max):
		return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

	def execute(self,userdata): # Userdata es informacion que se puede mover entre estados

		server_client = rospy.ServiceProxy('/pose_detector/detect', PersonDetection)
		request = PersonDetectionRequest()

		self.tts.set_language("Spanish")
		#self.tts.say("Setup detector de CARATULAS")
		self.tts.say("Ahora deben seleccionar con cual de las fotos quedarse")
		self.robot.tts.wait_until_done()
		self.tts.say("Porfavor mueve tu mano derecha para seleccionar la foto donde el animal se vea mejor")
		#self.tts.wait_until_done()

		request.image = rospy.wait_for_message("/usb_cam/image_raw", Image)
		#request.image = rospy.wait_for_message("/maqui/camera/front/image_raw", Image)
		detections = server_client(request)

		#print(detections)
		time_2=0
		self.InitialTime=rospy.get_rostime().secs

		skeleton=SkeletonWeb()

		while time_2<5:
			
			request.image = rospy.wait_for_message("/usb_cam/image_raw", Image)
			#request.image = rospy.wait_for_message("/maqui/camera/front/image_raw", Image)
			detections = server_client(request)
			if len(detections.labels)>0:
				cv_image=self.create_image()

				cv_image=self.draw_separation_lines(cv_image)
				cv_image=self.draw_skeleton(cv_image,detections.skeletons.data)
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
					elif j==9:
						self.reset_time_by_zone(cv_image,x,y)

						time_2=rospy.get_rostime().secs-self.InitialTime
						cv2.putText(cv_image,str(time_2) , (x,y), self.Font, 1,  
			 self.FontColor, 1, cv2.LINE_AA, False)
						cv2.circle(cv_image,(x,y), 10, (0,0,255), -1)
						x_web=self.remap(x,0,640,0,1280)
						y_web=self.remap(y,0,480,0,800)
						skeleton.Hand=[x_web,y_web]
						skeleton.Zone=self.last_zone
						skeleton.Time=int(time_2)*2
						rospy.loginfo(skeleton)
						self.skeleton_pub.publish(skeleton)


					elif j<11:
						cv2.circle(cv_image,(x,y), 6, (0,255,0), -1)
			try:
				ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
			except CvBridgeError as e:
				print(e)
			time_2=rospy.get_rostime().secs-self.InitialTime

			#rospy.loginfo(skeleton)
			self.img_pub.publish(ros_image)

		self.tts.say("seleccionada {}".format(self.ZonesNames[self.last_zone]))
		rospy.loginfo("../static/img/tmp/{}.jpg".format(self.last_zone))



		try:
			userdata.animal_info["Image"]="../static/tmp/{}.jpg".format(self.last_zone)
		except:
			pass
		#self.tts.wait_until_done()
		#return "preemted" 
		#OJO esto genera un error debido a que en los outcomes definidos de las clase no se tiene el preemted , recomiendo ejecutar para ver el error
		return "succeeded"


class CoverDetector(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded","preempted"],input_keys=['kid_name'])
		self.robot=robot
		self.tts=self.robot.get("tts")
		self.tablet=self.robot.get("tablet")
		self.it=0
		self.max=3

		self.bridge = CvBridge()


		self.img_pub = rospy.Publisher('/cover', Image, queue_size=1)
		#self.cover_detector=self.robot.get("cover_detector") # POR IMPLEMENTAR
	def execute(self,userdata):
		#rospy.loginfo(userdata.kid_name)
		server_client = rospy.ServiceProxy('/cover_detector/detect', DepthDetection)
		request = DepthDetectionRequest()
		request.rgb_image = rospy.wait_for_message("/maqui/camera/front/image_raw", Image)
		request.depth_image = rospy.wait_for_message("/maqui/camera/depth/image_raw", Image)
		request.rgb_camera_info = rospy.wait_for_message("/maqui/camera/front/camera_info", CameraInfo)
		request.depth_camera_info = rospy.wait_for_message("/maqui/camera/depth/camera_info", CameraInfo)
		detections = server_client(request)
		rospy.loginfo(detections)

		try:
			cv_image = self.bridge.imgmsg_to_cv2(request.rgb_image, "bgr8")
		except CvBridgeError as e:
			print(e)

		x,y,w,h=detections.bboxes[0].x,detections.bboxes[0].y,detections.bboxes[0].width,detections.bboxes[0].height

		crop_image=cv_image[x:x+w,y:y+h]

		
		cv2.imwrite("../animal_tornado/static/tmp/{}.jpg".format(self.it),crop_image)
		rospy.sleep(1)
		#try:
			#cv2.imwrite("sftp://nao@192.168.1.126/data/home/nao/animal_tornado/static/tmp/{}.jpg".format(self.it),crop_image)
		#except:
		#	pass

		if self.it<self.max:
			self.it+=1
			return "preempted"

		#Crear skill para el detector de caratulas
		# path = path a la imagen
		# self.cover_detector.save_cover(path)
		# self.tablet.show_image(path)
		#Speak(self.robot,"FALTA POR IMPLEMENTAR MAQUINA DE ESTADOS DEL DETECTOR DE CARATULAS")
		#self.tts.say("FALTA POR IMPLEMENTAR MAQUINA DE ESTADOS DEL DETECTOR DE CARATULAS")
		self.robot.tts.wait_until_done()
		return "succeeded"

def getInstance(robot):
	sm =smach.StateMachine(outcomes=['succeeded','aborted','preempted'],input_keys=['kid_name','animal_info'])


	with sm:
		smach.StateMachine.add('SETUP',Setup(robot),
			transitions={
				'succeeded':'DETECTOR'
			}
		)
		smach.StateMachine.add('DETECTOR',CoverDetector(robot),
			transitions={
				'preempted':'DETECTOR',
				'succeeded':'SHOW_COVERS'
			})
		smach.StateMachine.add('SHOW_COVERS',ShowWebpage(robot,page="http://198.18.0.1:8888/pics"),
			transitions={
				'succeeded':'WAIT_TOUCH_SCREEN',
				'preempted':'SHOW_COVERS'
			})
		smach.StateMachine.add('WAIT_TOUCH_SCREEN',WaitTouchScreen(robot),
			transitions={
			'succeeded':'SELECTOR'
			})
		smach.StateMachine.add('SELECTOR',Selector(robot),
			transitions={
				'succeeded':'succeeded'
			}
		)
	return sm


if __name__ == '__main__':
	rospy.init_node('COVERDETECTOR')

	robot= robot_factory.build([
		"audition",
		"tts",
		"tablet"],core=False)

	sm = getInstance(robot)


	sis = IntrospectionServer('cover_detector', sm, '/EA_COVER_DETECTOR') #Smach Viewer
	sis.start()
	outcome = sm.execute()
	sis.stop()
