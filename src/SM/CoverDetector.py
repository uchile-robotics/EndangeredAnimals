#!/usr/bin/env python
import rospy
import smach
from smach_ros import IntrospectionServer

from semu_skills import robot_factory

from uchile_states.interaction.states import Speak

from uchile_srvs.srv import DepthDetection, DepthDetectionRequest

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
		self.tts.say("Me gustaria guardar una imagen del animal para la ficha, puedes mostrarme una?")
		self.robot.tts.wait_until_done()
		rospy.sleep(10)


		#return "preemted" 
		#OJO esto genera un error debido a que en los outcomes definidos de las clase no se tiene el preemted , recomiendo ejecutar para ver el error
		return "succeeded"

class Selector(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded","aborted"]) # Aca se ponen todas las salidas que pueden tener este estado
		self.robot=robot
		self.audition=self.robot.get("audition")
		self.tts=self.robot.get("tts")
		#self.skill=self.robot.get("skill")

	def execute(self,userdata): # Userdata es informacion que se puede mover entre estados
		self.tts.set_language("Spanish")
		#self.tts.say("Setup detector de CARATULAS")
		self.tts.say("Ahora debemos seleccionar cual de las fotos quedarse")
		self.robot.tts.wait_until_done()
		rospy.sleep(10)


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

		cv2.imwrite("holi{}.jpg".format(self.it),crop_image)

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
	sm =smach.StateMachine(outcomes=['succeeded','aborted','preempted'],input_keys=['kid_name'])


	with sm:
		smach.StateMachine.add('SETUP',Setup(robot),
			transitions={
				'succeeded':'DETECTOR'
			}
		)
		smach.StateMachine.add('DETECTOR',CoverDetector(robot),
			transitions={
				'preempted':'DETECTOR',
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
		"tts"],core=False)

	sm = getInstance(robot)


	sis = IntrospectionServer('cover_detector', sm, '/EA_COVER_DETECTOR') #Smach Viewer
	sis.start()
	outcome = sm.execute()
	sis.stop()