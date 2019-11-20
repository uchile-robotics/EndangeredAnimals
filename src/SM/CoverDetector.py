#!/usr/bin/env python
import rospy
import smach
from smach_ros import IntrospectionServer

from maqui_skills import robot_factory

from uchile_states.interaction.states import Speak

class Setup(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded","aborted"]) # Aca se ponen todas las salidas que pueden tener este estado
		self.robot=robot
		self.audition=self.robot.get("audition")
		self.tts=self.robot.get("tts")
		#self.skill=self.robot.get("skill")

	def execute(self,userdata): # Userdata es informacion que se puede mover entre estados
		self.tts.set_language("Spanish")
		self.tts.say("Setup detector de CARATULAS")

		#return "preemted" 
		#OJO esto genera un error debido a que en los outcomes definidos de las clase no se tiene el preemted , recomiendo ejecutar para ver el error
		return "succeeded"


class CoverDetector(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded"],input_keys=['kid_name'])
		self.robot=robot
		self.tts=self.robot.get("tts")
		self.tablet=self.robot.get("tablet")
		#self.cover_detector=self.robot.get("cover_detector") # POR IMPLEMENTAR
	def execute(self,userdata):
		rospy.loginfo(userdata.kid_name)
		#Crear skill para el detector de caratulas
		# path = path a la imagen
		# self.cover_detector.save_cover(path)
		# self.tablet.show_image(path)
		Speak(self.robot,"FALTA POR IMPLEMENTAR MAQUINA DE ESTADOS DEL DETECTOR DE CARATULAS")
		return "succeeded"

def getInstance(robot):
	sm =smach.StateMachine(outcomes=['succeeded','aborted','preemted'],input_keys=['kid_name'])


	with sm:
		smach.StateMachine.add('SETUP',Setup(robot),
			transitions={
				'succeeded':'DETECTOR'
			}
		)
		smach.StateMachine.add('DETECTOR',CoverDetector(robot),
			transitions={
				'succeeded':'succeeded'
			})
	return sm


if __name__ == '__main__':
	rospy.init_node('COVERDETECTOR')

	robot= robot_factory.build([
		"audition",
		"tts"],core=True)

	sm = getInstance(robot)


	sis = IntrospectionServer('cover_detector', sm, '/EA_COVER_DETECTOR') #Smach Viewer
	sis.start()
	outcome = sm.execute()
	sis.stop()