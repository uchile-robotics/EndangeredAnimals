#!/usr/bin/env python
import rospy
import smach
from smach_ros import IntrospectionServer

from maqui_skills import robot_factory

from uchile_states.interaction.states import Speak
from uchile_srvs.srv import Onoff,PersonDetection, PersonDetectionResponse


class Setup(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded","aborted"]) # Aca se ponen todas las salidas que pueden tener este estado
		self.robot=robot
		self.audition=self.robot.get("audition")
		self.tts=self.robot.get("tts")
		#self.skill=self.robot.get("skill")

	def execute(self,userdata): # Userdata es informacion que se puede mover entre estados
		self.tts.set_language("Spanish")
		#self.tts.say("Setup juego del mapa")
		self.tts.say("Podrias seleccionar una parte de Chile donde se encuentra el animal?")
		self.robot.tts.wait_until_done()
		rospy.sleep(10)

		#return "preemted" 
		#OJO esto genera un error debido a que en los outcomes definidos de las clase no se tiene el preemted , recomiendo ejecutar para ver el error
		return "succeeded"


class Game(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded"])
		self.robot=robot
		self.tts=self.robot.get("tts")
	def execute(self,userdata):
		Speak(self.robot,"SETUP JUEGO DEL MAPA")
		return "succeeded"

def getInstance(robot):
	sm =smach.StateMachine(outcomes=['succeeded','aborted','preempted'])


	with sm:
		smach.StateMachine.add('SETUP',Setup(robot),
			transitions={
				'succeeded':'GAME'
			}
		)
		smach.StateMachine.add('GAME',Game(robot),
			transitions={
				'succeeded':'succeeded'
			})
	return sm


if __name__ == '__main__':
	rospy.init_node('MAPGAME')

	robot= robot_factory.build([
		"audition",
		"tts"],core=True)

	sm = getInstance(robot)


	sis = IntrospectionServer('map_game', sm, '/EA_MAP_GAME') #Smach Viewer
	sis.start()
	outcome = sm.execute()
	sis.stop()