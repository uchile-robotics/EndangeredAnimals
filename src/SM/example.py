#!/usr/bin/env python
import rospy
import smach
from smach_ros import IntrospectionServer



class Setup(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded","aborted"]) # Aca se ponen todas las salidas que pueden tener este estado
		self.robot=robot
		self.audition=self.robot.get("audition")
		self.tts=self.robot.get("tts")
		#self.skill=self.robot.get("skill")

	def execute(self,userdata): # Userdata es informacion que se puede mover entre estados
		self.tts.set_language("Spanish")
		self.tts.say("Hola esto es un test")

		#return "preemted" 
		#OJO esto genera un error debido a que en los outcomes definidos de las clase no se tiene el preemted , recomiendo ejecutar para ver el error
		return "succeeded"


def getInstance(robot):
	sm =smach.StateMachine(outcomes=['succeeded','aborted','preemted'])


	with sm:
		smach.StateMachine.add('SETUP',Setup(robot),
			trasitions={
				'succeeded':'EXAMPLE'
			}
		)
		smach.StateMachine.add('EXAMPLE',Setup(robot),
			trasitions={
				'succeeded':'succeeded'
			}
		)
	return sm


if __name__ == '__main__':
	rospy.init_node('ENDAGEREDANIMALS')

	robot= robot_factory.build([
		"audition",
		"tts"],core=True)

	sm = getInstance(robot)


    sis = IntrospectionServer('example_state', sm, '/EA_INIT_STATE') #Smach Viewer
    sis.start()
    outcome = sm.execute()
    sis.stop()