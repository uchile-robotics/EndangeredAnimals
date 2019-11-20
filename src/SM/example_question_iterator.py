#!/usr/bin/env python
import rospy
import smach
from smach_ros import IntrospectionServer

from maqui_skills import robot_factory


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


class IteratorManager(smach.State):
	def __init__(self,robot):
		self.it=0
		self.list=["Name","Location","Cantidad","etc"]
		self.max=len(self.list)
		smach.State.__init__(self,outcomes=["succeeded","aborted","preemted"],output_keys=["actual_question"]) # Preemted se usara cuando faltan preguntas a realizar 
		self.robot=robot
		self.tts=self.robot.get("tts")

	def execute(self,userdata):
		if self.it<self.max:
			# Realizar pregunta, aca lo que se debe realizar es poner en el userdata de actual question 
			#self.tts.say("Pregunta")
			userdata.actual_question="Pregunta numero {}".format(self.it)
			self.it+=1
			return "preemted"
		else:
			self.tts.say("Termine")
			return "succeeded"

		# Aca sacar de forma random una de lista,


class AskQuestions(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded"],input_keys=["actual_question"])
		self.robot=robot
		self.tts=self.robot.get("tts")
	
	def execute(self,userdata):
		self.tts.say(userdata.actual_question)
		return "succeeded"


def getInstance(robot):
	sm =smach.StateMachine(outcomes=['succeeded','aborted','preemted'])


	with sm:
		smach.StateMachine.add('SETUP',Setup(robot),
			transitions={
				'succeeded':'EXAMPLE'
			}
		)
		smach.StateMachine.add('EXAMPLE',Setup(robot),
			transitions={
				'succeeded':'ITERATORMANAGER'
			}
		)
		smach.StateMachine.add('ITERATORMANAGER',IteratorManager(robot),
			transitions={
				'succeeded':'succeeded',
				'preemted':'ASKQUESTIONS'
			}
		)
		smach.StateMachine.add('ASKQUESTIONS',AskQuestions(robot),
			transitions={
				'succeeded':'ITERATORMANAGER'
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