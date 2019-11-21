#!/usr/bin/env python
import rospy
import smach
import random
from smach_ros import IntrospectionServer

from maqui_skills import robot_factory
import MapGame
import CoverDetector
import HearInfo
from uchile_states.interaction.states import Speak



class Setup(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded","aborted"],output_keys=["kid_name"]) # Aca se ponen todas las salidas que pueden tener este estado
		self.robot=robot
		self.audition=self.robot.get("audition")
		self.tts=self.robot.get("tts")
		#self.skill=self.robot.get("skill")

	def execute(self,userdata): # Userdata es informacion que se puede mover entre estados
		self.tts.set_language("Spanish")
		Speak(self.robot,"Hola esto es un test")
		userdata.kid_name="TEST"
		#return "preemted" 
		#OJO esto genera un error debido a que en los outcomes definidos de las clase no se tiene el preemted , recomiendo ejecutar para ver el error
		return "succeeded"

def GetQuestion(keyword):
	#Cada una de estas listas son las 4 variantes para el tipo de preguntas para la misma info

	lista1=["Como se llama?","De que animal me estas hablando?","PENDIENTE VARIABLE 3","PENDIENTE VARIABLE 4"]
	lista2=["Donde se encuentra?","AAA","SSS","BBB"]
	lista3=["9","10","11","12"]
	lista4=["13","14","15","16"]
	lista5=["Como luce?","Me puedes mostrar una imagen","15","16"]




	lista=[lista1,lista2,lista3,lista4,lista5]
	n=random.randint(0,3)
	rospy.loginfo(n)
	return lista[keyword][n]


class IteratorManager(smach.State):
	def __init__(self,robot):
		self.it=0
		self.list=["Name","Location","Cantidad","Image"]
		self.max=len(self.list)
		smach.State.__init__(self,outcomes=["succeeded","aborted","preempted"],output_keys=["actual_question"]) # Preemted se usara cuando faltan preguntas a realizar 
		self.robot=robot
		self.tts=self.robot.get("tts")

	def execute(self,userdata):
		if self.it<self.max:
			# Realizar pregunta, aca lo que se debe realizar es poner en el userdata de actual question 
			#self.tts.say("Pregunta")
			userdata.actual_question=[self.list[self.it],"{}".format(GetQuestion(self.it))]
			self.it+=1
			return "preempted"
		else:
			self.tts.say("Termine")
			return "succeeded"

		# Aca sacar de forma random una de lista,


class AskQuestions(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded","map_game","cover_detector"],input_keys=["actual_question"])
		self.robot=robot
		self.tts=self.robot.get("tts")
	
	def execute(self,userdata):
		#self.tts.say(userdata.actual_question[1])
		Speak(self.robot,userdata.actual_question[1])
		if userdata.actual_question[0]=="Location":
			return "map_game"
		if userdata.actual_question[0]=="Image":
			return "cover_detector"
		return "succeeded"




def getInstance(robot):
	sm =smach.StateMachine(outcomes=['succeeded','aborted','preempted'])


	with sm:
		smach.StateMachine.add('SETUP',Setup(robot),
			transitions={
				'succeeded':'ITERATORMANAGER'
			}
		)
		smach.StateMachine.add('ITERATORMANAGER',IteratorManager(robot),
			transitions={
				'succeeded':'succeeded',
				'preempted':'ASKQUESTIONS'
			}
		)
		smach.StateMachine.add('ASKQUESTIONS',AskQuestions(robot),
			transitions={
				'succeeded':'HEAR_INFO',
				'map_game':'MAP_GAME',
				'cover_detector':'COVER_DETECTOR'
			}
		)
		smach.StateMachine.add('MAP_GAME',MapGame.getInstance(robot),
			transitions={
				'succeeded':'ITERATORMANAGER'
			}
		)
		smach.StateMachine.add('HEAR_INFO',HearInfo.getInstance(robot),
			transitions={
				'succeeded':'ITERATORMANAGER',
				'aborted':'ITERATORMANAGER'
			}
		)
		smach.StateMachine.add('COVER_DETECTOR',CoverDetector.getInstance(robot),
			transitions={
				'succeeded':'ITERATORMANAGER'
			}
		)
	return sm


if __name__ == '__main__':
	rospy.init_node('ENDAGEREDANIMALS')

	robot= robot_factory.build([
		"audition",
		"tts",
		"tablet"],core=False)

	sm = getInstance(robot)


	sis = IntrospectionServer('example_state', sm, '/EA_INIT_STATE') #Smach Viewer
	sis.start()
	outcome = sm.execute()
	sis.stop()