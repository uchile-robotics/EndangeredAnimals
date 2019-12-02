#!/usr/bin/env python
import rospy
import smach
import random
from smach_ros import IntrospectionServer

from maqui_skills import robot_factory
import MapInteraction
import CoverDetector
import HearInfo
from uchile_states.interaction.states import Speak
from uchile_msgs.msg import AnimalCard



class Setup(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded","aborted"],output_keys=["kid_name"],io_keys=["animal_info"]) # Aca se ponen todas las salidas que pueden tener este estado
		self.robot = robot
		self.audition = self.robot.get("audition")
		self.tts = self.robot.get("tts")
		self.animal_info_pub = rospy.Publisher('/animal_file',AnimalCard , queue_size=1)
		
		#self.skill=self.robot.get("skill")

	def execute(self,userdata): # Userdata es informacion que se puede mover entre estados
		self.tts.set_language("Spanish")
		self.tts.say("Hola, esto es animales en extincion")
		self.tts.wait_until_done()
		userdata.kid_name = "TEST"
		userdata.animal_info={"Name":"","Location":"","Cantidad":0,"Data_random":"","Child_name":"","Image":""}
		msg=AnimalCard()
		rospy.sleep(2)
		rospy.loginfo(msg)
		rospy.sleep(2)
		self.animal_info_pub.publish(msg)

		#return "preemted" 
		#OJO esto genera un error debido a que en los outcomes definidos de las clase no se tiene el preemted , recomiendo ejecutar para ver el error
		return "succeeded"

def GetQuestion(keyword):
	#Cada una de estas listas son las 4 variantes para el tipo de preguntas para la misma info
	#NOMBRE;HABITAT en Chile;ESPECIMENES RESTANTES; DATOS; FOTO; QUIEN ME ENSENO
	lista1 = ["Como se llama?","De que animal me estas hablando?","Este animal tiene algun nombre?","Puedes decirme su nombre?"]
	lista2 = ["Donde se encuentra?","Cual es su distribucion?","En que zona de Chile vive?","En que lugares habita"]
	lista3 = ["Cuantos especimenes quedan?","Cuantos hay actualmente?","Tienes algun numero de especimenes restantes?","Hay algun numero estimado de especies restantes?"]
	lista4 = ["Por que esta en peligro?" ,"Por que esta desapareciendo?","Que esta causando su disminucion?","Que esta motivando su desaparicion?", "Que motivos pone en peligro a este animal?"]
	lista5 = ["Como luce?","Me puedes mostrar una imaagen","Tienes alguna foto para mostrar?","me puedes mostrar  una foto del animaal?"]
	lista6 = ["Como te llamas?", "Cual es tu nombre?", "Me podrias decir tu nombre?", "Quien eres?"]



	lista=[lista1,lista2,lista3,lista4,lista5, lista6]
	n=random.randint(0, 3)
	rospy.loginfo(n)
	rospy.loginfo(keyword)
	return lista[keyword][n]


class IteratorManager(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded","aborted","preempted"],output_keys=["actual_question"]) # Preemted se usara cuando faltan preguntas a realizar 
		self.it=0
		self.list=["Name","Location","Cantidad","Data_random", 'Image', 'Child_name']
		self.max=len(self.list)
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
		self.tts.say(userdata.actual_question[1])
		if userdata.actual_question[0]=="Location":
			return "map_game"
		if userdata.actual_question[0]=="Image":
			return "cover_detector"
		return "succeeded"


class InformationSaver(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded"],io_keys=["animal_info"])
		self.robot=robot
		self.animal_info_pub = rospy.Publisher('/animal_file',AnimalCard , queue_size=1)
		rospy.sleep(2)

	def execute(self,userdata):
		msg=AnimalCard()
		msg.name=userdata.animal_info["Name"]
		msg.location=userdata.animal_info["Location"]
		msg.left_species=userdata.animal_info["Cantidad"]
		msg.pic=userdata.animal_info["Image"]
		msg.extra_info=userdata.animal_info["Data_random"]
		msg.author=userdata.animal_info["Child_name"]
		rospy.sleep(2)
		self.animal_info_pub.publish(msg)
		rospy.sleep(2)
		rospy.loginfo(msg)
		return "succeeded"


class Resume(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded"],input_keys=["animal_info"])
		self.robot=robot
		self.tts=self.robot.get("tts")

	def execute(self,userdata):
		self.tts.say("Aprendi lo siguiente {} tambien {} tambien {} tambien {} tambien{}".format(userdata.animal_info["Name"],
			userdata.animal_info["Location"],userdata.animal_info["Cantidad"],
			userdata.animal_info["Data_random"],userdata.animal_info["Child_name"]))
		self.tts.wait_until_done()

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
				'succeeded':'RESUME',
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
		smach.StateMachine.add('MAP_GAME',MapInteraction.getInstance(robot),
			transitions={
				'succeeded':'INFORMATION_SAVER'
			}
		)
		smach.StateMachine.add('HEAR_INFO',HearInfo.getInstance(robot),
			transitions={
				'succeeded':'INFORMATION_SAVER',
				'aborted':'ITERATORMANAGER'
			}
		)
		smach.StateMachine.add('INFORMATION_SAVER',InformationSaver(robot),
			transitions={
				'succeeded':'ITERATORMANAGER'
			})
		smach.StateMachine.add('COVER_DETECTOR',CoverDetector.getInstance(robot),
			transitions={
				'succeeded':'ITERATORMANAGER'
			}
		)
		smach.StateMachine.add('RESUME',Resume(robot),
			transitions={
				'succeeded':'succeeded'
			})
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