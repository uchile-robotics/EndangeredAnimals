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
from uchile_states.interaction.tablet_states import ShowWebpage, WaitTouchScreen



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
		# self.tts.say("Hola, te doy la bienvenida al proyecto animales en peligro de extincion. Para comenzar, les dare una serie de instrucciones. Debes hablarme fuerte y claro cuando mis ojos esten azules. Para comenzar, debes tocar la pantalla")
		# self.tts.wait_until_done()
		userdata.kid_name = "TEST"
		userdata.animal_info={"Name":"","Location":"","Cantidad":"","Data_random":"","Child_name":"","Image":"../static/img/animals/none.jpg"}
		msg=AnimalCard()
		msg.pic=userdata.animal_info["Image"]
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
	lista1 = ["Como se llama el animal?","De que animal me estas hablando?","Este animal tiene algun nombre?","Puedes decirme el nombre del animal?"]
	lista2 = ["Donde se encuentra ubicado?","Cual es su distribucion?","En que zona de Chile vive?","En que lugares de chile habita?"]
	lista3 = ["Cuantos especimenes quedan?","Cuantos hay actualmente?","Tienes algun numero de especimenes restantes?","Hay algun numero estimado de especies restantes?"]
	lista4 = ["Por que esta en peligro?" ,"Por que esta desapareciendo?","Que esta causando su disminucion?","Que esta motivando su desaparicion?", "Que motivos pone en peligro a este animal?"]
	lista5 = ["Como luce este animal?","Me puedes mostrar una imaagen del animal?","Tienes alguna foto para mostrar?","me puedes mostrar  una foto del animaal?"]
	lista6 = ["Como te llamas?", "Cual es tu nombre?", "Me podrias decir tu nombre?", "Puedes darme tu nombre?"]



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
		self.tts.say("Segun lo que aprendi, este animal llamado {}. vive en {}, donde quedan aproximadamente {}. Presenta amenazas como {}, que estan provocando su desaparicion. Todo esto lo pude aprender gracias al grupo de {}".format(
			userdata.animal_info["Name"],
			userdata.animal_info["Location"],userdata.animal_info["Cantidad"],
			userdata.animal_info["Data_random"],userdata.animal_info["Child_name"]))
		self.tts.wait_until_done()

		return "succeeded"



def getInstance(robot):
	sm =smach.StateMachine(outcomes=['succeeded','aborted','preempted'])


	with sm:
		smach.StateMachine.add('SETUP',Setup(robot),
			transitions={
				'succeeded':'SETUP2'
			}
		)
		smach.StateMachine.add('SETUP2',Speak(robot,text="Hola, te doy la bienvenida al proyecto animales en peligro de extincion. Para comenzar, les dare una serie de instrucciones. Deben hablarme fuerte y claro cuando mis ojos esten azules. Para comenzar, debes tocar la pantalla",gestures=True),
			transitions={
				'succeeded':'SHOW_LAMINA'
			})
		smach.StateMachine.add('SHOW_LAMINA',ShowWebpage(robot,page="http://198.18.0.1:8888/"),
			transitions={
				'succeeded':'WAIT_TOUCH_SCREEN',
				'preempted':'SHOW_LAMINA'
			})
		smach.StateMachine.add('WAIT_TOUCH_SCREEN',WaitTouchScreen(robot),
			transitions={
			'succeeded':'ITERATORMANAGER'
			})
		smach.StateMachine.add('ITERATORMANAGER',IteratorManager(robot),
			transitions={
				'succeeded':'RESUME',
				'preempted':'ASKQUESTIONS'
			}
		)
		smach.StateMachine.add('ASKQUESTIONS',AskQuestions(robot),
			transitions={
				'succeeded':'HEAR_INFO',
				'map_game':'LOAD_MAP',
				'cover_detector':'COVER_DETECTOR'
			}
		)
		smach.StateMachine.add('LOAD_MAP',ShowWebpage(robot,page="http://198.18.0.1:8888/map"),
			transitions={
				'succeeded':'MAP_GAME',
				'preempted':'LOAD_MAP'
			})
		smach.StateMachine.add('MAP_GAME',MapInteraction.getInstance(robot),
			transitions={
				'succeeded':'SHOW_LAMINA2'
			}
		)
		smach.StateMachine.add('SHOW_LAMINA2',ShowWebpage(robot,page="http://198.18.0.1:8888/"),
			transitions={
				'succeeded':'WAIT_TOUCH_SCREEN2',
				'preempted':'SHOW_LAMINA2'
			})
		smach.StateMachine.add('WAIT_TOUCH_SCREEN2',WaitTouchScreen(robot),
			transitions={
			'succeeded':'INFORMATION_SAVER'
			})
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
				'succeeded':'SHOW_LAMINA2'
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
