#!/usr/bin/env python
import rospy
import smach
from smach_ros import IntrospectionServer

from maqui_skills import robot_factory
from uchile_states.interaction.states import Hear,Speak
from uchile_states.interaction import HearWithInput 


class Iterator(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["preempted","aborted"])
		self.robot=robot
		self.it=0
		self.max_tries=2
		self.tts=self.robot.get("tts")

	def execute(self,userdata):
		self.tts.say("Oh, lo siento no te escuche bien, puedes repetir por favor?")
		Speak(self.robot,"Oh, lo siento no te escuche bien puedes repetir por favor?")
		self.robot.tts.wait_until_done()
		if self.it<self.max_tries:
			self.it+=1
			return "preempted"
		else:
			rospy.loginfo("Demasiados intentos")
			return "aborted"


class AddInfo(smach.State):
	def __init__(self,robot):
		smach.State.__init__(self,outcomes=["succeeded"],input_keys=["recognized_sentence","actual_question"])
		self.robot=robot

	def execute(self,userdata):
		## Aca debe ir el llamado a la funcion del felipe que guarda la info
		rospy.loginfo(userdata.actual_question[0]+userdata.recognized_sentence)
		Speak(self.robot,"Hola Escuche {}".format(userdata.recognized_sentence))
		tts=self.robot.get("tts")
		tts.say("Hola Escuche {}".format(userdata.recognized_sentence))
		return "succeeded"


def getInstance(robot):
	sm =smach.StateMachine(outcomes=['succeeded','aborted','preempted'],input_keys=["actual_question"])


	with sm:
		smach.StateMachine.add('HEAR',Hear(robot,timeout=20,eyes=False),
			transitions={
				'succeeded':'ADDINFO',
				'preempted':'ITERATOR'
			}
		)
		smach.StateMachine.add('ITERATOR',Iterator(robot),
			transitions={
				'aborted':'aborted',
				'preempted':'HEAR'
			}
		)
		smach.StateMachine.add('ADDINFO',AddInfo(robot),
			transitions={
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


	sis = IntrospectionServer('hear_info', sm, '/EA_HEAR_INFO') #Smach Viewer
	sis.start()
	outcome = sm.execute()
	sis.stop()