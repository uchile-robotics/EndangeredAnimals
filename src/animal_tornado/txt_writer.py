#!/usr/bin/env python
import rospy
from uchile_msgs.msg import AnimalCard

def callback(data):
    rospy.loginfo("Saving Info! ;)")
    file_dir = "static/data/"+data.name+".txt"
    message = data.name+","+data.location+","+data.left_species+","+data.pic+","+data.extra_info+","+data.author
    with open(file_dir,"w") as file:
        file.write(message)
    rospy.loginfo("Info Saved! Tnx Buddy!")

    
def txt_writer():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('sub_test')

    rospy.Subscriber("/animal_info", AnimalCard, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    txt_writer()