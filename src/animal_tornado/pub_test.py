import rospy
from uchile_msgs.msg import AnimalCard

rospy.init_node("test")
pub = rospy.Publisher("/animal_file", AnimalCard, queue_size=10)
msg = AnimalCard()
msg.name = "Ranita"
msg.location = "uwu"
msg.left_species = "420"
msg.pic = "../static/img/animal_pics/ranita.jpg"
msg.extra_info = "iwi"
msg.author = "ewe"
rospy.sleep(1)	
pub.publish(msg)
print("Published!")


