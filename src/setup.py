#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from turtlesim.srv import Kill, TeleportAbsolute, SetPen, Spawn
from turtlesim.msg import Pose

x = 0
y = 0
theta = 0

hall_width = rospy.get_param('hall_width')
wall_y_positions = [5.5-hall_width/2,5.5+hall_width/2]


def pose_callback(data):
	global x
	global y
	global theta
	x = data.x
	y = data.y
	theta = float(np.mod(data.theta,2*np.pi)) # bring angle value to within +2*pi

def walker(turtle_name,path_end):

	rospy.Subscriber(turtle_name+'/pose',Pose,pose_callback)

	pub = rospy.Publisher(turtle_name+'/cmd_vel', Twist, queue_size=10)
	rospy.init_node('environmental_setup', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	print('zoom zoom')
	while not (x >= path_end-.1 and x <= path_end+.1):
		lin = Vector3(11,0,0)
		ang = Vector3(0,0,0)
		go = Twist(lin,ang)
		pub.publish(go)

def teleport_me(x,y,theta):

	rospy.wait_for_service('turtle1/set_pen')
	try:
		print('Taking away my paint brush! :(')
		changepen = rospy.ServiceProxy('turtle1/set_pen',SetPen)
		changepen(69,86,255,15,0)
	except rospy.ServiceException, e:
		print "%s"%e

	rospy.wait_for_service('turtle1/teleport_absolute')
	try:
		print('Weeee I''m teleporting!')
		teleport = rospy.ServiceProxy('turtle1/teleport_absolute',TeleportAbsolute)
		resp = teleport(x,y,theta)
	except rospy.ServiceException, e:
		print "%s"%e

	print('Yay! My paint brush! :)')
	changepen(25,100,50,15,0)

if __name__ == '__main__':

	teleport_me(11,wall_y_positions[1],np.pi)
	print("T1: Let's walk!")
	walker('turtle1',0)
	print('T1: I don''t wanna die!')
	rospy.wait_for_service('kill')
	try:
		print('Don''t fire me! I''ve worked so hard!')
		retire = rospy.ServiceProxy('kill',Kill)
		resp = retire('turtle1')
	except rospy.ServiceException, e:
		print "%s"%e

	rospy.wait_for_service('spawn')
	try:
		new_turtle = rospy.ServiceProxy('spawn',Spawn)
		resp = new_turtle(11,wall_y_positions[0],np.pi,'turtle2')
	except rospy.ServiceException, e:
		print "%s"%e

	x = 11
	y = 2
	theta = np.pi
	rospy.wait_for_service('turtle2/set_pen')
	try:
		print('Taking away my paint brush! :(')
		changepen = rospy.ServiceProxy('turtle2/set_pen',SetPen)
		changepen(25,100,50,15,0)
	except rospy.ServiceException, e:
		print "%s"%e

	print("T2: Let's walk!")
	walker('turtle2',0)
	print('T2: I don''t wanna die!')
	rospy.wait_for_service('kill')
	try:
		print('Don''t fire me! I''ve worked so hard!')
		retire = rospy.ServiceProxy('kill',Kill)
		resp = retire('turtle2')
	except rospy.ServiceException, e:
		print "%s"%e

	rospy.set_param('setup_complete','True')

