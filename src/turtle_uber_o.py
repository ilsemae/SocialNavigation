#!/usr/bin/env python

import rospy
import numpy as np
import random
import sys
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute
from turtlesim.msg import Color, Pose

#pose values for turtle_name turtle
x = 0
y = 0
theta = 0

turtle_poses = dict({"theo":[0,0],
	                 "josephine":[0,0],
	                 "yma":[0,0],
	                 "alan":[0,0],
	                 "everett":[0,0],
	                 "rose":[0,0],
	                 "zed":[0,0],
	                 "robin":[0,0]})

# wall parameters drawn in white by the first turtle which gets killed afterwards
hall_width = rospy.get_param('hall_width')
wall_y_positions = [5.5-hall_width/2,5.5+hall_width/2]

#u sed to set global variables for the pose of this node's turtle's position
def pose_callback(data):
	global x
	global y
	global theta
	x = data.x
	y = data.y
	theta = float(np.mod(data.theta,2*np.pi)) # store angle value within +2*pi

# used to set global variable for the pose of all turtles' positions
def t_pose_callback(data):
	topic_name=data._connection_header['topic']
	turtle_name=topic_name.replace('/',"").replace("pose","")
	global turtle_poses
	turtle_poses[turtle_name][0] = data.x
	turtle_poses[turtle_name][1] = data.y

def get_net_force(turtle_name,goal):

	net_force = np.array([0,0])

	goal_coefficient = 0

	# add the social force from each stranger turtle
	for stranger in ["theo","josephine","yma","alan","everett","rose","zed","robin"]:

		global turtle_poses
		rospy.Subscriber(stranger+'/pose',Pose,t_pose_callback)

		if stranger!=turtle_name:	# There is no force on that turtle from itself.
			if turtle_poses[stranger]==[0,0]:
				time.sleep(.1) #wait for turtle's pos to be updated for the first time
				if turtle_poses[stranger]==[0,0]: # if it still hasn't updated, that's because there is no turtle with that name (lazy programming on my part)
					continue # skip this iteration
			goal_coefficient = goal_coefficient + 20
			stranger_pos = np.array(turtle_poses[stranger])
			stranger_pointer = stranger_pos-np.array([x,y]) # vector from current turtle to stranger turtle
			stranger_dist = np.linalg.norm(stranger_pointer)
			if stranger_pointer[1] < .1:
				stranger_pointer[1] = .1
			if stranger_pointer[0] < .1:
				stranger_pointer[0] = .1
			stranger_force = np.array([.1/stranger_pointer[0],.5/stranger_pointer[1]])
			net_force = net_force-stranger_force
			print(turtle_name+": due to "+stranger+": "+str(-stranger_force))
			if stranger_dist<.5:
				print(turtle_name+": Oh dear! Pardon me, "+stranger+"! So sorry to have bumped you!")
			#elif stranger_dist<2 and stranger_dist>=1.8:
			#	print(turtle_name+": Yikes! Watch out for "+stranger+"!")

	# add the social forces from the walls
	global wall_y_positions
	dists_to_obst = [wall_y_positions[0]-y,wall_y_positions[1]-y] # y distances to each wall
	if dists_to_obst[0]>0:
		print(turtle_name+"- Oy! off the grass!")
		walls_force = np.array([0,7*abs(dists_to_obst[0])])
	elif dists_to_obst[1]<0:
		print(turtle_name+" - Oy! Off the grass!")
		walls_force = np.array([0,-7*abs(dists_to_obst[1])])
	else:
		pointers = [[0,-1/dists_to_obst[0]],[0,-1/dists_to_obst[1]]]
		walls_force = np.array(pointers[0]) + np.array(pointers[1])
	net_force = net_force + walls_force
	print(turtle_name+": due to walls:"+str(walls_force))

	# add the social force of the goal
	if goal == "right":
		dist_to_goal = 11 - x
	else:
		dist_to_goal = 0 - x

	goal_vect = np.array([1/dist_to_goal,0])
	net_force = net_force + goal_coefficient*goal_vect #this coefficient needs to be larger the more turtles you have 15*(n-1)
	print(turtle_name+": due to goal:"+str(goal_coefficient*goal_vect))

	return net_force

def driver(turtle_name):
	global r
	global g
	global b
	global x
	global y
	global theta

	rospy.Subscriber(turtle_name+'/pose',Pose,pose_callback)

	pub = rospy.Publisher(turtle_name+'/cmd_vel', Twist, queue_size=10)
	rospy.init_node(turtle_name+'_uber', anonymous=True)
	rate = rospy.Rate(300) # hz

	goal=rospy.get_param(turtle_name+"_goal")
	print("Hi! My name is "+turtle_name+" and my goal is "+goal)

	while not rospy.is_shutdown():

		# compute net force on turtle_name
		my_pos = np.array([x,y])
		net_force = get_net_force(turtle_name,goal)
		net_force = net_force/np.linalg.norm(net_force) #keep the net force magnitude 1
		#print(turtle_name+": Ok! My net force is "+str(net_force))
		#raw_input()
		# drive turtle based on net force and current position

		# if the turtle has reached its goal
		if goal=="right" and abs(x-11) < 1 or goal=="left" and abs(x-0) < 1:
			print(turtle_name+": Yay! I reached my goal!")
			lin = Vector3(0,0,0)
			ang = Vector3(0,0,0)
			return

		# otherwise, head in the direction of the net force
		else:
			force_angle = np.arctan2(net_force[1],net_force[0])
			#print(turtle_name+": Ok! My force angle is "+str(force_angle))

			force_angle = np.mod(force_angle,2*np.pi) # bring angle value to within +2*pi
			angle_difference = force_angle-theta
			i   = np.sign(angle_difference)
			if abs(angle_difference) > np.pi:
				i = -i
			if abs(angle_difference) < np.pi/2 or abs(angle_difference)>3*np.pi/2:
				lin = Vector3(1,0,0)
				ang = Vector3(0,0,i/2)
			else:
				lin = Vector3(0,0,0)
				ang = Vector3(0,0,5*i)
				while abs(angle_difference)>.05:
					#print(turtle_name+": Ok! My angle error is "+str(angle_difference)+", so I am spinning!")
					angle_difference = force_angle-theta
					stumble = Twist(lin,ang)
					pub.publish(stumble)
					#rate.sleep()
				lin = Vector3(1,0,0)
				ang = Vector3(0,0,0)
			#print(turtle_name+": Ok! My angle error is "+str(angle_difference)+", so I am walking!")
			#print(turtle_name+": My goal is "+str(goal)+". My force angle is "+str(force_angle)+". My current angle is "+str(theta))

		stumble = Twist(lin,ang)
		pub.publish(stumble)
		rate.sleep()
		#raw_input()

#Spawn the turtle for this node
def mommy(baby_name):
	global x
	global y
	global theta
	global wall_y_positions

	rospy.wait_for_service('spawn')
	try:
		new_turtle = rospy.ServiceProxy('spawn',Spawn)
		goal = rospy.get_param(baby_name+'_goal')
		if goal=="right":
			x = 0
			theta = 0
		else:
			x = 11
			theta = np.pi
		y0 = rospy.get_param(baby_name+"_y0")
		if y0 == "top":
			y = wall_y_positions[1]-1
		elif y0 =="bottom":
			y = wall_y_positions[0]+1
		else:
			y = 5.5
		resp = new_turtle(x,y,theta,baby_name)
	except rospy.ServiceException, e:
		print "%s"%e

	rospy.wait_for_service(baby_name+'/set_pen')
	try:
		changepen = rospy.ServiceProxy(baby_name+'/set_pen',SetPen)
		changepen(np.random.randint(0,255),np.random.randint(0,255),np.random.randint(0,255),5,0)
	except rospy.ServiceException, e:
		print "%s"%e


if __name__ == '__main__':

	while not rospy.get_param('setup_complete')=="True":
		time.sleep(1)

	baby_name = rospy.myargv(argv=sys.argv)[1]
	mommy(baby_name)
	try:
		driver(baby_name)
	except rospy.ROSInterruptException:
		pass



