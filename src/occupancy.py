#!/usr/bin/env python

import numpy as np
import math
import rospy
import tf

from geometry_msgs.msg import Point, Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import *


import matplotlib.pyplot as plt

#declaracao de variaveis
laserMsg = None
odomMsg = None
goal = None
goals = []
euler = None

map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'

resolution = 10
width = 32 * resolution
height = 32 * resolution
mapa = [[0.5 for i in range(0,width)] for j in range(0,height)]
grid = [[0.5 for i in range(0,width)] for j in range(0,height)]

def map(odom,laserMsg):
	global mapa
	global height
	global width
	global resolution
	global grid

	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y

	prob_base = 0.5
	for idx, value in enumerate(laserMsg.ranges): # intera na mensagem do laser
		a = idx/2

		for angle in np.arange(a-.1,a+.1, 0.1):
			#espaco em brancos
			for dist in np.arange(0,min(value,8),0.1):
				x_obj_map = math.floor(dist * math.cos(math.radians(angle)) * resolution )
				y_obj_map = math.floor(dist * math.sin(math.radians(angle)) * resolution)
				x_obj_map_real = -(math.floor(x * resolution) + width/2 ) + x_obj_map
				y_obj_map_real = -(math.floor(y * resolution) + height/2 ) + y_obj_map

				grid_anterior = grid[int(y_obj_map_real)+1][int(x_obj_map_real)+1]

				prob_ocupado = 0.2
				#calculos de probabilidade
				grid[int(y_obj_map_real)+1][int(x_obj_map_real)+1] = grid_anterior + math.log(prob_ocupado / (1-prob_ocupado)) - math.log(prob_base / (1-prob_base))
				try:
					exp = math.exp(grid_anterior)
				except OverflowError:
					exp = float('inf')
				mapa[int(y_obj_map_real)+1][int(x_obj_map_real)+1] = 1 - (1 / ( 1 + exp ))
			#espaco com obstaculo
			for dist in np.arange(value-0.1,value+.1, 0.1):
				x_obj_map = math.floor(dist * math.cos(math.radians(angle)) * resolution )
				y_obj_map = math.floor(dist * math.sin(math.radians(angle))  * resolution)

				x_obj_map_real = -(math.floor(x * resolution) + width/2 ) + x_obj_map + 1
				y_obj_map_real = -(math.floor(y * resolution) + height/2 ) + y_obj_map + 1

				grid_anterior = grid[int(y_obj_map_real)+1][int(x_obj_map_real)+1]

				prob_ocupado = 0.8

		#calculos de probabilidade
		grid[int(y_obj_map_real)+1][int(x_obj_map_real)+1] = grid_anterior + math.log(prob_ocupado / (1-prob_ocupado)) - math.log(prob_base / (1-prob_base))
		try:
			exp = math.exp(grid_anterior)
		except OverflowError:
			exp = float('inf')

		mapa[int(y_obj_map_real)+1][int(x_obj_map_real)+1] = 1 - (1 / ( 1 + exp ))

def init_goals(file):
	#trajetos para os arquivos de teste
    global goals
    print file
    if 'holonomic_map.world' in file:
    	goals = [[5.5,4,0],[5.5,10,0],[7,11,0],[10,11,0],[15,15,0],[6,15,0],[5,13.5,0],[1,13.5,0],[1,11,0],[7,7,0],[13,6.5,0],[11,1,0]
		,[1,1,0]]#cave
    elif 'holonomic_paredes.world' in file:
    	goals = [[1,3,0],[7,4.5,0],[3,11,0],[1,12,0],[1,15.5,0],[15,15.5,0],[15,1,0],[12,6,0],[12,10,0]]#paredes
    elif 'holonomic_autolab.world' in file:
	goals = [[3.5,14,0],[3.5,12.5,0],[1,12,0],[1,5,0],[8.5,5,0],[8.5,2,0],[8.5,10,0],[8.5,6,0],[5.5,9,0],[5.5,12,0],[10,12,0],[6,12,0]
		,[4.5,12.4,0],[4.5,15,0]]#autolab

def LaserCallback(msg):
	global laserMsg
	laserMsg = msg

def OdomCallback(msg):
	global odomMsg
	odomMsg = msg
	quaternion = (
		odomMsg.pose.pose.orientation.x,
		odomMsg.pose.pose.orientation.y,
		odomMsg.pose.pose.orientation.z,
		odomMsg.pose.pose.orientation.w
		)
	global euler
	euler = tf.transformations.euler_from_quaternion(quaternion)

def distance(msg,goal): #distancia euclidiana
	return math.sqrt(pow((goal.x - msg.pose.pose.position.x),2)+pow((goal.y - msg.pose.pose.position.y),2))

def atracao(v1,v2):
	return (v1-v2)

def repulsao(msg,theta):
	#forca de repulsao
    fx = 0;fy = 0
    treshold = 0.4
    dx = 0; dy = 0
    angle = np.argmin(np.array(laserMsg.ranges))
    for idx,value in enumerate(msg.ranges):
        if (value < treshold):
        	fx += -0.1*(1/value - 1/treshold)*(1/value**2)*(-math.cos(math.radians(msg.angle_increment*idx+msg.angle_min)))
        	fy += -0.1*(1/value - 1/treshold)*(1/value**2)*(-math.sin(math.radians(msg.angle_increment*idx+msg.angle_min)))

    return fx,fy

def run():
	rospy.init_node('occupancy',anonymous = True )
	occ_pub = rospy.Publisher("/map", OccupancyGrid, queue_size = 10)
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size =10)
	rospy.Subscriber('/base_scan',LaserScan,LaserCallback)
	rospy.Subscriber('/base_pose_ground_truth',Odometry,OdomCallback)
	rate = rospy.Rate (5) # 5 hz
	cmd_vel = Twist()
	file  = rospy.get_param("~file")
	global goal
	global goals
	global mapa,resolution,width,height
    	init_goals(file)
	goal = Pose2D
    	goal.x = goals[0][0]
    	goal.y = goals[0][1]
    	goal.theta = goals[0][2]
	i = 0
    	global euler
	fim = False
	minimo_local = False
	iterations = 100

	#cabecalho topico rviz
	map_msg.info.resolution = resolution
	map_msg.info.width = width
	map_msg.info.height = height
	map_msg.data = range(map_msg.info.width * map_msg.info.height)

	map_msg.info.origin.orientation.x = -0.70711
	map_msg.info.origin.orientation.y = 0.70711
	map_msg.info.origin.orientation.z = 0
	map_msg.info.origin.orientation.w = 0

	while (not rospy.is_shutdown() and not fim): #enquanto nao rodo todos os goals e enquanto o mapa esta mudando
		#falha nos sensores
		if (laserMsg == None) or (odomMsg == None) :
			rate.sleep ()
			continue

	        if distance(odomMsg,goal) < 1.0 or minimo_local or iterations == 0: #chegando no goal, mudar goal de posicao
            		minimo_local = False
            		iterations = 100
            		if i is len(goals) - 1:
                		fim = True
                		break
            		else:
			    	goal.x = goals[i+1][0]
			    	goal.y = goals[i+1][1]
			    	goal.theta = goals[i+1][2]
				i += 1
		fax = atracao(goal.x,odomMsg.pose.pose.position.x)
        	fay = atracao(goal.y,odomMsg.pose.pose.position.y)
		frex,frey = repulsao(laserMsg,euler[2])
		frx = fax + frex
		fry = fay + frey
		if abs(fax*10) < abs(frex) and abs(fay*10) < abs(frey):#minimo local
			cmd_vel.linear.x = 0.0
			cmd_vel.angular.z = 0.0
			minimo_local = True
  	 	else:
			cmd_vel.linear.x = frx
			cmd_vel.linear.y = fry
		map(odomMsg,laserMsg)
		np_mapa = np.asarray(mapa)
		map_msg.data = 100*np_mapa.flatten()

		occ_pub.publish(map_msg)
		iterations -= 1
		pub.publish(cmd_vel)
		rate.sleep()

	#fim do loop
	cmd_vel.linear.x = 0.0
	cmd_vel.linear.y = 0.0
	pub.publish(cmd_vel)

	#plotar ultimo mapa gerado
	mapa_p = 255*(np.array(mapa) / np.max(np.array(mapa)) )

	mapa_p = [255-x for x in mapa_p]

	plt.imshow(mapa_p,cmap='gray')
	plt.show()

if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
