#! /usr/bin/env python

from operator import xor
from tempfile import tempdir
from turtle import st
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
import numpy as np

x = 0
y = 0
z = 0
theta = 0

x2 = 0
y2 = 0
z2 = 0
theta2 = 0

seguida = False
circulo = False
cuadrado = False

def poseCallback(pose_message):
    global x
    global y
    global z
    global theta
    
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta
def pose2Callback(pose_message):
    global x2
    global y2
    global z2
    global theta2
    
    x2 = pose_message.x
    y2 = pose_message.y
    theta2 = pose_message.theta
def orientate (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):
        ka = 4.0
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
        dtheta = desired_angle_goal-theta        
        angular_speed = ka * (dtheta)
        velocity_message.linear.x = 0.0
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
       #print ('x=', x, 'y=', y)

        if (dtheta < 0.01):
            break
def inRange(x,x2):
    tolerance = 0.1
    return x2 - tolerance < x and x2 + tolerance > x

def oppositeAngle(angle):
    newAngle = angle + np.pi
    if(abs(newAngle) > np.pi):
        newAngle = -abs(newAngle)/newAngle * (abs(newAngle) - np.pi)

    return newAngle
def getSign(x):
    return abs(x) / x
def go_to_goal (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'
    pastTheta = theta
    tempBool = False
    esquivando = False
    pasoEsquivo = 0
    coordenadasPaso2 = [0,0]
    coordenadasPaso3 = [0,0]
    while(True):
        

        ka = 4.0
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)

        '''
        tempBool = desired_angle_goal < 0 and pastTheta > 0
        tempBool = tempBool or desired_angle_goal > 0 and pastTheta < 0
        
        if(tempBool):
            tempFloat = abs(abs(desired_angle_goal) - abs(pastTheta))
            print('diff=',tempFloat)
            if(tempFloat < 0.1 and abs(desired_angle_goal) > 2.5):
                print('Desired angle Ant=',desired_angle_goal, 'theta_ant=', pastTheta)
                if desired_angle_goal < pastTheta:
                    tempFloat = np.pi - pastTheta
                else:
                    tempFloat = np.pi - desired_angle_goal
                desired_angle_goal = pastTheta + tempFloat
                print('Desired angle=',desired_angle_goal,'tempFloat =',tempFloat)
                print("ok,lol")
        
        
       
        if abs(dtheta) > np.pi:
            dtheta = (abs(dtheta) - np.pi)
        '''
        dtheta = desired_angle_goal-theta
        if abs(dtheta) > np.pi:
            if inRange(abs(dtheta),2 * np.pi):
                
                dtheta = (abs(dtheta) - 2 * np.pi)  
            else:
                dtheta = (abs(dtheta) - np.pi)  
                
            print('dtheta new',dtheta)
        angular_speed = ka * (dtheta)
        if(abs(dtheta) > 0.5):
            kv = 0.1
        else:
            kv = 0.5
        if seguida:
            kv = kv * 2
        if esquivando:
            kv = kv * 7
        distance = abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
        linear_speed = kv * distance
        distanciaTurtles = np.sqrt((x2-x)**2 + (y2-y)**2)

        if distanciaTurtles < 3.0:
            '''
            linear_speed = -1/5 * linear_speed
            angular_speed = 0.0
            '''
            if not esquivando:
                lastCoordenada = [xgoal,ygoal]
                #Definir si el otro va en la misma direccion pero otro sentido
                tolerance = 0.1
                print('theta',theta)
                print('theta2',theta2)
                print("opTheta", oppositeAngle(theta))
                if inRange(theta,oppositeAngle(theta2)):
                    #definir vertical horizontal o digonal
                    #Vertical
                    print("Ruta col")
                    esquivando = True
                    
                    if inRange(np.pi/2,theta):
                        xgoal = x + 1.5
                        ygoal = y + 2 *abs(y2-y)/5
                        coordenadasPaso2 = [xgoal, ygoal + 1]
                        coordenadasPaso3 = [x,y + 2 *abs(y2-y)/5 + 3]
                    elif inRange(-np.pi /2,theta):
                        xgoal = x + 1.5
                        ygoal = y - 2 *abs(y2-y)/5
                        coordenadasPaso2 = [xgoal, ygoal - 1]
                        coordenadasPaso3 = [x,y - 2 *abs(y2-y)/5 - 3]
                    #Horizontal
                    elif inRange(0,theta):
                        xgoal = x + 2 *abs(x2-x)/5
                        ygoal = y + 1.5
                        coordenadasPaso2 = [xgoal + 1, ygoal]
                        coordenadasPaso3 = [x+ 2 *abs(x2-x)/5 + 3,y]
                    elif inRange(np.pi,theta) or inRange(-np.pi,theta):
                        xgoal = x - 2 *abs(x2-x)/5
                        ygoal = y + 1.5
                        coordenadasPaso2 = [xgoal - 1, ygoal]
                        coordenadasPaso3 = [x - 2 *abs(x2-x)/5 - 1,y]
                    
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        
        #print ('x=', x, 'y=', y)
        pastTheta = desired_angle_goal
        if seguida:
            if (distance < 0.1):
                break
        else:
            if not esquivando:
                if (distance < 0.01):
                    break
            else:
                if (distance < 0.1):
                    if pasoEsquivo == 0:
                        #moverse en y nada mas
                        xgoal = coordenadasPaso2[0]
                        ygoal = coordenadasPaso2[1]
                        pasoEsquivo += 1
                    elif pasoEsquivo == 1:
                        xgoal = coordenadasPaso3[0]
                        ygoal = coordenadasPaso3[1]
                        pasoEsquivo += 1
                    else:
                        xgoal = lastCoordenada[0]
                        ygoal = lastCoordenada[1]
                        pasoEsquivo = 0  
                        esquivando = False
def stop():
    print("Esquivator finito")

if __name__ == '__main__':
    try:

        rospy.init_node('turtlesim_motion_pose', anonymous = True)
        rospy.on_shutdown(stop)
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)

        position_topic = "/turtle2/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, pose2Callback)

        time.sleep(2)     
        delayTime = 1.0
        initPos = [1.0,5.5]
        pos = [[5.5,10.0]]

        for i in range(len(pos)):
            print(pos[i][0],"\t",pos[i][1])
            orientate(pos[i][0],pos[i][1])
            time.sleep(1.0)
            go_to_goal(pos[i][0],pos[i][1])
            time.sleep(1.0)	


    except rospy.ROSInterruptException:        
	    pass
