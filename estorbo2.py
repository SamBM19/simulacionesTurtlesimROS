#! /usr/bin/env python

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

seguida = False
circulo = True
cuadrado = False

def poseCallback(pose_message):
    global x
    global y
    global z
    global theta
    
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta
def inRange(x,x2):
    tolerance = 0.1
    return x2 - tolerance < x and x2 + tolerance > x
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

def go_to_goal (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'
    pastTheta = theta
    tempBool = False
    while(True):
        

        ka = 4.0
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
        '''
        tempBool = desired_angle_goal < 0 and pastTheta > 0
        tempBool = tempBool or desired_angle_goal > 0 and pastTheta < 0
        #
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
        '''
        
        dtheta = desired_angle_goal-theta 
        print('dtheta',dtheta)
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
        distance = abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
        linear_speed = kv * distance

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        #print ('x=', x, 'y=', y)
        pastTheta = desired_angle_goal
        if seguida:
            if (distance < 0.1):
                break
        else:
            if (distance < 0.01):
                break
def hacerCuadrado(o, off,n):
    l = o + n
    return [[o, o+off],[o,l],[o+off, l],[l,l],[l, l-off],[l,o],[l-off, o],[o,o]]
def hacerGamma(o, p, off,n):
    
    return [[o, p+off],[o,p+n],[o-off, p+n],[o-n,p+n]]

def hacerRombo(o,off,n):
    return [[o-off, o+off],[o-n,o+n],[o-n+off,o+n+off],[o,o+2*n],[o+off,o+2*n-off],[o+n,o+n],[o+n-off,o+n-off],[o,o]]
def hacerCirculo(steps, centro,r):
    angStep = 2 * np.pi / (steps-1)
    angle = 0
    pos = []
    for i in range(steps):

        x = r * np.cos(angle)
        
        #x**2 + y**2 = r**2
        #
        y = np.sqrt(r**2 - x**2)
        if(angle > np.pi):
            y = -y
        
        pos.append([x+ centro[0],y + centro[1]])
        angle += angStep
    pos.append(pos[0])
    return pos
def stop():
    print("Esquivator finito")

if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous = True)
        rospy.on_shutdown(stop)
        cmd_vel_topic = '/turtle2/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)


        position_topic = "/turtle2/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)

        time.sleep(2)     
        delayTime = 1.0
        initPos = [5.5,10.0]
        pos = [[5.5,5.5]]

        for i in range(len(pos)):
            print(pos[i][0],"\t",pos[i][1])
            orientate(pos[i][0],pos[i][1])
            time.sleep(1.0)
            go_to_goal(pos[i][0],pos[i][1])
            time.sleep(1.0)	


    except rospy.ROSInterruptException:        
	    pass
