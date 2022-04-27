#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENPM 661 - Dr. Reza Monfaredi
Project 3 Part 2
Due date: April 24, 11:59 p.m.

@author: jcheng and ezoboli
"""
import numpy as np
import math
import sys
from collision import Vector, Poly, Concave_Poly, Circle, collide
import pygame as pg
import time
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import time


# global user inputs

# world dimensions
wW = 1000 # world Width
wH = 1000 # world Height
pR = 2

# wait for all Gazebo terminal statements to clear
time.sleep(5)

print("\n\nEnter inputs\n\n")
start = [wW*0.5,wH*0.9,135] #getCoordinate(True)
goal = [wW*0.4,wH*0.1,0] #getCoordinate(False)
lowRPM = int(input("Enter robot's low RPM: \n"))
highRPM = int(input("Enter robot's high RPM: \n"))
rpm = [lowRPM,highRPM]

clearance = int(input("Enter robot's clearance: \n"))
robot_radius = 22
clear_Radius = clearance + robot_radius
bufferRadius = 1.5

initial_distance = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2)  

#vector declaration for collision
v = Vector

rect1 = Poly.from_box(v(wW*0.1, wH*0.5), wW*0.15, wH*0.15)
rect2 = Poly.from_box(v(wW*0.5, wH*0.5), wW*0.25, wH*0.15)
rect3 = Poly.from_box(v(wW*0.8, wH*0.7), wW*0.15, wH*0.2)

circle1 = Circle(v((wW * 0.2),(wH * 0.2)), wW*0.1)
circle2 = Circle(v((wW * 0.2),(wH * 0.8)), wW*0.1)

#create list of all nodes (Node Structure: ownID, parentID, position, angle, cost)
nodes = []
openNodes = []
closedNodes = []
solution = []
tempSol = []
final_Node = {}

#create 3D array for all visited notes
visitedNodes = np.zeros((wW*2 + 1, wH*2 + 1, 13))

#flag for complete search
completed = False

#=============================================================== FUNCTIONS DEFINITIONS ==========================================

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def test(start_pose):
    global solution
    
    #print(start_pose)
    q = get_quaternion_from_euler(0, 0, start[2] * np.pi / 180)

    pathCount = 1
    pathEnd = len(solution)

    msg=Twist()
    pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    #rospy.init_node('robot_talker',anonymous=True)
    
    rospy.init_node('set_pose')

    state_msg = ModelState()
    state_msg.model_name = 'turtlebot3_waffle_pi'
    state_msg.pose.position.x = (start_pose[0]*10/wW) - 5
    state_msg.pose.position.y = 5 - (start_pose[1]*10/wW)
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = q[0]
    state_msg.pose.orientation.y = q[1]
    state_msg.pose.orientation.z = q[2]
    state_msg.pose.orientation.w = q[3]

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
    except:
        print("Service call failed")
    
    while not rospy.is_shutdown():
            
        if pathCount < pathEnd:

            #x, y = solution[pathCount]['pos']
            linear_vel, angular_vel = solution[pathCount]['velocity']
            
            linear_vel = linear_vel/10
            angular_vel = angular_vel
            
            #print("Linear V: ", linear_vel)
            #print("Angular V: ", angular_vel)

            msg.linear.x = 1.25*linear_vel
            msg.angular.z = angular_vel

        else:
            msg.angular.z = 0
            msg.linear.x = 0
        
        pub.publish(msg)
        time.sleep(1)
        
        pathCount += 1

# Check for obstacle collision
def checkDistance(x, y):
    robot = Circle(v(x,y), clear_Radius)
    if collide(robot, circle1) or collide(robot, circle2) or collide(robot, rect1) or collide(robot, rect2) or collide(robot, rect3):
        # if collide(robot, circle1):
        #     print("circle1")
        # if collide(robot, circle2):
        #     print("circle2")
        # if collide(robot, rect1):
        #     print("rect1")
        # if collide(robot, rect2):
        #     print("rect2")
        # if collide(robot, rect3):
        #     print("rect3")
        return True    
    elif x <= clear_Radius or x>= wW-clear_Radius or y <= clear_Radius or y >= wH-clear_Radius:
        return True
    else:
        return False

# Check if goal is reached
def checkGoal(distance):
    global goal, clear_Radius
    
    if distance < bufferRadius*clear_Radius:
        print("Reached Target...")
        return True
    else:
        return False
        
# Intake user input
def getCoordinate(flag):
    global start, clearance, robot_radius, initial_distance, step, dTheta
    # Print out start or goal commands
    if flag:
        string = "starting"
    else:
        string = "goal"
    
    while True:
              
        # Check if within width bounds
        xBound = range(1, 10)
        x = int(input("Enter "+string+" X coordinate: \n(Must be between 1 and 9): \n"))
        while True:
            if x in xBound:
                break
            else:
                x = int(input("Error. Please enter an X between 1 and 9.\nEnter "+string+" X coordinate:"))
        
        # Check if within height bounds
        yBound = range(1,10)
        y = int(input("Enter "+string+" Y coordinate: \n(Must be between 1 and 9): \n"))
        while True:
            if y in yBound:
                break
            else:
                y = int(input("Error. Please choose a Y between 1 and 9.\nEnter "+string+" Y coordinate:"))
        
        # Check if within height bounds
        angle = int(input("Enter "+string+" angle degree: \n"))
        if abs(angle % 360) > 180:
            if angle > 0:
                angle = (angle % 360) - 360
            else:
                angle = (angle % 360) + 360
        else:
            angle = angle % 360
        
        # Check if the point is over an object
        if not checkDistance(wW*x/10,wH*y/10):
            break
        else:
            print("Point is on an object. Please enter corrdinates again.\n\n")
    return[wW*x/10, wH*y/10, angle]


def cost(node, movement):
    global completed, final_Node, goal
    r = 0.33
    L = 2.87
    D = 0
    dt = 0.1
    Xn = node['pos'][0]
    Yn = node['pos'][1]
    Thetai = node['angle']
    UL = movement[0]
    UR = movement[1]
    t = 0
    
    #print("UL: ", UL)
    #print("UR: ", UR)
    #print("Xn: ", Xn)
    #print("Yn: ", Yn)
    #print("Thetai: ", Thetai)
    
    if Thetai >= 360:
        Thetai = Thetai % 360
    if Thetai < 0:
        Thetai = 360 + Thetai
    
    UL = UL * (2 * np.pi/ 60)
    UR = UR * (2 * np.pi/ 60)
    Thetan = math.pi * Thetai / 180
    l_X = 0.5*r * (UL + UR) * math.cos(Thetan)
    l_Y = 0.5*r * (UL + UR) * (-1*math.sin(Thetan))
    angular_vel = ((r / L) * (UR - UL))
    linear_vel = math.sqrt(math.pow((l_X),2)+math.pow((l_Y),2))
    
    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes
    D=0
    while t<1:
        t = t + dt
        Thetan += ((r / L) * (UR - UL)) * dt
        dX = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        dY = 0.5*r * (UL + UR) * (-1*math.sin(Thetan)) * dt
        Xn += dX*10
        Yn += dY*10
    Thetan = 180 * (Thetan) / np.pi
        
    parentID = node['ownID']
    cost = node['cost'] + D
    round_x = round(int(Xn)*2)
    round_y = round(int(Yn)*2)
    goal_distance = math.sqrt((goal[0]-Xn)**2 + (goal[1]-Yn)**2)
    
    #print("UL: ", UL)
    #print("UR: ", UR)
    #print("Xn: ", Xn)
    #print("Yn: ", Yn)
    #print("Thetan: ", Thetan)
    #check if we reached the goal
    completed = checkGoal(goal_distance)
    if completed:
        final_Node = {'ownID': len(nodes),
                    'parentID': parentID,
                    'pos': [Xn,Yn],
                    'angle': Thetan,
                    'cost': cost,
                    'cost2': cost + 2*goal_distance,
                    'velocity': [linear_vel, angular_vel]
                    }
    
    #if we haven't visited this node yet, and we're clear, then create it and add it to the list
    if checkDistance(Xn,Yn) == False:
        if visitedNodes[round_x][round_y][1] == 0:
            new_Node = {'ownID': len(nodes),
                        'parentID': parentID,
                        'pos': [Xn,Yn],
                        'angle': Thetan,
                        'cost': cost,
                        'cost2': cost + 2*goal_distance,
                        'velocity': [linear_vel, angular_vel]
                        }
            
            nodes.append(new_Node)
            openNodes.append(new_Node)
            visitedNodes[round_x][round_y][1] = 1
        
            return new_Node
    
    # if none of the conditions are met, just return the node passed
    return node

## 8 Move Functions

def move11(node, move):
    
    return cost(currentNode, [move[0], move[0]])
    
def move22(node, move):
    
    return cost(currentNode, [move[1], move[1]])
    
def move10(node, move):
    
    return cost(currentNode, [move[0], 0])
    
def move01(node, move):
    
    return cost(currentNode, [0, move[0]])

def move20(node, move):
    
    return cost(currentNode, [move[1], 0])
    
def move02(node, move):
    
    return cost(currentNode, [0, move[1]])
        
def move12(node, move):

    return cost(currentNode, [move[0], move[1]])

def move21(node, move):

    return cost(currentNode, [move[1], move[0]])

## Backtrack Function

def backtrack(startNode):
    global nodes, openNodes, closedNodes, solution, tempSol, final_Node, bufferRaidus
    
    # Make sure final goal node is appended
    currentNode = final_Node

    while currentNode['parentID'] != 0:
        solution.append(currentNode)
        currentNode = nodes[currentNode['parentID']]
        
    # Make sure origin is appended    
    solution.append(startNode)
    
    # Reverse solution
    solution.reverse()
    
    pW = wW/pR
    pH = wH/pR
    
    # Initialize screen
    SCREENSIZE = (pW,pH)
    screen = pg.display.set_mode(SCREENSIZE, pg.DOUBLEBUF|pg.HWACCEL)
    clock = pg.time.Clock()
    prev_x, prev_y = startingNode['pos']
    prev_x1, prev_y1 = startingNode['pos']
    pg.draw.circle(screen, (0,0,255), (prev_x/pR, prev_y/pR), 5, 1)
    pg.draw.circle(screen, (255,0,0), (goal[0]/pR, goal[1]/pR), bufferRadius*clear_Radius/pR, 1)
    pg.draw.circle(screen, (255,0,0), (goal[0]/pR, goal[1]/pR), 1, 1)
    
    # Obstacles
    pg.draw.rect(screen, (255,255,255), (pW*0.1 - pW*(0.15/2), pH*0.5 - pH*(0.15/2), pW*0.15, pH*0.15), 1)
    pg.draw.rect(screen, (255,255,255), (pW*0.5 - pW*(0.25/2), pH*0.5 - pH*(0.15/2), pW*0.25, pH*0.15), 1)
    pg.draw.rect(screen, (255,255,255), (pW*0.8 - pW*(0.15/2), pH*0.7 - pH*(0.1), pW*0.15, pH*0.2), 1)
    pg.draw.circle(screen, (255,255,255), ((pW * 0.2),(pH * 0.2)), pW*0.1, 1)
    pg.draw.circle(screen, (255,255,255), ((pW * 0.2),(pH * 0.8)), pW*0.1, 1)
    
    # Draw all nodes
    for i in nodes:
        currentNode = i
        while currentNode['parentID'] != 0:
            tempSol.append(currentNode)
            currentNode = nodes[currentNode['parentID']]
        
        for i in tempSol:
            x, y = i['pos']
            prev_x, prev_y = nodes[i['parentID']]['pos']
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    sys.exit() 
            
            pg.draw.line(screen, (0,255,255), (int(prev_x/pR), int(prev_y/pR)), (round(x/pR), round(y/pR)), 1)
            pg.display.flip()
            
            clock.tick(6000)   
            
        tempSol = []
        
    # Draw solution
    for i in solution:
        
        x, y = i['pos']
    
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit() 
        
        pg.draw.line(screen, (0,0,255), (int(prev_x/pR), int(prev_y/pR)), (round(x/pR), round(y/pR)), 3)
        pg.draw.circle(screen, (0,0,255), (x/pR,y/pR), 4, 3)
        pg.display.flip()
        
        clock.tick(10)   
        prev_x, prev_y = i['pos']
    
    # Close Pygame
    time.sleep(5)
    
#==========================================================START OF CODE========================================================

print("\nLoading...\n")

start = getCoordinate(True)
goal = getCoordinate(False)

#create starting node
startingNode = {'ownID': 0,
                'parentID': 0,
                'pos': [start[0], start[1]],
                'angle': start[2],
                'cost': 0,
                'cost2': initial_distance,
                'velocity':[0,0]
                }

nodes.append(startingNode)
openNodes.append(startingNode)
prevNode = startingNode

#find all nodes and reach target
while completed == False:
    
    newList = sorted(openNodes, key=lambda d:d['cost2'], reverse = False)
    currentNode = newList[0]
    openNodes.remove(currentNode)
    closedNodes.append(currentNode)
    
    #print("round")
    move11(currentNode, rpm)
    if completed:
        break
    move22(currentNode, rpm)
    if completed:
        break
    move10(currentNode, rpm)
    if completed:
        break
    move01(currentNode, rpm)
    if completed:
        break
    move20(currentNode, rpm)
    if completed:
        break
    move02(currentNode, rpm)
    if completed:
        break
    move12(currentNode, rpm)
    if completed:
        break
    move21(currentNode, rpm)
    if completed:
        break
    
    # if openNodes is empty at any point in this loop, it means that the starting node is
    # blocked off from the goal, most likely because it is forced to run into walls or obstacles
    if not openNodes:
        final_Node = currentNode
        backtrack(startingNode)
    
#back track
print("\nGenerating Backtrack Visualization...\n")
backtrack(startingNode)

print("\n-------\nDone!")
test(start)
pg.display.quit()