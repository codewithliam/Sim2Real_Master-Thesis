#!/usr/bin/env python


# Author: Liam Neric
#Uncommented because the pandemic hit before i got a chance to clean things up on the lab computer

# BEGIN_SUB_TUTORIAL imports
##
# To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
# This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
# and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import os.path
import os
import copy
import rospy
import rosbag
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import numpy as np
import subprocess 
from math import pi
import random
import inspect
import datetime
import csv
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from table_logger import TableLogger
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import pickle
import math
# END_SUB_TUTORIAL


class MoveGroup():
  """MoveGroup class initiation"""

  def __init__(self):    # init moveit commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('linear', anonymous=True)
    # specify move group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("panda_arm")
    #group.set_planner_id("RRTkConfigDefault")
    #group.set_planner_id("RRTstarkConfigDefault")
    #group.set_planner_id("PRMkConfigDefault")
    #group.set_planner_id("PRMstarkConfigDefault")
    #group.set_planner_id("LazyPRMkConfigDefault")
    #group.set_planner_id("LazyPRMstarkConfigDefault")
    #group.set_planner_id("TRRTkConfigDefault")
    group.set_planner_id("RRTConnectkConfigDefault")
    
    #group.set_planning_time(20)

    rospy.sleep(0.5)
    # init ros node

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    #group.set_goal_tolerance(pi/90)
    # move grasper to init position
    # set ros publisher rate, 10hz = 10 seconds for a circle

  def linear_move(self, X, Y , Z, velocity_scaling_argument, acceleration_scaling_argument):
    """Linear movement planner, (For the Mathias Haage calibration)
    max_x: Maximum allowed movement Pose in x direction
    max_y: Maximum allowed movement Pose in x direction
    max_z: Maximum allowed movement Pose in x directions
    Returns: Plan that can be executed in main methods
    """
    old_values=self.group.get_current_joint_values()
    self.group.set_max_velocity_scaling_factor(velocity_scaling_argument)
    self.group.set_max_acceleration_scaling_factor(acceleration_scaling_argument)

    #eef_step=velocity_scaling_argument/100
    eef_step=0.01

    waypoints = []
    wpose = self.group.get_current_pose().pose

    #wpose.orientation.x = 0
    #wpose.orientation.y = 0
    #wpose.orientation.z = 0
    #wpose.orientation.w = 1

    for i in range(len(X)): 
      if X[i] != 0.0:
        wpose.position.x += X[i]  # and sideways (y)
      if Y[i] != 0.0:
        wpose.position.y += Y[i]
      if Z[i] != 0.0:
        wpose.position.z += Z[i] 
    
      waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      eef_step,        # eef_step
                                      0.0)         # jump_threshold
    plan= self.group.retime_trajectory(self.robot.get_current_state(), plan, velocity_scaling_argument)
    #self.group.retime_trajectory(velocity_scaling_argument,acceleration_scaling_argument,'iterative_time_parameterization')
    self.group.execute(plan, wait=True)
    current_values=self.group.get_current_joint_values()
    if(np.allclose(current_values, old_values, 0, 0.1)==True): 
      subprocess.Popen('rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"', stdout=subprocess.PIPE, stderr=None, shell=True)
      time.sleep(1)
      self.add_objects()
      time.sleep(1)
      while len(self.scene.get_known_object_names() ) != 4: 
        print('objects didnt load, trying again')
        time.sleep(2)
        self.add_objects()
        time.sleep(1)
      group.default(velocity_scaling_argument, acceleration_scaling_argument)
      group.default_work_position(velocity_scaling_argument, acceleration_scaling_argument)
      group.go_to_basepose(velocity_scaling_argument, acceleration_scaling_argument)

  def linear_move2(self, X, Y , Z, velocity_scaling_argument, acceleration_scaling_argument, path, counter):
    """Linear movement planner, (For the Mathias Haage calibration)
    max_x: Maximum allowed movement Pose in x direction
    max_y: Maximum allowed movement Pose in x direction
    max_z: Maximum allowed movement Pose in x directions
    Returns: Plan that can be executed in main methods
    """
    old_values=self.group.get_current_joint_values()
    self.group.set_max_velocity_scaling_factor(velocity_scaling_argument)
    self.group.set_max_acceleration_scaling_factor(acceleration_scaling_argument)

    eef_step=velocity_scaling_argument/10
    #eef_step=0.01

    waypoints = []
    wpose = self.group.get_current_pose().pose

    #wpose.orientation.x = 0
    #wpose.orientation.y = 0
    #wpose.orientation.z = 0
    #wpose.orientation.w = 1

    #Alex test remove maybe
    qx= -0.9238795
    qy= 0.3826834
    qz= 0  #GOOD QUATERNION
    qw= 0
    wpose.orientation.x = qx
    wpose.orientation.y = qy
    wpose.orientation.z = qz
    wpose.orientation.w = qw

    wpose.position.x += X  # and sideways (y)
    wpose.position.y += Y
    wpose.position.z += Z 
    x=str(wpose.position.x - X)
    y=str(wpose.position.y - Y)
    z=str(wpose.position.z - Z)

    write_log('count=' + str(counter) + ' velocity_scaling= ' + str(velocity_scaling_argument) + ' acceleration_scaling= ' + str(acceleration_scaling_argument)
    + ' x= ' +x + ' y= ' + y + ' z= ' + z + ' dx= ' + str(X) + ' dy= ' + str(Y) + ' dz= ' + str(Z), False)
    
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      eef_step,        # eef_step
                                      0.00)         # jump_threshold
    plan= self.group.retime_trajectory(self.robot.get_current_state(), plan, velocity_scaling_argument)
    pickle_save3(plan, path, counter)
    #self.group.retime_trajectory(velocity_scaling_argument,acceleration_scaling_argument,'iterative_time_parameterization')
      
    subprocess.Popen('./record_pose.sh record', stdout=subprocess.PIPE, stderr=None, shell=True) #Record to csv file
    subprocess.Popen('./record.sh record', stdout=subprocess.PIPE, stderr=None, shell=True) #Record to csv file

    time.sleep(1.5)
    self.group.execute(plan,wait=True)
    time.sleep(0.5)
    self.group.stop()
    string_cart = './record_pose.sh stop ' + 'move_cart' + str(counter) + '.csv' + ' ' + path
    string_joint = './record.sh stop ' + 'move_joint' + str(counter) + '.csv' + ' ' + path

    subprocess.Popen(string_cart, stdout=subprocess.PIPE, stderr=None, shell=True)
    subprocess.Popen(string_joint, stdout=subprocess.PIPE, stderr=None, shell=True)


    current_values=self.group.get_current_joint_values()
    if(np.allclose(current_values, old_values, 0, 0.1)==True): 
      subprocess.Popen('rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"', stdout=subprocess.PIPE, stderr=None, shell=True)
      time.sleep(1)
      self.add_objects()
      time.sleep(1)
      while len(self.scene.get_known_object_names() ) != 4: 
        print('objects didnt load, trying again')
        time.sleep(2)
        self.add_objects()
        time.sleep(1)
      group.default(velocity_scaling_argument, acceleration_scaling_argument)
      group.default_work_position(velocity_scaling_argument, acceleration_scaling_argument)
      group.go_to_basepose(velocity_scaling_argument, acceleration_scaling_argument)
      return None
    return plan

  def display_trajectory(self, plan):
    #Displays trajectory from plan object in rviz

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory)


  def default(self, velocity_scaling_argument, acceleration_scaling_argument):
    
    old_values=self.group.get_current_joint_values()
    self.group.set_max_velocity_scaling_factor(velocity_scaling_argument)
    self.group.set_max_acceleration_scaling_factor(acceleration_scaling_argument)


    joint_goal = self.group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    self.group.set_joint_value_target(joint_goal)
    plan=self.group.plan()
    plan2=self.group.retime_trajectory(self.robot.get_current_state(), plan, velocity_scaling_argument)
    
    file1 = open('/data/logs/plan_normal.txt', "w")

    file1.write(str(plan)) #Write arbitrary plan 0-6 to text file

    file1.close()

    file1 = open('/data/logs/plan_param.txt', "w")

    file1.write(str(plan2)) #Write arbitrary plan 0-6 to text file

    file1.close()



    #self.display_trajectory(plan)
    #input("press enter to countinue")
    self.group.go(joint_goal)
    self.group.stop()
    current_values=self.group.get_current_joint_values()
    if(np.allclose(current_values, old_values, 0, 0.1)==True): 
      subprocess.Popen('rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"', stdout=subprocess.PIPE, stderr=None, shell=True)
      time.sleep(1)
      self.add_objects()
      time.sleep(1)
      while len(self.scene.get_known_object_names() ) != 4: 
        print('objects didnt load, trying again')
        time.sleep(2)
        self.add_objects()
        time.sleep(1)

  def default_work_position(self, velocity_scaling_argument, acceleration_scaling_argument):
    
    old_values=self.group.get_current_joint_values()
    self.group.set_max_velocity_scaling_factor(velocity_scaling_argument)
    self.group.set_max_acceleration_scaling_factor(acceleration_scaling_argument)


    joint_goal = self.group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4-0.3
    joint_goal[2] = 0
    joint_goal[3] = -pi/2-1.1
    joint_goal[4] = 0
    joint_goal[5] = pi/3+0.7
    joint_goal[6] = 0+1.3

    self.group.set_joint_value_target(joint_goal)
    plan=self.group.plan()
    plan2=self.group.retime_trajectory(self.robot.get_current_state(), plan, velocity_scaling_argument)
    
    file1 = open('/data/logs/plan_normal.txt', "w")

    file1.write(str(plan)) #Write arbitrary plan 0-6 to text file

    file1.close()

    file1 = open('/data/logs/plan_param.txt', "w")

    file1.write(str(plan2)) #Write arbitrary plan 0-6 to text file

    file1.close()

    return plan2



    #self.display_trajectory(plan)
    #input("press enter to countinue")
    self.group.go(joint_goal)
    self.group.stop()
    current_values=self.group.get_current_joint_values()
    if(np.allclose(current_values, old_values, 0, 0.1)==True): 
      subprocess.Popen('rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"', stdout=subprocess.PIPE, stderr=None, shell=True)
      time.sleep(1)
      self.add_objects()
      time.sleep(1)
      while len(self.scene.get_known_object_names() ) != 4: 
        print('objects didnt load, trying again')
        time.sleep(2)
        self.add_objects()
        time.sleep(1)

  
  def add_objects(self):
    self.scene.remove_world_object()

    #box1_pose = [1.7, 0, 0.0, 0, 0, 0, 1] #Forward restriction (facing pose) 
    #box1_dimensions = [0.1, 1, 4]
    
    box2_pose = [0, -0.7, 1, 0, 0, 0, 1] #Left restriction
    box2_dimensions = [1.6, 0.2, 2]

    box3_pose = [0, 0.7, 1, 0, 0, 0, 1] #Right restriction
    box3_dimensions = [1.6, 0.2, 2]

    box4_pose = [-0.55, 0, 1, 0, 0, 0, 1] #backwards restriction
    box4_dimensions = [0.2, 1.2, 2]

    box5_pose = [0, 0, -0.1, 0, 0, 0, 1] #Ground restriction (added 10cm of space) 
    box5_dimensions = [1.6, 1.6, 0.2]

    #self.add_box_object("box1", box1_dimensions, box1_pose)
    self.add_box_object("box2", box2_dimensions, box2_pose)
    self.add_box_object("box3", box3_dimensions, box3_pose)
    self.add_box_object("box4", box4_dimensions, box4_pose)
    self.add_box_object("box5", box5_dimensions, box5_pose)


  def add_box_object(self, name, dimensions, pose):

      p = geometry_msgs.msg.PoseStamped()
      p.header.frame_id = self.robot.get_planning_frame()
      p.header.stamp = rospy.Time.now()
      p.pose.position.x = pose[0]
      p.pose.position.y = pose[1]
      p.pose.position.z = pose[2]
      p.pose.orientation.x = pose[3]
      p.pose.orientation.y = pose[4]
      p.pose.orientation.z = pose[5]
      p.pose.orientation.w = pose[6]

      self.scene.add_box(
          name, p, (dimensions[0], dimensions[1], dimensions[2]))

  def go_to_pose(self, x,y,z, roll, pitch, yaw, velocity_scaling_argument, acceleration_scaling_argument): 


    self.group.set_max_velocity_scaling_factor(velocity_scaling_argument)
    self.group.set_max_acceleration_scaling_factor(acceleration_scaling_argument)

    p_x,p_y,p_z,p_w=euler_to_quaternion(roll, pitch, yaw)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = p_x
    pose_goal.orientation.y = p_y
    pose_goal.orientation.z = p_z
    pose_goal.orientation.w = p_w
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    self.group.set_pose_target(pose_goal)
    plan=self.group.plan()

    time.sleep(1)
    self.group.execute(plan,wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()

  def go_to_pose2(self, x,y,z, velocity_scaling_argument, acceleration_scaling_argument): 

    old_values=self.group.get_current_joint_values()
    self.group.set_max_velocity_scaling_factor(velocity_scaling_argument)
    self.group.set_max_acceleration_scaling_factor(acceleration_scaling_argument)

    #pose_goal = geometry_msgs.msg.Pose()
    pose_goal = self.group.get_current_pose().pose
    #pose_goal.orientation.x = qx
    #pose_goal.orientation.y = qy
    #pose_goal.orientation.z = qz
    #pose_goal.orientation.w = qw

    #Alex test remove maybe
    qx= -0.9238795
    qy= 0.3826834
    qz= 0  #GOOD QUATERNION
    qw= 0
    pose_goal.orientation.x = qx
    pose_goal.orientation.y = qy
    pose_goal.orientation.z = qz
    pose_goal.orientation.w = qw

    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    self.group.set_pose_target(pose_goal)
    try: 
      plan=self.group.plan()
    except:
      print('cant find plan for: x=' +str(x) + 'y=' + str(y) + 'z=' +str(z))
      return None

    time.sleep(1)
    self.group.execute(plan,wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    current_values=self.group.get_current_joint_values()
    if(np.allclose(current_values, old_values, 0, 0.1)==True): 
      subprocess.Popen('rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"', stdout=subprocess.PIPE, stderr=None, shell=True)
      time.sleep(1)
      self.add_objects()
      time.sleep(1)
      while len(self.scene.get_known_object_names() ) != 4: 
        print('objects didnt load, trying again')
        time.sleep(2)
        self.add_objects()
        time.sleep(1)
      group.default(velocity_scaling_argument, acceleration_scaling_argument)
      group.default_work_position(velocity_scaling_argument, acceleration_scaling_argument)
      group.go_to_basepose(velocity_scaling_argument, acceleration_scaling_argument)
      return None
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()
    return plan

  def go_to_basepose(self, velocity_scaling_argument, acceleration_scaling_argument): 

    old_values=self.group.get_current_joint_values()
    self.group.set_max_velocity_scaling_factor(velocity_scaling_argument)
    self.group.set_max_acceleration_scaling_factor(acceleration_scaling_argument)

    qx= -0.899716623161
    qy= 0.435957488599
    qz= 0.0206599293529    #GOOD QUATERNION
    qw= 0.00492274935241 

    x= 0.321279756657
    y= -0.107017952207
    z= 0.493393428371

    #Alex test remove maybe
    qx= -0.9238795
    qy= 0.3826834
    qz= 0  #GOOD QUATERNION
    qw= 0

    #pose_goal = geometry_msgs.msg.Pose()
    pose_goal = self.group.get_current_pose().pose
    pose_goal.orientation.x = qx
    pose_goal.orientation.y = qy
    pose_goal.orientation.z = qz
    pose_goal.orientation.w = qw

    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    self.group.set_pose_target(pose_goal)
    try: 
      plan=self.group.plan()
    except:
      print('cant find plan for: x=' +str(x) + 'y=' + str(y) + 'z=' +str(z))
      return None

    self.group.execute(plan,wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    current_values=self.group.get_current_joint_values()
    if(np.allclose(current_values, old_values, 0, 0.1)==True): 
      subprocess.Popen('rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"', stdout=subprocess.PIPE, stderr=None, shell=True)
      time.sleep(1)
      self.add_objects()
      time.sleep(1)
      while len(self.scene.get_known_object_names() ) != 4: 
        print('objects didnt load, trying again')
        time.sleep(2)
        self.add_objects()
        time.sleep(1)
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()
    return plan


  def move_joint(self, joint_index, velocity_scaling_argument, acceleration_scaling_argument, path):
    '''
    This is the function I will use for first experiment
    joint_index: Which joint to move (0-6)
    velocityscaling= The execution of a plan is slowed down by a chosen factor between [0,1] where 0 is frozen.
    '''

    self.group.set_max_velocity_scaling_factor(velocity_scaling_argument)
    self.group.set_max_acceleration_scaling_factor(acceleration_scaling_argument)

    joint_state=self.group.get_current_joint_values() 

    if(joint_index==0):
      write_to_file(joint_state, True) #Writes commands to files to repeat in simulation

    good_plan=False #Initiates boolean for planning

    joint_limit = np.zeros([7,2])
    x=0.1 #Tolerance for joint randomization, (the numbers below are the panda joint restrictions, but add x for extra safety (important))
    joint_limit[0]=(-2.89 +x, 2.89 -x)
    joint_limit[1]=(-1.76 +x, 1.76 -x)
    joint_limit[2]=(-2.89 +x, 2.89 -x)
    joint_limit[3]=(-3.07 +x, -0.069 -x)
    joint_limit[4]=(-2.89 +x, 2.89 -x)
    joint_limit[5]=(-0.017 +x, 3.75 -x)
    joint_limit[6]=(-2.89 +x, 2.89 -x)

    max_iterations=150
    try_iterations=0
    
    while good_plan==False: #While the randomized joint state cannot be planned
      joint_new=random.uniform(joint_limit[joint_index][0],joint_limit[joint_index][1]) #0 = lower bound, 1 = upper bound
      joint_go = self.group.get_current_joint_values()  #copy joint state (doesnt copy the pointer in Python) 
      joint_go[joint_index]=joint_new #Change one joint value from the joint state, and use the new joint_go to plan

      try:
        good_plan=True
        self.group.set_joint_value_target(joint_go)
        #self.group.go(joint_go, wait=True)
        plan=self.group.plan()
        planchecker(plan)
        1/len(plan.joint_trajectory.points) #If the plan object is empty (because a plan couldnt be found) this returns an error and the loop keeps iterating, sometimes this happens
      except Exception as e: #This is like a catch all
        print(e)
        good_plan=False
        
      try_iterations+=1
      if try_iterations > max_iterations:
        good_plan=False
        break
    
    current_values = []
    if good_plan==True:
      plan= self.group.retime_trajectory(self.robot.get_current_state(), plan, velocity_scaling_argument)

      #string="./record.sh stop" + " " + str(filename) + ".csv"
      subprocess.Popen('./record.sh record', stdout=subprocess.PIPE, stderr=None, shell=True) #Record to csv file
      time.sleep(2.0)
      self.group.execute(plan,wait=True)
      time.sleep(0.7)
      self.group.stop()
      string= './record.sh stop ' + 'joint_' +  str(joint_index) + '.csv' + ' ' + path
      subprocess.Popen(string, stdout=subprocess.PIPE, stderr=None, shell=True)
      current_values=self.group.get_current_joint_values()

      if(np.allclose(current_values, joint_go, 0, 0.1)==False): 
        subprocess.Popen('rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"', stdout=subprocess.PIPE, stderr=None, shell=True)
        time.sleep(1)
        self.add_objects()
        time.sleep(1)
        while len(self.scene.get_known_object_names() ) != 4: 
          print('objects didnt load, trying again')
          time.sleep(2)
          self.add_objects()
          time.sleep(1)

      write_to_file(current_values) #Writes the new joint positions to a csv file
      pickle_save(plan, path, joint_index)

      return plan

    if good_plan==False:
      pickle_save(None, path, joint_index)
      write_to_file(current_values) #Writes the new joint positions to a csv file
      return None #Returns plan for the joint movement
  
  def sim(self):

    for traj in range(1000): 
      newpath = '/data/real_logs/08:22:13.829309/traj_' + str(traj)
      for i in range(7): 
        try: 
          plan = pickle_loader(newpath, i)
        except Exception as e:
          print(e)
          plan=None
          pass  
        
        if plan != None:
          start_positions=plan.joint_trajectory.points[0].positions
          joint_go=group.group.get_current_joint_values()
          for x in range(7): 
            joint_go[x]=start_positions[x]
          self.group.go(joint_go)
          subprocess.Popen('./record.sh record', stdout=subprocess.PIPE, stderr=None, shell=True) #Record to csv file
          time.sleep(1.6)
          self.group.execute(plan,wait=True)
          self.group.stop()
          time.sleep(0.4)
          string= './record.sh stop ' + 'sim_' +  str(i) + '.csv' + ' ' + newpath
          subprocess.Popen(string, stdout=subprocess.PIPE, stderr=None, shell=True)
          time.sleep(0.1)

  def sim_wednesday(self): 
    
    path= '/data/real_logs/wednesday14:55:40.346866_0.2_0.2_1_z'
    '''
    default_plan=pickle_loader_sim(path, 'go_to_default_plan')
    joint_go=self.group.get_current_joint_values()
    print(joint_go)
    default_plan_start=default_plan.joint_trajectory.points[0].positions
    print(default_plan_start)
    for x in range(7): 
      joint_go[x]=default_plan_start[x]
    self.group.go(joint_go)
    '''
    #self.group.execute(default_plan)
    #base_plan=pickle_loader_sim(path, 'go_to_base_plan')
    #self.group.execute(base_plan)

    for go_pose_index in range(71):
      for linear_move_index in range(2):

        #go_to_pose_plan=pickle_loader_sim(path, 'go_to_pose_plan_' + str(go_pose_index)) 

        joint_go=self.group.get_current_joint_values()
        #pose_plan_start=go_to_pose_plan.joint_trajectory.points[0].positions
        #for x in range(7): 
        #  joint_go[x]=pose_plan_start[x]
        #self.group.go(joint_go)
        #self.group.execute(go_to_pose_plan)
        #time.sleep(0.4) 


        linear_move_plan=pickle_loader_sim(path, 'linear_plan_' + str(go_pose_index*2 +linear_move_index))
        linear_plan_start=linear_move_plan.joint_trajectory.points[0].positions


        for x in range(7): 
          joint_go[x]=linear_plan_start[x]

        try: 
          self.group.go(joint_go)
          time.sleep(0.2)
          #subprocess.Popen('./record_pose.sh record', stdout=subprocess.PIPE, stderr=None, shell=True) #Record to csv file
          subprocess.Popen('./record.sh record', stdout=subprocess.PIPE, stderr=None, shell=True) #Record to csv file
          time.sleep(1.6)
          self.group.execute(linear_move_plan)
          time.sleep(0.3)
          self.group.stop()
          string_joint = './record.sh stop ' + 'sim_joint' + str(go_pose_index*2 +linear_move_index) + '.csv' + ' ' + path
          string_cart = './record_pose.sh stop ' + 'sim_cart' + str(go_pose_index*2 +linear_move_index) + '.csv' + ' ' + path
          subprocess.Popen(string_joint, stdout=subprocess.PIPE, stderr=None, shell=True)
          subprocess.Popen(string_cart, stdout=subprocess.PIPE, stderr=None, shell=True)
        except: 
          print('error')


    

def write_to_file(joint, newcommand=False):
  #Writes joint_go commands to a csv file for saving
  if newcommand==True: 
    with open('log.csv', 'a') as file:
      writer = csv.writer(file)
      writer.writerow('START STATE BEGIN:')
      writer.writerow(joint)
      writer.writerow('START STATE END')
      sys.stdout.flush()
  else:
    with open('log.csv', 'a') as file:
      writer = csv.writer(file)
      writer.writerow(joint)
      sys.stdout.flush()


def plan_writer(plans, path): #FOR ALEX
  #plans[0], plans[1] ... plans[6] to get the plans

  for i in range(0,7):
    save_path =""

    name_of_file = "plan"+str(i)

    completeName = os.path.join(save_path, name_of_file+".txt")         

    file1 = open(completeName, "w")

    file1.write(str(plans[i])) #Write arbitrary plan 0-6 to text file

    file1.close()

def plan_reader(name_of_file): 
  save_path =""

  completeName = os.path.join(save_path, name_of_file+".txt")         

  file1 = open(completeName, "r")

  plan=file1.read(completeName) #Write arbitrary plan 0-6 to text file

  file1.close()

  return plan

def planchecker(plan): 
  
  trajectories=len(plan.joint_trajectory.points)
  positions=len((plan.joint_trajectory.points[1].positions))
  data=np.empty([trajectories, positions])

  for i in range(trajectories): 
    for j in range(positions):
      data[i][j]=plan.joint_trajectory.points[i].positions[j]

  variances= np.var(data, axis=0)

  number_over= [variances[i] > 1e-4 for i in range(len(variances))]

  if np.sum(number_over) > 1: 
    raise ValueError('plan involved several joints')

  return np.max(variances)

def pickle_save(plan, path, index): 

  filename= path + '/plan_' + str(index)
  
  outfile = open(filename, 'wb')
  pickle.dump(plan, outfile)
  outfile.close()

def pickle_save2(plan, path, index): 

  filename= path + '/go_to_pose_plan_' + str(index)
  
  outfile = open(filename, 'wb')
  pickle.dump(plan, outfile)
  outfile.close()

def pickle_save3(plan, path, index): 
  
  filename= path + '/linear_plan_' + str(index)

  outfile = open(filename, 'wb')
  pickle.dump(plan, outfile)
  outfile.close()

def pickle_save4(plan, path, string): 
  
  filename= path + '/go_to_' + string + '' + '_plan'

  outfile = open(filename, 'wb')
  pickle.dump(plan, outfile)
  outfile.close()


def pickle_loader(path, index): 
  filename=path + '/plan_' + str(index)
  infile = open(filename,'rb')
  plan = pickle.load(infile)
  infile.close()

  return plan

def pickle_loader_sim(path, string): 
  filename=path + '/' + string
  infile = open(filename,'rb')
  plan = pickle.load(infile)
  infile.close()

  return plan

def create_dir(time_string, string): 


  newpath = '/data/real_logs/' + time_string + '/traj_' + string 
  if not os.path.exists(newpath):
    os.makedirs(newpath)

  return newpath

def write_pathname(string): 
  with open('/data/wednesday_files', 'a') as file:
    #writer = csv.writer(file)
    file.write(string + '\n')
    #writer.writerow(string)
    sys.stdout.flush()

def write_log(string, new): 
    with open('/data/command_log', 'a') as file:
      if new: 
        file.write(string)
        file.write('\n')
        file.write('\n')
      else: 
        file.write(string + '\n')
      sys.stdout.flush()




def euler_to_quaternion(roll, pitch, yaw):

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return qx, qy, qz, qw
  

if __name__ == '__main__': #sys.argv[1] means the word after the execution, like "python linear.py default", to call default mode 
  group = MoveGroup()
  group.add_objects()


  velocity_scaling_argument=float(sys.argv[2])
  
  if(velocity_scaling_argument > 1): 
    velocity_scaling_argument=0.01
  if(velocity_scaling_argument<0): 
    velocity_scaling_argument=0.01


  acceleration_scaling_argument=float(sys.argv[3])
  
  if(acceleration_scaling_argument > 1): 
    acceleration_scaling_argument=0.01
  if(acceleration_scaling_argument<0): 
    acceleration_scaling_argument=0.01
  
  repetitions=int(sys.argv[4])
  #repetitions=1
  low_defaults = np.genfromtxt('defaults.csv', delimiter=',')

  if len(sys.argv)>1: 
    if sys.argv[1] == "default":
      group.default(velocity_scaling_argument, acceleration_scaling_argument)
    if sys.argv[1] == "default_work_position":
      group.default_work_position(velocity_scaling_argument, acceleration_scaling_argument)

    # THESE FOLLOWING CALLS ARE THE MOST IMPORTANT, ESPECIALLY "joint"

    if sys.argv[1] =="joint": #Same mon launch panda_moveit_config panda_control_moveit_rviz.launch load_gripper:=true \
      plan_list=[]
      #time_string=str(time.strftime("%d/%m/%Y%H:%M:%S"))
      time_string=str(datetime.datetime.now().time()) + '_' +  str(acceleration_scaling_argument)
      for j in range(0,repetitions):
        path=create_dir(time_string,str(j))
        
        if j%4==0 and j!=0 and len(group.scene.get_known_object_names())==5:

          rows=low_defaults.shape[0]
          rand=random.randint(0, rows-1)
          joint_target=low_defaults[rand, :]

          joint_go=group.group.get_current_joint_values()
          for i in range(7): 
            joint_go[i]=joint_target[i]

          group.group.set_joint_value_target(joint_go)
          plan= group.group.plan()
          group.group.execute(plan)
        
        for i in range(0, 7):
          plan=group.move_joint(i, velocity_scaling_argument, acceleration_scaling_argument, path)
          plan_list.append(plan)
        plan_writer(plan_list, path) #Writes


    if sys.argv[1] == "sleep":
      joint_vec = []
      time.sleep(2)
      for i in range(30):
        joint_vec.append(group.group.get_current_joint_values()) 
        time.sleep(1)
      joints=np.asarray(joint_vec)
      np.savetxt("defaults.csv", joints, delimiter=",")


    if sys.argv[1] =="sim":
       group.sim()
    
    if sys.argv[1] == "sim_wednesday": 
      group.sim_wednesday()


    if sys.argv[1] =="pose":
      group.go_to_basepose(velocity_scaling_argument, acceleration_scaling_argument)
      #pose=group.group.get_current_pose().pose
      #print(pose)

      x_r=random.uniform(-0.2, 0.2)
      y_r=random.uniform(-0.2, 0.2)
      z_r=random.uniform(-0.2, 0.2)

      X=[x_r, 0.0, 0.0]
      Y=[0.0, y_r, 0.0]
      Z=[0.0, 0.0, z_r]
      
      X_rev = [-x for x in X]
      Y_rev = [-y for y in Y]
      Z_rev = [-z for z in Z]

      group.linear_move(X,Y,Z, velocity_scaling_argument, acceleration_scaling_argument)
      group.linear_move(X_rev,Y_rev,Z_rev, velocity_scaling_argument, acceleration_scaling_argument)


    if sys.argv[1] =="pose2":
      group.go_to_basepose(velocity_scaling_argument, acceleration_scaling_argument)
      #pose=group.group.get_current_pose().pose
      #print(pose)
      #X=[x_r, 0.0, 0.0]
      #Y=[0.0, y_r, 0.0]
      #Z=[0.0, 0.0, z_r]
      for i in range(repetitions): 
        x_r=random.uniform(-0.2, 0.2)
        y_r=random.uniform(-0.2, 0.2)
        z_r=random.uniform(-0.2, 0.2)
        
      
      

    
    if sys.argv[1] == "test": 
      print(group.group.get_current_pose().pose)
      group.default(velocity_scaling_argument, acceleration_scaling_argument)
      group.go_to_basepose(velocity_scaling_argument, acceleration_scaling_argument)
      print(group.group.get_current_pose().pose)

    if sys.argv[1] == "print_pose": 
      print(group.group.get_current_pose().pose)

    if sys.argv[1] == "goto": 

      x_range=np.linspace(-0.2, 0.8, 5)
      y_range=np.linspace(-0.4, 0.2, 5)
      z_range=np.linspace(0.4, 1.5, 5)

      for x in x_range: 
        for y in y_range:
          for z in z_range: 
            group.go_to_pose2(x, y, z, velocity_scaling_argument, acceleration_scaling_argument)

# rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"

    if sys.argv[1] == "goto_wednesday": 
      time_string=str(datetime.datetime.now().time())
      time_string=time_string + "_" + str(velocity_scaling_argument) + "_" + str(acceleration_scaling_argument) + "_" + str(repetitions) + "_" + str(sys.argv[5])

      newpath = '/data/real_logs/wednesday' + time_string
      if not os.path.exists(newpath):
        os.makedirs(newpath)
      write_pathname(newpath)
      write_pathname(str(repetitions))
      write_log(newpath, True)
      write_log(str(velocity_scaling_argument) + ' ' + str(acceleration_scaling_argument) + ' ' + str(repetitions) + ' ' + str(sys.argv[5]), False)

      group.default(velocity_scaling_argument, acceleration_scaling_argument)
      plan_work=group.default_work_position(velocity_scaling_argument, acceleration_scaling_argument)
      pickle_save4(plan_work, newpath, 'default')
      plan_base=group.go_to_basepose(velocity_scaling_argument, acceleration_scaling_argument)
      pickle_save4(plan_base, newpath, 'base')
      '''
      CONSTRAINTS: 
      X 0.312 0.7
      Y -0.49 0.41
      Z 0.16 0.75
      '''
      """ box1_pose = [1.7, 0, 0.0, 0, 0, 0, 1] #Forward restriction (facing pose) 
      box1_dimensions = [0.1, 1, 4]

      box2_pose = [0, -0.45, 0, 0, 0, 0, 1] #Left restriction
      box2_dimensions = [1, 0.1, 4]

      box3_pose = [0, 0.4, 0, 0, 0, 0, 1] #Right restriction
      box3_dimensions = [1, 0.1, 4]

      box4_pose = [-0.37, 0, 0, 0, 0, 0, 1] #backwards restriction
      box4_dimensions = [0.1, 1, 4]

      box5_pose = [0, 0, 0, 0, 0, 0, 1] #Ground restriction (added 10cm of space) 
      box5_dimensions = [1.5, 1.5, 0.15] """
      x_interval=[0.35,0.6]
      y_interval=[-0.30,0.30]
      z_interval=[0.15,0.6]
      test = False
      if test == False:
        x_range=np.linspace(x_interval[1], x_interval[0], (x_interval[1]-x_interval[0])/0.05+1) # reversed
        y_range=np.linspace(y_interval[1], y_interval[0], (y_interval[1]-y_interval[0])/0.05+1)
        z_range=np.linspace(z_interval[1], z_interval[0], (z_interval[1]-z_interval[0])/0.05+1) # reversed
      else:
        x_range=np.linspace(x_interval[1], x_interval[0], 3) # reversed
        y_range=np.linspace(y_interval[1], y_interval[0], 3)
        z_range=np.linspace(z_interval[1], z_interval[0], 3) # reversed
      delta_x=0
      delta_y=0
      delta_z=0
      """
      delta_xz=0
      delta_yz=0
      delta_xy=0
      """
      if sys.argv[5] == "y":
        print("linear in y")
        y_range=[y_interval[1]]
        delta_y=-(y_interval[1]-y_interval[0])

      if sys.argv[5] == "x":
        print("linear in x")
        x_range=[x_interval[1]]
        delta_x=-(x_interval[1]-x_interval[0])

      if sys.argv[5] == "z":
        print("linear in z")
        z_range=[z_interval[1]]
        delta_z=-(z_interval[1]-z_interval[0])

      if sys.argv[5] == "yz": 
        print("linear in zy")
        z_range=[z_interval[1]]
        delta_z=-(z_interval[1]-z_interval[0])
        y_range=[y_interval[1]]
        delta_y=-(y_interval[1]-y_interval[0])

      if sys.argv[5] == "xz": 
        print("linear in xz")
        z_range=[z_interval[1]]
        delta_z=-(z_interval[1]-z_interval[0])
        x_range=[x_interval[1]]
        delta_x=-(x_interval[1]-x_interval[0])

      if sys.argv[5] == "xy": 
        print("linear in xy")
        y_range=[y_interval[1]]
        delta_y=-(y_interval[1]-y_interval[0])
        x_range=[x_interval[1]]
        delta_x=-(x_interval[1]-x_interval[0])

      if sys.argv[5] == "yz_back": 
        print("linear in zy back")
        z_range=[z_interval[1]]
        delta_z=-(z_interval[1]-z_interval[0])
        y_range=[y_interval[0]]
        delta_y=(y_interval[1]-y_interval[0])

      if sys.argv[5] == "xz_back": 
        print("linear in xz back")
        z_range=[z_interval[0]]
        delta_z=(z_interval[1]-z_interval[0])
        x_range=[x_interval[1]]
        delta_x=-(x_interval[1]-x_interval[0])

      if sys.argv[5] == "xy_back": 
        print("linear in xy back")
        y_range=[y_interval[0]]
        delta_y=(y_interval[1]-y_interval[0])
        x_range=[x_interval[1]]
        delta_x=-(x_interval[1]-x_interval[0])

      if sys.argv[5] == "xy_45": 
        print("linear in xy 45 degrees")
        x_range=[x_interval[1]]
        delta_x=-(x_interval[1]-x_interval[0])
        delta_y=delta_x
        y_interval=[ y_interval[0]-delta_x,y_interval[1]]
        y_range=np.linspace(y_interval[1], y_interval[0], (y_interval[1]-y_interval[0])/0.05+1)
        # This is not a good implementation since it just does not fill the space in y and it is hardprogrammed that the interval in x is smaller than the interval of y

      """
      elif sys.argv[5] == "xz":
        x_range=x_interval[0]
      else
        # important, do nothing here but calculate in the for loops
      """
      experiment_repetitions=1
      for r in range(0,experiment_repetitions): 
        counter=0
        position_plan_counter=0
        for x in x_range: 
          for y in y_range:
            for z in z_range:
              """             
              if sys.argv[5] == "xz":
                if sys.argv[6] == "down" or "front":
                  delta_xz=z-y_interval[0]
                elif if sys.argv[6] == "up" or "back":
              elif sys.argv[5] == "x"
                x_range=x_interval[0]
                delta_x=x_interval[1]-x_interval[0]
              elif sys.argv[5] == "z"
                z_range=z_interval[0]
                delta_z=z_interval[1]-z_interval[0]
              else
              """
              difficult=False
              if difficult == True:
                group.default_work_position(velocity_scaling_argument, acceleration_scaling_argument)
                group.go_to_basepose(velocity_scaling_argument, acceleration_scaling_argument)
                time.sleep(1)
                
              go_to_pose_plan=group.go_to_pose2(x, y, z, velocity_scaling_argument, acceleration_scaling_argument)
              time.sleep(1)

              if go_to_pose_plan == None:
                # was not able to find a start position for the first time
                group.default(velocity_scaling_argument, acceleration_scaling_argument)
                group.default_work_position(velocity_scaling_argument, acceleration_scaling_argument)
                group.go_to_basepose(velocity_scaling_argument, acceleration_scaling_argument)
                time.sleep(1)
                go_to_pose_plan=group.go_to_pose2(x, y, z, velocity_scaling_argument, acceleration_scaling_argument)
                time.sleep(1)
                # was not able to find a start position for the third time - don't even do the linear moves
                print('cant reach start position: x=' +str(x) + 'y=' + str(y) + 'z=' +str(z))
                continue # skipping the linear moves
              pickle_save2(go_to_pose_plan, newpath, position_plan_counter)
              position_plan_counter +=1
              for r in range(0,repetitions):
                linear_move_plan=group.linear_move2(delta_x,delta_y,delta_z, velocity_scaling_argument, acceleration_scaling_argument, newpath, counter)
                counter +=1
                if linear_move_plan == None:
                  # was not able to find a start position for the third time - don't even do the next linear move
                  print('cant find first linear move from: x=' +str(x) + 'y=' + str(y) + 'z=' +str(z))
                  break
                time.sleep(1)
                linear_move_plan=group.linear_move2(-delta_x,-delta_y,-delta_z, velocity_scaling_argument, acceleration_scaling_argument, newpath, counter)
                counter+=1
                if linear_move_plan == None:
                  # was not able to find a start position for the third time - don't even do the next linear move
                  print('cant find second linear move from: x=' +str(x) + 'y=' + str(y) + 'z=' +str(z))
                  break
                time.sleep(1)
    group.default(velocity_scaling_argument, acceleration_scaling_argument)
    print("finished successfully")

 
