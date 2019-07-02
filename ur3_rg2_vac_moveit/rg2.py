#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys	
import copy
import rospy

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import *

from geometry_msgs.msg import Quaternion
import geometry_msgs.msg
from math import pi
from std_msgs.msg import *

from moveit_commander.conversions import pose_to_list
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from ur3_rg2_moveit.srv import *
from ur_msgs.srv import *
from ur_msgs.msg import *
import socket

#from MsgCenter.msg import *
from opencv.msg import MsgAngle
from opencv.msg import MsgCenter
from opencv.msg import MsgDetection

## END_SUB_TUTORIAL




## Manipulator의 각 joint가 actual에서 goal로 이동할 때 tolerance를 초과하지 않는지 확인하는 함수?
def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:	# python에서는 데이터 타입을 확인 하기 위해서 type()을 사용. 즉 "goal의 type이 list라면"이란 의미
							# 여기서 goal은 A list of floats, a Pose or a PoseStamped
							# Manipulator의 각 joint의 goal을 의미한다. 그러므로 UR3의 경우 총 6개가 될 것이다.
    for index in range(len(goal)):	# len() 함수는 문자열의 문자 개수를 반환하는 내장함수. 
									# range()는 연속된 숫자(정수)를 만들어준다. ex) range(10) = [0,1,2,3,4,5,6,7,8,9]
									# 그러므로 UR3의 경우 [0,1,2,3,4,5] 총 6번의 반복이 이루어질 것이다.
      if abs(actual[index] - goal[index]) > tolerance:	# abs()는 절대값을 의미한다. 
														# actual[index]는 현재 joint의 값, goal[index]는 목표 joint의 값이며
														# 이 차의 절대값이 tolerance(허용한계치)를 넘어서면 False를 return하게 된다.
        return False

  ## 아래 두 elif 조건문 내용은 만약 goal의 type이 list가 아닌 geometry_msgs.msg.PoseStamped나 geometry_msgs.msg.Pose라면
  ## 이를 list 형태로 재수행 한다는 내용이다.
  elif type(goal) is geometry_msgs.msg.PoseStamped:	# geometry_msgs.msg.PoseStampedsms pose와 timestamp내용을 함께 담고있다.	
    return all_close(goal.pose, actual.pose, tolerance)	# 이를 해결하기 위해 pose 내용만을 다시 보낸다.
														# pose는 Point position과 Quaternion orientation을 담고 있다. 

  elif type(goal) is geometry_msgs.msg.Pose:	# geometry_msgs.msg.Pose는 position과 Quaternion orientation을 담고 있다.
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)	# 이를 list 형태로 보내기 위해 pose_to_list()함수를 사용한다.

  return True	# 각 joint의 운동이 tolerance 이내라면 True를 return한다.

## 현재 UR3 Gripper에는 파지를 도울 보조 vaccumm 그리퍼가 달려있다. 이는 UR3의 제어기의 IO포트를 통해 제어되며, 아래는 그에 대한 설정이다. 
class VAC_gripper():	
  def __init__(self):	# self는 객체의 인스턴스 그 자체를 의미한다. 대부분 객체지향 언어는 이걸 메소드에 안 보이게 전달하지만 
						# 파이썬에서 클래스의 메소드를 정의할 때는 self를 꼭 명시하여야 하고 그 메소드를 불러올 때 self는 자동으로 전달된다.
						# __init__은 파이썬에서 쓰이는 생성자이다.
    rospy.wait_for_service('ur_driver/set_io')	# rospy.wait_for_service는 'ur_driver/set_io'라는 이름의 서비스가 활성화 될 때 까지 대기하는 함수이다.
												# 'ur_driver/set_io'를 이용하여  UR3 제어기의 io포트를 제어할 수 있다.
    set_io = rospy.ServiceProxy('ur_driver/set_io', SetIO)	# rospy.ServiceProxy를 이용해 서비스를 호출하는 핸들(set_io)을 생성한다.
								# 'ur_driver/set_io'는 서비스의 이름이며
								# 'SetIO'는 'from ur_msgs.srv import *'를 이용해 import된 서비스 모듈이며 아래와 같이 구성된다.
								# int8 fun : The function to perform
								# int8 pin : Numeric ID of the pin to set
								# float32 state : Desired pin state (signal level for analog or STATE_(ON|OFF) for DIO and flags)
															# ---
															# bool success
    self.set_io = set_io	# 서비스 핸들(set_io)의 return값을 객체의 인스턴스의 set_io로 가져온다.

  def On_grip(self):	
    print "on_grip"
    # fun ===>>> int8 FUN_SET_DIGITAL_OUT = 1 / int8 FUN_SET_FLAG = 2 / int8 FUN_SET_ANALOG_OUT = 3 / int8 FUN_SET_TOOL_VOLTAGE = 4


    self.set_io(1, 0, 24.0)	# Digital Out을 0번 pin에 24V의 크기로 설정한다.
    #rospy.sleep(3)
    #set_io(1, 0, 0)

  def Off_grip(self):
    self.set_io(1, 0, 0.0)	# Digital Out을 0번 pin에 0V의 크기로 설정한다.

	
class RG2_gripper():
  def __init__(self):
    print "RG2_gripper_init"
    rospy.wait_for_service('/rg2_gripper/control_width')	# rospy.wait_for_service는 '/rg2_gripper/control_width'라는 이름의 서비스가 활성화 될 때 까지 대기하는 함수이다.
    print "RG2_gripper_init_wait_for_service success"
  def move_grip(self, target_width):	# target_width는 목표 물체의 넓이를 의미한다.
    try:
      val = rospy.ServiceProxy('/rg2_gripper/control_width', RG2)	# rospy.ServiceProxy를 이용해 서비스를 호출하는 핸들(val)을 생성한다.
									# '/rg2_gripper/control_width'는 서비스의 이름이며
									# ur_modern_driver/src/ur_ros_wrapper.cpp 파일에 
									# '/rg2_gripper/control_width' 서비스에 대한 내용이 있다.
									# 'RG2'는 'from ur3_rg2_moveit.srv import *'를 이용해 import된 서비스 모듈이며 아래와 같이 구성된다.
									# (request)	std_msgs/Float64 target_width
									# ---
									# (response) std_msgs/Float64 current_width

      width = RG2Request()	# 서비스 요청 인스턴스 width를 만든다.
      width.target_width.data = target_width * 1000	# target_width에 1000을 곱한 값을 width.target_width.data의 값으로 준다.
													# 해당 값에 접근하기 위해서는 .data를 사용하여야 한다.

      val(width)	# 서비스 핸들(val)에 width값을 파라미터로 주어 호출한다.
      print "gripped with : %.3f" % width.target_width.data

    except rospy.ServiceException, e:
      print "Service call failed: %s" % e

  def feedback_grip(self):
    rospy.Subscriber("/chatter", UInt16, self.callback)	# 사용자의 노드가 UInt16 타입의 메세지를 /chatter 토픽으로부터 구독한다는 선언이다. 
							# 새로운 메세지를 수신했을 때, self.callback 함수는 그 메세지를 첫번째 파라미터로 하여 실행되게 된다.
    rospy.spin()					# 서비스 수신 대기

  def callback(self, data):
     print "i heard : %s " % data.data


## UR3 control Class
## object라는 부모 클래스를 상속한 MoveGroupPythonIntefaceTutorial라는 자식 클래스
## 예상컨데 object 내에는 파지할 대상에 대한 정보가 있을 것임
class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""

  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()	# super() : 자식 클래스의 오버라이드된 메소드(def __init__(self))에서 부모클래스의 메소드를 사용할 때 사용한다.
															# Python3에서는 super를 인수 없이 호출하면 __class__와 self를 인수로 넘겨서 호출한 것으로 처리된다.
															# 여기서는 부모 클래스(object)의 생성자(__init__)을 사용한 것이다.
	
    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)	# sys.argv[0]는 프로그램 명, sys.argv[1]은 입력파라미터이다.
													# 즉, 프로그램명과 프로그램이 실행될 때 입력된 파라미터들을 함께
													# moveit_commander.roscpp_initialize에 전달한다.
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)	# 'move_group_python_interface_tutorial' node 초기화

    ## Instantiate a `RobotCommander`_ object /  `PlanningSceneInterface`_ object  /  `MoveGroupCommander`_ object

    robot = moveit_commander.RobotCommander()	# 로봇의 상태와 관련된 메소드를 지닌다.
    scene = moveit_commander.PlanningSceneInterface()	# Simple interface to making updates to a planning scene

    group_name = "ur3_rg2_vac"
    group = moveit_commander.MoveGroupCommander(group_name)	# Execution of simple commands for a particular group

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

	## UR3 Execution과 관련된 설정
    group.set_pose_reference_frame("/world")
    group.allow_replanning(True)
    group.set_goal_position_tolerance(0.001)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)

    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print
    "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print
    "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print
    "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print
    "============ Printing robot state"
    print
    robot.get_current_state()

    # Move to Home Position
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = - pi / 2
    joint_goal[2] = 0
    joint_goal[3] = -pi / 2
    joint_goal[4] = 0
    joint_goal[5] = 0

    group.go(joint_goal, wait=True)
    group.stop()
    print
    "========== Complete to Initialize"

    # Misc variables
    self.box_name = ''
    self.object_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    self.angle = 0.0
    self.center_x = 0.0
    self.center_y = 0.0
    self.detection = 0

  def go_to_joint_state(self):

    group = self.group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi / 2
    joint_goal[2] = 0
    joint_goal[3] = -pi / 2
    joint_goal[4] = 0
    joint_goal[5] = 0
    # joint_goal[6] = 0

    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = group.get_current_joint_values()

    print
    "====== Printing current Pose & Joint Value (After Joint Move)"
    print
    current_joints
    print
    group.get_current_pose().pose

    return all_close(joint_goal, current_joints, 0.01)	# !!! 해당 함수는 group.go() 전에 수행해서 
														# !!! return 값이 true면 group().go를 실행하는 식으로 해야하는거 아닌가??

  def go_to_pose_goal(self, pose_goal):
    #print "get in def go_to_pose_goal"
    group = self.group

    #print group.get_current_pose().pose

    ## Planning to a Pose Goal
    ## Plan a motion for this group to a desired pose for the end-effector:
    # pose_goal = geometry_msgs.msg.Pose()

    # Orientation
    # 45deg = 0.3927 / 90deg = 0.7854 / 135deg = 1.1781 / 180deg = 1.5707 / 225deg = 1.9635 / 270deg = 2.3562 / 315deg = 2.7489
    # forward : (1.5707, 0, 2.3562)

    group.set_pose_target(pose_goal)

    #if(sqrt(pow(abs(pose_goal.position.x), 2) + pow(abs(pose_goal.position.y), 2) + pow(abs(pose_goal.position.z), 2)) > 0.62):
    #  print "Unvalidate Target : So far"

    #print "====== Target Pose (Before Pose move)"
    #print pose_goal

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    group.stop() 

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    # print "====== Current Pose (After Pose move)"
    current_pose = group.get_current_pose().pose
    #print "====== Target Pose (After Pose move)"
    #print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])	# get_attached_objects : 
																# Get the attached objects identified by the given object ids list. 
																# If no ids are provided, return all the attached objects.
      is_attached = len(attached_objects.keys()) > 0	# keys() : 딕셔너리 attached_objects의 key만을 모아서 dict_keys라는 객체를 리턴한다.
														# ex) 	a = {'name': 'pey', 'phone': '0119993323', 'birth': '1118'}
														# 		a.keys()
														# 		dict_keys(['name', 'phone', 'birth'])
														# len(dict.keys()) : keys()의 개수를 나타낸다. dictionary의 개수와 keys의 개수는 같다. 
														# 즉 is_attached는 keys의 개수가 1개 이상이면 true, 0개이면 flase가 됨.

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()	# get_known_object_names : Get the names of all known objects in the world. 
															# If with_type is set to true, only return objects that have a known type.

      # Test if we are in the expected state
	  # box_is_attached가 true인지 false인지 혹은 box_is_known이 true인지 false인지 메소드의 입력 파라미터로 줌으로써 예측을 하고
	  # 메소드 내에서 실제 로봇의 상태를 scene와 관련된 함수를 이용해 알아내어, 예측과 실제 상태가 같다면 True를 return하고 다르다면 False를 리턴한다.
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

## scene에 box를 추가하는 메소드
  def add_box(self, obj_pose, size, timeout=4):

    box_name = self.box_name
    scene = self.scene

    box_name = "box"

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "/world"
    box_pose.pose.position.x = obj_pose.position.x
    box_pose.pose.position.y = obj_pose.position.y
    box_pose.pose.position.z = obj_pose.position.z  # ((obj_pose.position.z - 0.05) / 2)
    box_pose.pose.orientation = obj_pose.orientation

    scene.add_box(box_name, box_pose, size=(size[0], size[1], size[2]))

    self.box_name = box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

## 
  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    grasping_group = 'rg2_vac'
    touch_links = robot.get_link_names(group=grasping_group)	# get_link_names : Get the links that make up a group. 
																# If no group name is specified, 
																# all the links in the robot model are returned.
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=box_name)

    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def add_object(self, object_kinds, object_pose, object_size, timeout=4):
    name = self.box_name
    scene = self.scene

    pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = "rg2_eef_link"
    pose.header.frame_id = "base_link"
    pose.pose.orientation.w = 1.0
    name = object_name
    size = object_size

    if (object_kinds == 'box'):
      scene.add_box(name, pose, size=(0.1, 0.1, 0.1))

    # self.box_name = box_name

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def callback_angle(self, angle):
    #rospy.loginfo(rospy.get_caller_id() + "angle %f", angle.angle)
    self.angle = angle.angle

  def callback_center(self, center):
    #rospy.loginfo(rospy.get_caller_id() + "center %f, %f", center.x, center.y)
    self.center_x = center.x
    self.center_y = -center.y

  def callback_detection(self, detection):
    #rospy.loginfo(rospy.get_caller_id() + "detection %d", detection.detection)
    self.detection = detection.detection

  def listener(self):
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("angle", MsgAngle, self.callback_angle)
    rospy.Subscriber("center", MsgCenter, self.callback_center)
    rospy.Subscriber("detection", MsgDetection, self.callback_detection)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    UR3 = MoveGroupPythonIntefaceTutorial()
    RG2 = RG2_gripper()
    VAC = VAC_gripper() # not using



    #global location
    group = UR3.group
    scene = UR3.scene
    grip_width = 0.110
    angle = UR3.angle
    center_x = UR3.center_x
    center_y = UR3.center_y
    detection = UR3.detection
    # Open gripper
    RG2.move_grip(grip_width)
    VAC.Off_grip()
    rospy.sleep(1)
    detection_flag = 0
    # Create objects(Butter waffle) on simulation

    print "add box"



    ########################################################################################################################################
    ### Searching Objects
    ########################################################################################################################################

    print "move for searching"
	
    pose_goal = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(-1.5707, 1.5707, 1.5707)	# from tf.transformations import quaternion_from_euler 의 모듈
														# parameter는 roll / pitch / yaw 이다.
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]

    o_size=[]	# object_size







   
    #server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	# socket : import socket 의 모듈
								# 단순히 프로그램의 내부와 외부를 잇는 표준 입출력과는 다르게
								# 네트워크의 반대편이 어디인지에 대한 정보를 가지고 있다.
								# socket.socket() : 소켓 객체를 생성할 수 있다. 이 함수는 두 가지 인자를 받는데,
								# 하나는 패밀리이고 다른 하나는 타입이다.
								# 1. 패밀리 : 소켓의 패밀리란, “택배상자에 쓰는 주소 체계가 어떻게 되어 있느나”에 관한 것으로 
								# 흔히 AF_INET, AF_INET6를 많이 쓴다. 전자는 IP4v에 후자는 IP6v에 사용된다. 
								# 각각 socket.AF_INET, socket.AF_INET6로 정의되어 있다.
								# 2. 타입 : 소켓 타입이다. raw 소켓, 스트림소켓, 데이터그램 소켓등이 있는데, 
								# 보통 많이 쓰는 것은 socket.SOCK_STREAM 혹은 socket.SOCK_DGRAM이다.
								# 가장 흔히 쓰이는 socket.AF_INET, socket.SOCK_STREAM 조합은 
								# socket.socket()의 인자 중에서 family=, type=에 대한 기본 인자값이다. 
								# 따라서 이 타입의 소켓을 생성하고자 하는 경우에는 인자 없이 socket.socket()만 써도 무방하다.
    
    #server.connect(('192.168.0.5', 6720))	# !!! server가 아니라 client 아닌가? connect를 한다는 것은 client에서 server로 하잖은가? 
											# 생성->바인드->듣기->수락->읽기->쓰기->닫기의 사이클 대로 동작한다면, 
											# 클라이언트는 그 자신이 접속을 능동적으로 수행하기 때문에 생성->연결->쓰기->읽기의 사이클이 적용된다.
											# 그러므로 이건 client가 맞다.
											# 서버의 IP와 포트번호를 튜플 파라미터로 입력한다.
    #print('connected to server')

    UR3.listener() # subscriber which accept angle and center point

    move_cnt = 0
    while(1):
      # 1. Detect Object
      if UR3.detection > 0 :
      	detection_flag = 1
      	
      print "move_cnt : ", move_cnt
      
      #object is detected
      if detection_flag == 1 :
	if move_cnt==0:
		# 2. Stop the Conveyor
		# we should stop the conveyor in this line
		# ========================================
		rospy.sleep(3)
		move_cnt += 1
		
	elif move_cnt==1:
		# 3. Go to the Center Point of detected Object. height should be higher than Object.
		#RG2.move_grip(0.110)

		angle = UR3.angle
		euler_angle = (angle*3.14)/180
		#print "angle : ",angle
		print "euler_angle : ", euler_angle

		center_x = (((UR3.center_x/862)*520)+130)/1000
		center_y = (((UR3.center_y/646)*390)+230)/1000
		print "center_x : ", center_x
		print "center_y : ", center_y

		q = quaternion_from_euler(-1.5707, 1.5707, euler_angle)  # from tf.transformations import quaternion_from_euler 의 모듈
		# parameter는 roll / pitch / yaw 이다.
		pose_goal.orientation.x = q[0]
		pose_goal.orientation.y = q[1]
		pose_goal.orientation.z = q[2]
		pose_goal.orientation.w = q[3]

		pose_goal.position.x = center_x
		pose_goal.position.y = center_y
		pose_goal.position.z = 0.030 + 0.1 + 0.0392 - (0.0003564 * 0.042 * 1000)
		print "get in detection > 0"

		UR3.go_to_pose_goal(pose_goal)
		UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
		#rospy.sleep(5)
		move_cnt += 1
	elif move_cnt==2:
		
		# 4. Go Down
		pose_goal.position.z = 0.010
		  # pose_goal.position.z = o_size[2] + 0.1 + 0.0392 - (0.0003564 * grip_width * 1000)
			  # o_size[2] + 0.1 + 0.0392 : o_size[2]는 object의 높이이며 0.1과 0.0392를 더해 end effector의 최대 높이로 맞추어준다.
			  # - (0.0003564 * grip_width * 1000) : 0.0003564는 grip_width의 변환 정도에 따른 값의 변화량(기울기)이다.
			  # 그러므로 0.0003564에 grip_width를 곱하고 단위정리를 위해 1000을 곱하여서 빼주면 grip_width에 따른 적절한 z를 구할 수 있게 된다.

		UR3.go_to_pose_goal(pose_goal)
		UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
		#rospy.sleep(5)
		move_cnt += 1
	        
	elif move_cnt==3:
        
        
		# 5. Gripping
		RG2.move_grip(0.042)
		#VAC.On_grip()
		rospy.sleep(3)
        	move_cnt += 1
        
	elif move_cnt==4:
		# 6. Go Up
		pose_goal.position.z = 0.250
		UR3.go_to_pose_goal(pose_goal)
		UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
		#rospy.sleep(5)
		move_cnt += 1

	elif move_cnt==5:
        
		# 7. Go to the Plate
		q = quaternion_from_euler(-1.5707, 0.0000, 3.1405)  # from tf.transformations import quaternion_from_euler 의 모듈
		# parameter는 roll / pitch / yaw 이다.
		pose_goal.orientation.x = q[0]
		pose_goal.orientation.y = q[1]
		pose_goal.orientation.z = q[2]
		pose_goal.orientation.w = q[3]

		pose_goal.position.x = -0.2
		pose_goal.position.y = 0.2
		UR3.go_to_pose_goal(pose_goal)
		UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
		#rospy.sleep(5)
		move_cnt += 1
	
	elif move_cnt==6:
		# 8. Go Down
		#q = quaternion_from_euler(-1.5707, 0.0000, 3.1405)  # from tf.transformations import quaternion_from_euler 의 모듈
		# parameter는 roll / pitch / yaw 이다.
		#pose_goal.orientation.x = q[0]
		#pose_goal.orientation.y = q[1]
		#pose_goal.orientation.z = q[2]
		#pose_goal.orientation.w = q[3]

		pose_goal.position.x = -0.6
		pose_goal.position.y = 0.1
		#pose_goal.position.z = 0.010
		UR3.go_to_pose_goal(pose_goal)
		UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
		#rospy.sleep(5)
		move_cnt += 1

	elif move_cnt==7:
		# 9. Release the Object
		RG2.move_grip(0.110)
		#VAC.Off_grip()
		rospy.sleep(3)
		move_cnt += 1
	
	elif move_cnt==8:
		# 10. Go to the Origin Position
		q = quaternion_from_euler(-1.5707, 1.5707, 1.5707)  # from tf.transformations import quaternion_from_euler 의 모듈

		pose_goal.orientation.x = q[0]
		pose_goal.orientation.y = q[1]
		pose_goal.orientation.z = q[2]
		pose_goal.orientation.w = q[3]

		pose_goal.position.x = 0.2
		pose_goal.position.y = 0.2
		pose_goal.position.z = 0.2

		UR3.go_to_pose_goal(pose_goal)
        	UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
		#VAC.Off_grip()
		#rospy.sleep(5)
		move_cnt = 0
		detection_flag = 0

	'''
        # 10. Go to the Origin Position
        q = quaternion_from_euler(-1.5707, 1.5707, 1.5707)  # from tf.transformations import quaternion_from_euler 의 모듈
                                                            # parameter는 roll / pitch / yaw 이다.
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        pose_goal.position.x = 0.2
        pose_goal.position.y = 0.2
        pose_goal.position.z = 0.2

        UR3.go_to_pose_goal(pose_goal)
        UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
        rospy.sleep(5)
        '''
      #object is not detected
      else :

        #RG2.move_grip(grip_width)
        q = quaternion_from_euler(-1.5707, 1.5707, 1.5707)  # from tf.transformations import quaternion_from_euler 의 모듈
                                                            # parameter는 roll / pitch / yaw 이다.
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        pose_goal.position.x = 0.2
        pose_goal.position.y = 0.2
        pose_goal.position.z = 0.2
        print "get in detection < 0"

        UR3.go_to_pose_goal(pose_goal)
        UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
        #rospy.sleep(5)
      '''
      data = server.recv(1024).decode('ascii')			# client는 1024(2^10), 즉 10비트의 데이터를 받고 ascii 방식으로 디코딩한다.
      if data == 'scan start':							# 만약 server로부터 'scan start'라는 메세지를 받는다면
        print "scan start"								# 'scan start'를 출력해주고
        while True:										# 해당 과정을 반복한다.
          location = server.recv(1024).decode('ascii')	# server로부터 scan할 location을 수신받고
          if location == 'scan end':					# location을 수신받지 않고 'scan end'를 수신받았다면
            break										# 멈추게 된다.
          print(location)								# location을 출력해주고
          location = location.split(',')				# location은 "x, y, z" 형태의 메세지로 수신된다. 이를 .split(',')을 이용해 나누어 준다.	
          pose_goal.position.x = float(location[1])
          pose_goal.position.y = float(location[2])
          pose_goal.position.z = float(location[3])

          UR3.go_to_pose_goal(pose_goal)
          UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
          # rospy.sleep(1)
          server.send('located')

      # UR3.go_to_joint_state()
      # UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
      '''
      ########################################################################################################################################
      ### Picking Objects
      ########################################################################################################################################   
      '''	
      print "wait to grip"
      server.send('ready')	# client에서 'ready'를 server로 보낸다? 어째서? ... 아마 아무 행동도 하지 말고 기다리라는 의미?
      data = server.recv(1024).decode('ascii')	# 'ready'를 보낸 이후 data 수신...
      print(data)
      if data == 'start grip':	# 'start grip'이 수신 된다면
        print "start grip"
        while True:
          location = server.recv(2048).decode('ascii')	# server로부터 location 수신
          if location == 'end grip':	# grip이 끝났다면
            print "end grip"
            UR3.go_to_joint_state()	# 원상태 복귀
            break
          location = location.split(',')	# location을 받으면 먼저 split 함
											# 예상컨데 여기서 받는 location은 pose_goal로써 ( start or end grip, position.x, position.y, 
											# 오른쪽과 같다.								object.rad, object.x, object.y, object.z )
											# 그렇다면 아래의 o_size는 object_size가 된다.										

          o_size = [float(location[4]) / 1000, float(location[5]) / 1000, float(location[6]) / 1000]
          grip_width = o_size[1]

          pose_goal.position.x = float(location[1])
          pose_goal.position.y = float(location[2])
          pose_goal.position.z = o_size[2] + 0.1 + 0.0392 - (0.0003564 * grip_width * 1000)
		  # o_size[2] + 0.1 + 0.0392 : o_size[2]는 object의 높이이며 0.1과 0.0392를 더해 end effector의 최대 높이로 맞추어준다.
		  # - (0.0003564 * grip_width * 1000) : 0.0003564는 grip_width의 변환 정도에 따른 값의 변화량(기울기)이다.
		  # 그러므로 0.0003564에 grip_width를 곱하고 단위정리를 위해 1000을 곱하여서 빼주면 grip_width에 따른 적절한 z를 구할 수 있게 된다.
		  
          rad = float(location[3])	# location[3]는 
          q = quaternion_from_euler(-1.5707, 1.5707, rad)	# roll : -180도 / pitch : 180도 / yaw : rad(object의 각도)
          pose_goal.orientation.x = q[0]
          pose_goal.orientation.y = q[1]
          pose_goal.orientation.z = q[2]
          pose_goal.orientation.w = q[3]
          print(pose_goal)

          UR3.go_to_pose_goal(pose_goal)
          UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

		  # object의 위치에 도달해서 orientation까지 맞춘 후 아래로 내려가는 과정이다.
          pose_goal.position.z -= 0.125
          UR3.go_to_pose_goal(pose_goal)
          print(pose_goal)
          UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

          print("Enter for grasping")
          raw_input()	# 사용자로부터 키보드로 입력받는 함수

          RG2.move_grip(grip_width)
          VAC.On_grip()
          rospy.sleep(5)



          ### Placing Objects

          pose_goal = group.get_current_pose().pose
          pose_goal.position.z = 0.25
          UR3.go_to_pose_goal(pose_goal)
          UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

          pose_goal.position.x = -0.25
          pose_goal.position.y = 0
          UR3.go_to_pose_goal(pose_goal)
          UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

          pose_goal.position.z = o_size[2] + 0.05 + 0.0392 - (0.0003564 * grip_width * 1000)
          UR3.go_to_pose_goal(pose_goal)
          UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

          grip_width = 0.110
          RG2.move_grip(grip_width)
          VAC.Off_grip()
          rospy.sleep(1)

          q = quaternion_from_euler(-1.5707, 1.5707, 1.5707)
          pose_goal.orientation.x = q[0]
          pose_goal.orientation.y = q[1]
          pose_goal.orientation.z = q[2]
          pose_goal.orientation.w = q[3]

          server.send('ok')

    #UR3.add_box(o_pose, o_size)

    '''


    ########################################################################################################################################
    #### Grasping After moving
    ########################################################################################################################################
    print "#### Grasping After moving"
    ''' 
    pose_goal = group.get_current_pose().pose
    pose_goal.position.z += 0.05
    UR3.go_to_pose_goal(pose_goal)
    UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

    # pose_goal.position.x = -0.25
    # UR3.go_to_pose_goal(pose_goal)
    # UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

    pose_goal = group.get_current_pose().pose
    pose_goal.position.z += 0.05
    UR3.go_to_pose_goal(pose_goal)
    UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)


    RG2.move_grip(110)
    VAC.Off_grip()
    rospy.sleep(1)

    #UR3.detach_box()
    #UR3.remove_box()

    UR3.go_to_joint_state()
    '''
    print "============ Python tutorial demo complete!"


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
