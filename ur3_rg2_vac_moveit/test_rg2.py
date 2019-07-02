#!/usr/bin/env python

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
import serial
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class VAC_gripper():
  def __init__(self):
    rospy.wait_for_service('ur_driver/set_io')
    set_io = rospy.ServiceProxy('ur_driver/set_io', SetIO)
    self.set_io = set_io

  def On_grip(self):
    print "on_grip"
    # fun ===>>> int8 FUN_SET_DIGITAL_OUT = 1 / int8 FUN_SET_FLAG = 2 / int8 FUN_SET_ANALOG_OUT = 3 / int8 FUN_SET_TOOL_VOLTAGE = 4


    self.set_io(1, 0, 24.0)
    #rospy.sleep(3)
    #set_io(1, 0, 0)

  def Off_grip(self):
    self.set_io(1, 0, 0.0)

class RG2_gripper():
  def __init__(self):
    rospy.Subscriber("/chatter", UInt16, self.callback)
    rospy.wait_for_service('/rg2_gripper/control_width')


    # self.ser = serial.Serial('/dev/ttyACM0', 9600)
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    # r = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #   message = self.ser.readline()
    #   pub.publish(message)
    #   r.sleep()

  def callback(self, data):
    print "i heard : %s " % data.data

  def feedback_grip(self):
    # rospy.Subscriber("/chatter", String, self.callback)
    print("feedback")
    rospy.spin()

  def move_grip(self, target_width):
    try:
      val = rospy.ServiceProxy('/rg2_gripper/control_width', RG2)

      width = RG2Request()
      width.target_width.data = target_width * 1000

      val(width)
      print "gripped with : %.3f" % width.target_width.data

    except rospy.ServiceException, e:
      print "Service call failed: %s" % e

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""

  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object /  `PlanningSceneInterface`_ object  /  `MoveGroupCommander`_ object

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "ur3_rg2_vac"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    group.set_pose_reference_frame("/world")
    group.allow_replanning(True)
    group.set_goal_position_tolerance(0.001)
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)

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

    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self, pose_goal):

    group = self.group

    # print group.get_current_pose().pose

    ## Planning to a Pose Goal
    ## Plan a motion for this group to a desired pose for the end-effector:
    # pose_goal = geometry_msgs.msg.Pose()

    # Orientation
    # 45deg = 0.3927 / 90deg = 0.7854 / 135deg = 1.1781 / 180deg = 1.5707 / 225deg = 1.9635 / 270deg = 2.3562 / 315deg = 2.7489
    # forward : (1.5707, 0, 2.3562)

    group.set_pose_target(pose_goal)

    #if(sqrt(pow(abs(pose_goal.position.x), 2) + pow(abs(pose_goal.position.y), 2) + pow(abs(pose_goal.position.z), 2)) > 0.62):
    #  print "Unvalidate Target : So far"

    # print "====== Target Pose (Before Pose move)"
    # print pose_goal

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
    print "====== Target Pose (After Pose move)"
    print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def track_to_pose(self, pose_goal):
    group = self.group

    group.set_pose_target(pose_goal)

    # if(sqrt(pow(abs(pose_goal.position.x), 2) + pow(abs(pose_goal.position.y), 2) + pow(abs(pose_goal.position.z), 2)) > 0.62):
    #  print "Unvalidate Target : So far"

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=False)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()

    group.clear_pose_targets()

    current_pose = group.get_current_pose().pose
    print "====== Target Pose (After Pose move)"
    print current_pose

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
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

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
    touch_links = robot.get_link_names(group=grasping_group)
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

def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."

    UR3 = MoveGroupPythonIntefaceTutorial()
    RG2 = RG2_gripper()
    VAC = VAC_gripper()

    group = UR3.group
    scene = UR3.scene

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.3
    pose_goal.position.y = 0.3
    pose_goal.position.z = 0.2
    q = quaternion_from_euler(-1.5707, 1.5707, 1.5707)
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]

    UR3.track_to_pose(pose_goal)

    rospy.sleep(2)

    pose_goal.position.x = -0.2
    UR3.track_to_pose(pose_goal)

    rospy.sleep(1)
    pose_goal.position.y = -0.2
    UR3.track_to_pose(pose_goal)

    print "============ Python tutorial demo complete!"


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()