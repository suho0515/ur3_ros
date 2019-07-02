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
    rospy.wait_for_service('/rg2_gripper/control_width')

  def move_grip(self, target_width):
    try:
      val = rospy.ServiceProxy('/rg2_gripper/control_width', RG2)
      width = RG2Request()
      width.target_width.data = target_width * 1000

      val(width)
      print "gripped with : %.3f" % width.target_width.data

    except rospy.ServiceException, e:
      print "Service call failed: %s" % e

  def feedback_grip(self):
    rospy.Subscriber("chatter", String, self.callback)
    rospy.spin()

  def callback(self, data):
     print "i heard : %s " % data

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

    group.set_pose_reference_frame("/base_link")
    group.allow_replanning(True)
    group.set_goal_position_tolerance(0.001)
    group.set_max_velocity_scaling_factor(0.5)
    group.set_max_acceleration_scaling_factor(0.4)
    
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()


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
    print "========== Complete to Initialize"

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
    joint_goal[1] = -pi/2
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = 0
   # joint_goal[6] = 0

    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = group.get_current_joint_values()

    print "====== Printing current Pose & Joint Value (After Joint Move)"
    print current_joints
    print group.get_current_pose().pose

    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self, pose_goal):

    group = self.group

    #print group.get_current_pose().pose

    ## Planning to a Pose Goal
    ## Plan a motion for this group to a desired pose for the end-effector:
    #pose_goal = geometry_msgs.msg.Pose()
    

    # Orientation
    # 45deg = 0.3927 / 90deg = 0.7854 / 135deg = 1.1781 / 180deg = 1.5707 / 225deg = 1.9635 / 270deg = 2.3562 / 315deg = 2.7489
    # forward : (1.5707, 0, 2.3562)


    group.set_pose_target(pose_goal)

    print "====== Target Pose (Before Pose move)"
    print pose_goal

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    #print "====== Current Pose (After Pose move)"
    current_pose = group.get_current_pose().pose

    #print current_pose
    
    return all_close(pose_goal, current_pose, 0.01)

  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = []

    # UR3.go_to_pose_goal(-0.39, 0.30, 0.38, -1.5707, 1.5707, 2.3562)
    # UR3.go_to_pose_goal(-0.39, 0.30, 0.32, -1.5707, 1.5707, 2.3562)    # position [m]   /   pose [rad]    /

    wpose = group.get_current_pose().pose

    wpose.position.x = -0.39
    wpose.position.y = 0.30
    wpose.position.z = 0.36
    q = quaternion_from_euler(-1.5707, 1.5707, 2.3562)  # Euler ==> Quaternnion
    wpose.orientation.x = q[0]
    wpose.orientation.y = q[1]
    wpose.orientation.z = q[2]
    wpose.orientation.w = q[3]

    waypoints.append(copy.deepcopy(wpose))


    wpose.position.y -= 0.6  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

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
    box_pose.header.frame_id = "base_link"
    box_pose.pose.position.x = obj_pose.position.x
    box_pose.pose.position.y = obj_pose.position.y
    box_pose.pose.position.z = obj_pose.position.z#((obj_pose.position.z - 0.05) / 2)
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
    #box_pose.header.frame_id = "rg2_eef_link"
    pose.header.frame_id = "base_link"
    pose.pose.orientation.w = 1.0
    name = object_name
    size = object_size

    if(object_kinds == 'box'):
      scene.add_box(name, pose, size=(0.1, 0.1, 0.1))


    #self.box_name = box_name

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def move_for_picking(self, grasping_object, timeout=4):

    group = self.group

    # Using Cartesian Path
    waypoints = []

    wpose = geometry_msgs.msg.Pose()

    wpose.position.x = grasping_object.position.x
    wpose.position.y = grasping_object.position.y
    wpose.position.z = grasping_object.position.z + 0.1
    wpose.orientation.x = grasping_object.orientation.x
    wpose.orientation.y = grasping_object.orientation.y
    wpose.orientation.z = grasping_object.orientation.z
    wpose.orientation.w = grasping_object.orientation.w
    '''
    q = quaternion_from_euler(grasping_object.rx, grasping_object.ry, grasping_object.rz)  # Euler ==> Quaternnion
    wpose.orientation.x = q[0]
    wpose.orientation.y = q[1]
    wpose.orientation.z = q[2]
    wpose.orientation.w = q[3]
    '''

    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z -= 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                                    waypoints,  # waypoints to follow
                                                    0.01,  # eef_step
                                                    0.0)  # jump_threshold

    self.display_trajectory(plan)

    print "============ Press `Enter` to execute Cartesian Path  ..."
    raw_input()
    self.execute_plan(plan)

    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def move_for_placing(self, timeout=4):

    group = self.group

    # Using Cartesian Path
    waypoints = []

    wpose = group.get_current_pose().pose

    #UR3.go_to_pose_goal(-0.2, 0.2, 0.25, -1.5707, 1.5707, 2.3562)

    wpose.position.z += 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = -0.2
    wpose.position.y = 0.2
    '''
    q = quaternion_from_euler(-1.5707, 1.5707, 1.5707)
    wpose.orientation.x = q[0]
    wpose.orientation.y = q[1]
    wpose.orientation.z = q[2]
    wpose.orientation.w = q[3]
    '''
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                                    waypoints,  # waypoints to follow
                                                    0.005,  # eef_step
                                                    0.0)  # jump_threshold

    self.display_trajectory(plan)

    print "============ Press `Enter` to execute Cartesian Path  ..."
    raw_input()
    self.execute_plan(plan)

    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    UR3 = MoveGroupPythonIntefaceTutorial()
    RG2 = RG2_gripper()
    VAC = VAC_gripper()

    group = UR3.group
    scene = UR3.scene

    # Open gripper
    RG2.move_grip(110)
    VAC.Off_grip()
    rospy.sleep(1)

    # Create objects(Butter waffle) on simulation
    '''
    print "add box"
    raw_input()
    # Move for picking
    o_pose = geometry_msgs.msg.Pose()
    o_pose.position.x = 0
    o_pose.position.y = 0.3
    o_pose.position.z = 0.22
    q = quaternion_from_euler(0, 0, 0)
    o_pose.orientation.x = q[0]
    o_pose.orientation.y = q[1]
    o_pose.orientation.z = q[2]
    o_pose.orientation.w = q[3]

    UR3.add_box(o_pose)

    print "move for picking"
    raw_input()

    #UR3.move_for_picking(o_pose)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0
    pose_goal.position.y = 0.3
    pose_goal.position.z = 0.22
    q = quaternion_from_euler(-1.5707, 1.5707, 1.5707)  # Euler ==> Quaternnion
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]

    UR3.go_to_pose_goal(pose_goal)

    UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

    #grasping_point.position.z -= 10

    print "move for picking2"
    raw_input()

    UR3.remove_box()

    pose_goal.position.z = 0.1
    UR3.go_to_pose_goal(pose_goal)
    UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

    UR3.add_box(o_pose)
    UR3.attach_box()

    rospy.sleep(1)

    print "grapsing"
    raw_input()
    RG2.move_grip(95)
    VAC.On_grip()

    rospy.sleep(2)

    #UR3.move_for_placing()
    pose_goal = group.get_current_pose().pose
    pose_goal.position.z += 0.12

    UR3.go_to_pose_goal(pose_goal)
    UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

    pose_goal.position.x = -0.2
    pose_goal.position.y = 0
    UR3.go_to_pose_goal(pose_goal)
    UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

    VAC.Off_grip()
    RG2.move_grip(110)

    UR3.detach_box()

    UR3.remove_box()

    UR3.go_to_joint_state()
    '''

    # #o_size = [0.074, 0.074, 0.1]
    # o_size = [0.074, 0.074, 0.03]
    #
    # print "add box"
    #
    # # Move for picking
    # o_pose = geometry_msgs.msg.Pose()
    # ######################################################################################
    # ### Modify X, Y coordinate Here [m]
    # o_pose.position.x = -0.245
    # o_pose.position.y = 0.296
    # ######################################################################################
    # o_pose.position.z = o_size[2]/2
    # q = quaternion_from_euler(0, 0, 0)
    # o_pose.orientation.x = q[0]
    # o_pose.orientation.y = q[1]
    # o_pose.orientation.z = q[2]
    # o_pose.orientation.w = q[3]
    #
    # UR3.add_box(o_pose, o_size)
    #
    # print "move for picking"
    #
    #
    # # UR3.move_for_picking(o_pose)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x = o_pose.position.x
    # pose_goal.position.y = o_pose.position.y
    # pose_goal.position.z = o_size[2] + 0.12
    # q = quaternion_from_euler(-1.5707, 1.5707, 1.5707)  # Euler ==> Quaternnion
    # pose_goal.orientation.x = q[0]
    # pose_goal.orientation.y = q[1]
    # pose_goal.orientation.z = q[2]
    # pose_goal.orientation.w = q[3]
    #
    # UR3.go_to_pose_goal(pose_goal)
    #
    # UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)

    # grasping_point.position.z -= 10

    # print "going down for picking"
    # raw_input()
    #
    # UR3.remove_box()
    #
    # pose_goal.position.z -= 0.135
    # UR3.go_to_pose_goal(pose_goal)
    # UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
    #
    # UR3.add_box(o_pose, o_size)
    # UR3.attach_box()
    #
    # RG2.move_grip(o_size[0]-0.002)
    # VAC.On_grip()
    #
    # rospy.sleep(1)
    #
    # # UR3.move_for_placing()
    # place_goal = group.get_current_pose().pose
    # place_goal.position.z += 0.12
    #
    # UR3.go_to_pose_goal(place_goal)
    # UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
    #
    # place_goal.position.x = -0.35
    # place_goal.position.y = 0
    # UR3.go_to_pose_goal(place_goal)
    # UR3.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4)
    #
    # VAC.Off_grip()
    # RG2.move_grip(110)
    #
    # rospy.sleep(1)
    #
    # UR3.detach_box()
    #
    # UR3.remove_box()
    #
    # UR3.go_to_joint_state()


    #print "============ Press `Enter` to execute a movement using a joint goal ..."
    #raw_input()
    #UR3.go_to_joint_state()

    #print "============ Press `Enter` to execute a feedback graping  ..."
    #raw_input()
    RG2.feedback_grip()

    #print "============ Press `Enter` to execute a IO Setting ..."
    #raw_input()
    #VAC.On_grip()

    print "============ Python tutorial demo complete!"


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

