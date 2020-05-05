#!/usr/bin/env python

import rospy
from omtp_gazebo.msg import LogicalCameraImage
from omtp_gazebo.srv import ConveyorBeltControl
from gazebo_msgs.srv import SpawnModel
from omtp_gazebo.msg import ConveyorBeltState
from omtp_gazebo.srv import VacuumGripperControl
from std_srvs.srv import Empty
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib


def plan_and_execute_named_pose(named_pose):
    global robot1_client, robot1_group

    robot1_group.set_named_target(named_pose)
    dir(robot1_group)
    plan = robot1_group.plan()
    goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    goal.trajectory = plan

    robot1_client.send_goal(goal)
    robot1_client.wait_for_result()


def plan_and_execute_pose(goal_pose):
    global robot1_client, robot1_group

    robot1_group.set_pose_target(goal_pose)

    plan = robot1_group.plan()
    goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    goal.trajectory = plan

    robot1_client.send_goal(goal)
    robot1_client.wait_for_result()


def plan_and_execute_pose_cartesian(goal_pose):
    global robot1_client, robot1_group

    waypoints = [goal_pose]

    fraction = 0.0
    plan_cartesian = None
    for count_cartesian_path in range(0, 15):
        if fraction < 1.0:
            (plan_cartesian, fraction) = robot1_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0)  # jump_threshold
        else:
            break
    robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot1_goal.trajectory = plan_cartesian
    rospy.loginfo('Cartesian movement planned with fraction %f' % fraction)
    robot1_client.send_goal(robot1_goal)
    robot1_client.wait_for_result()


def logical_camera1_callback(data):
    global robot1_client, robot1_group, current_pose, spawn_object_once, conveyor_control, stop_spawn, moving, gripper_control
    # Check if the logical camera has seen our box which has the name 'object'.
    # print(data)
    if (data.models[-1].type == 'object'):
        print('Object detected')
        # Create a pose stamped message type from the camera image topic.
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.stamp = rospy.Time.now()
        object_pose.header.frame_id = "logical_camera_1_frame"
        object_pose.pose.position.x = data.models[-1].pose.position.x
        object_pose.pose.position.y = data.models[-1].pose.position.y
        object_pose.pose.position.z = data.models[-1].pose.position.z
        object_pose.pose.orientation.x = data.models[-1].pose.orientation.x
        object_pose.pose.orientation.y = data.models[-1].pose.orientation.y
        object_pose.pose.orientation.z = data.models[-1].pose.orientation.z
        object_pose.pose.orientation.w = data.models[-1].pose.orientation.w
        while True:
            try:
                object_world_pose = tf_buffer.transform(object_pose, "world")
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        if moving:
            rospy.loginfo('Belt is moving, stopping belt and moving to pregrasp')
            conveyor_control(ConveyorBeltState(0))
            stop_spawn()
            moving = False
            plan_and_execute_named_pose("R1PreGrasp")

            # start with the current pose
            current_pose = robot1_group.get_current_pose()
            rospy.sleep(0.5)
            current_pose = robot1_group.get_current_pose()
            print(current_pose.pose.orientation)
            rospy.sleep(0.5)
            return
        rospy.loginfo('Pose of the object in the world reference frame is: %s', object_world_pose)
        # rospy.loginfo('Pose of the object in the reference framecamera of logical camera2 is: %s', object_pose)

        rospy.loginfo('Belt is stopped, planning a movement towards the object')

        above_object = geometry_msgs.msg.Pose()
        above_object.position = copy.deepcopy(object_world_pose.pose.position)
        above_object.position.z += 0.2
        above_object.orientation = copy.deepcopy(current_pose.pose.orientation)

        on_object = copy.deepcopy(above_object)
        on_object.position.z -= 0.05

        plan_and_execute_pose(above_object)
        plan_and_execute_pose_cartesian(on_object)
        gripper_control(True)
        plan_and_execute_pose_cartesian(above_object)

        rospy.loginfo('Object picked up, going to home')
        # drop_point = geometry_msgs.msg.Pose()
        # drop_point.orientation = copy.deepcopy(current_pose.pose.orientation)
        # while True:
        #     try:
        #         trans = tf_buffer.lookup_transform('bin_2_drop_point', 'world', rospy.Time(0))
        #         print(trans.transform.translation)
        #         drop_point.position = copy.deepcopy(trans.transform.translation)
        #         break
        #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #         continue

        plan_and_execute_named_pose("R1Home")
        rospy.sleep(2)
        rospy.loginfo('Going from home to R1Place')
        rospy.sleep(2)
        plan_and_execute_named_pose("R1Place")

        rospy.loginfo('At drop point, dropping object and going back')
        gripper_control(False)
        plan_and_execute_named_pose("R1Home")

        rospy.loginfo('At home, restarting belt')
        spawn_object_once()
        conveyor_control(ConveyorBeltState(100))

        current_pose = robot1_group.get_current_pose()
        rospy.sleep(0.5)
        current_pose = robot1_group.get_current_pose()
        rospy.sleep(0.5)
        moving = True

        # Gracefully terminate the ROS node after transforming the pose.
        # rospy.signal_shutdown('Successfully transformed pose.')
    # else:
    # Do nothing.
    # print('')


current_pose = None
robot1_client = None
robot1_group = None
spawn_object_once = None
stop_spawn = None
conveyor_control = None
gripper_control = None
moving = None
pose = "R1PreGrasp"

if __name__ == '__main__':
    print('Starting')
    # Initialize ROS node to transform object pose.
    rospy.init_node('lecture6_assignment4', anonymous=True)

    spawn_object_once = rospy.ServiceProxy('/spawn_object_once', Empty)
    spawn_object_once()

    stop_spawn = rospy.ServiceProxy('/stop_spawn', Empty)

    gripper_control = rospy.ServiceProxy('/gripper1/control', VacuumGripperControl)

    conveyor_control = rospy.ServiceProxy('/omtp/conveyor/control', ConveyorBeltControl)
    conveyor_control(ConveyorBeltState(100))
    moving = True

    print('Initialized Node')
    # Create a TF buffer in the global scope
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    print('Created Buffer')

    moveit_commander.roscpp_initialize(sys.argv)

    robot1_group = moveit_commander.MoveGroupCommander("robot1")
    robot1_client = actionlib.SimpleActionClient('execute_trajectory',
                                                 moveit_msgs.msg.ExecuteTrajectoryAction)
    robot1_client.wait_for_server()
    print('Execute Trajectory server is available for robot1')

    plan_and_execute_named_pose("R1Home")

    # start with the current pose
    current_pose = robot1_group.get_current_pose()
    rospy.sleep(0.5)
    current_pose = robot1_group.get_current_pose()

    # Subscribe to the logical camera topic.
    rospy.Subscriber('omtp/logical_camera_1/', LogicalCameraImage, logical_camera1_callback, queue_size=1)

    print('Subscribed')
    rospy.spin()
