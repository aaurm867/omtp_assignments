#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from omtp_gazebo.msg import LogicalCameraImage
from omtp_gazebo.srv import VacuumGripperControl
import tf2_ros
import tf2_geometry_msgs


def logical_camera2_callback(data):
    global Object_position
    # Check if the logical camera has seen our box which has the name 'object'.
    if (data.models[-1].type == 'object'):
        # Create a pose stamped message type from the camera image topic.
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.stamp = rospy.Time.now()
        object_pose.header.frame_id = "logical_camera_2_frame"  # "logical_camera_2_link"
        object_pose.pose.position.x = data.models[-1].pose.position.x
        object_pose.pose.position.y = data.models[-1].pose.position.y
        object_pose.pose.position.z = data.models[-1].pose.position.z
        object_pose.pose.orientation.x = data.models[-1].pose.orientation.x
        object_pose.pose.orientation.y = data.models[-1].pose.orientation.y
        object_pose.pose.orientation.z = data.models[-1].pose.orientation.z
        object_pose.pose.orientation.w = data.models[-1].pose.orientation.w
        while True:
            try:
                object_robot2_frame = tf_buffer.transform(object_pose, "world")
                Object_position=object_robot2_frame
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        # Gracefully terminate the ROS node after transforming the pose.
        #rospy.signal_shutdown('Successfully transformed pose.')
    else:
        # Do nothing.
        print('')


Object_position=None

Z_offset=0.2
if __name__ == '__main__':


    # Initialize ROS node
    rospy.init_node('lecture6_assignment4', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    # Create a TF buffer in the global scope
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Subscribe to the logical camera topic.
    rospy.Subscriber('omtp/logical_camera_2',LogicalCameraImage, logical_camera2_callback)

    # MoveGroup Commander Object for robot2.
    robot2_group = moveit_commander.MoveGroupCommander("robot2")

    # Action clients to the ExecuteTrajectory action server.
    robot2_client = actionlib.SimpleActionClient('execute_trajectory',
                                                 moveit_msgs.msg.ExecuteTrajectoryAction)
    robot2_client.wait_for_server()
    rospy.loginfo('Execute Trajectory server is available for robot2')

    # go to pregrasp position
    robot2_group.set_named_target("R2PreGrasp")

    plan_pregrasp=robot2_group.plan()
    goal=moveit_msgs.msg.ExecuteTrajectoryGoal()
    goal.trajectory=plan_pregrasp

    robot2_client.send_goal(goal)
    robot2_client.wait_for_result()

    rospy.loginfo('finished first movement')

    #Go to Object
    current_pose = robot2_group.get_current_pose()
    rospy.loginfo(current_pose)
    above_object_pose = geometry_msgs.msg.Pose()
    above_object_pose.position.x =Object_position.pose.position.x
    above_object_pose.position.y =Object_position.pose.position.y
    above_object_pose.position.z =Object_position.pose.position.z+Z_offset

    rospy.loginfo(above_object_pose)

    # Retain orientation of the current pose.
    above_object_pose.orientation = copy.deepcopy(current_pose.pose.orientation)

    robot2_group.set_pose_target(above_object_pose)

    plan=robot2_group.plan()
    goal=moveit_msgs.msg.ExecuteTrajectoryGoal()
    goal.trajectory=plan

    robot2_client.send_goal(goal)
    robot2_client.wait_for_result()

    #carthesion downward movement
    on_object_pose=copy.deepcopy(above_object_pose)
    on_object_pose.position.z-=Z_offset/2
    waypoints=[]
    waypoints.append(above_object_pose)
    waypoints.append(on_object_pose)

    rospy.loginfo('created waypoints')

    fraction = 0.0
    for count_cartesian_path in range(0,3):
        if fraction < 1.0:
            (plan_cartesian, fraction) = robot2_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        else:
            break
    
    rospy.loginfo('planned second movement')

    Setgripper=rospy.ServiceProxy('/gripper2/control',VacuumGripperControl)
    Setgripper(True)

    robot2_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot2_goal.trajectory = plan_cartesian
    robot2_client.send_goal(robot2_goal)
    robot2_client.wait_for_result()

    # go to pregrasp position
    robot2_group.set_named_target("R2PreGrasp")

    plan_pregrasp=robot2_group.plan()
    goal=moveit_msgs.msg.ExecuteTrajectoryGoal()
    goal.trajectory=plan_pregrasp

    robot2_client.send_goal(goal)
    robot2_client.wait_for_result()

    # go to high position
    robot2_group.set_named_target("R2High")

    plan_high=robot2_group.plan()
    goal=moveit_msgs.msg.ExecuteTrajectoryGoal()
    goal.trajectory=plan_high

    robot2_client.send_goal(goal)
    robot2_client.wait_for_result()

    #go to bin
    robot2_group.set_named_target("R2Place")
    plan_bin=robot2_group.plan()
    goal=moveit_msgs.msg.ExecuteTrajectoryGoal()
    goal.trajectory=plan_bin

    robot2_client.send_goal(goal)
    robot2_client.wait_for_result()

    Setgripper(False)






    rospy.spin()
