#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from omtp_factory_flexbe_states.srdf_state_to_moveit import SrdfStateToMoveit as omtp_factory_flexbe_states__SrdfStateToMoveit
from omtp_factory_flexbe_states.control_feeder_state import ControlFeederState
from omtp_factory_flexbe_states.set_conveyor_power_state import SetConveyorPowerState
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.decision_state import DecisionState
from omtp_factory_flexbe_states.detect_part_camera_state import DetectPartCameraState
from flexbe_states.log_key_state import LogKeyState
from omtp_factory_flexbe_states.compute_grasp_state import ComputeGraspState
from flexbe_manipulation_states.moveit_to_joints_dyn_state import MoveitToJointsDynState as flexbe_manipulation_states__MoveitToJointsDynState
from flexbe_states.log_state import LogState
from omtp_factory_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu May 07 2020
@author: AAU gr867
'''
class lecture7pickandplaceSM(Behavior):
	'''
	Pick and place objects with robot1 and place it in a bin
	'''


	def __init__(self):
		super(lecture7pickandplaceSM, self).__init__()
		self.name = 'lecture7 pick and place '

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		conveyor_speed = 100
		move_group = "robot1"
		joint_names = ["robot1_elbow_joint", "robot1_shoulder_lift_joint", "robot1_shoulder_pan_joint", "robot1_wrist_1_joint", "robot1_wrist_2_joint", "robot1_wrist_3_joint"]
		# x:1498 y:734, x:90 y:852
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.conveyor_speed = conveyor_speed

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:455, x:130 y:455, x:230 y:455, x:330 y:455
		_sm_grasp_0 = OperatableStateMachine(outcomes=['failed', 'planning_failed', 'control_failed', 'reached'], input_keys=['pose'])

		with _sm_grasp_0:
			# x:33 y:40
			OperatableStateMachine.add('compute grasp',
										ComputeGraspState(group=move_group, offset=0.15, joint_names=joint_names, tool_link="vacuum_gripper1_suction_cup", rotation=3.14),
										transitions={'continue': 'move to grasp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:30 y:146
			OperatableStateMachine.add('move to grasp',
										flexbe_manipulation_states__MoveitToJointsDynState(move_group=move_group, action_topic='/move_group'),
										transitions={'reached': 'reached', 'planning_failed': 'planning_failed', 'control_failed': 'control_failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'joint_values', 'joint_names': 'joint_names'})


		# x:29 y:455, x:130 y:455, x:230 y:455, x:330 y:455
		_sm_pregrasp_position_1 = OperatableStateMachine(outcomes=['failed', 'reached', 'planning_failed', 'control_failed'], input_keys=['pose'])

		with _sm_pregrasp_position_1:
			# x:30 y:40
			OperatableStateMachine.add('compute pregrasp position',
										ComputeGraspState(group=move_group, offset=0.2, joint_names=joint_names, tool_link="vacuum_gripper1_suction_cup", rotation=3.14),
										transitions={'continue': 'move to pregrasp', 'failed': 'log'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:162 y:126
			OperatableStateMachine.add('move to pregrasp',
										flexbe_manipulation_states__MoveitToJointsDynState(move_group=move_group, action_topic='/move_group'),
										transitions={'reached': 'reached', 'planning_failed': 'log_2', 'control_failed': 'control_failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:17 y:201
			OperatableStateMachine.add('log',
										LogState(text="compute pregrasp failed", severity=Logger.REPORT_HINT),
										transitions={'done': 'compute pregrasp position'},
										autonomy={'done': Autonomy.Off})

			# x:220 y:214
			OperatableStateMachine.add('log_2',
										LogState(text="planning pregrasp failed", severity=Logger.REPORT_HINT),
										transitions={'done': 'planning_failed'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:40 y:88
			OperatableStateMachine.add('go home',
										omtp_factory_flexbe_states__SrdfStateToMoveit(config_name="R1Home", move_group="robot1", action_topic='/move_group', robot_name=""),
										transitions={'reached': 'start conveyor', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:477 y:89
			OperatableStateMachine.add('add objects',
										ControlFeederState(activation=True),
										transitions={'succeeded': 'wait', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:242 y:92
			OperatableStateMachine.add('start conveyor',
										SetConveyorPowerState(stop=False),
										transitions={'succeeded': 'add objects', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'speed': 'conveyor_speed'})

			# x:992 y:23
			OperatableStateMachine.add('subscribe to beam',
										SubscriberState(topic="/break_beam_sensor", blocking=True, clear=False),
										transitions={'received': 'decision', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:957 y:111
			OperatableStateMachine.add('decision',
										DecisionState(outcomes=["True","False"], conditions=lambda a: a.object_detected==True),
										transitions={'True': 'stop conveyor', 'False': 'subscribe to beam'},
										autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
										remapping={'input_value': 'message'})

			# x:1193 y:99
			OperatableStateMachine.add('stop conveyor',
										SetConveyorPowerState(stop=True),
										transitions={'succeeded': 'get camera state', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'speed': 'conveyor_speed'})

			# x:1332 y:204
			OperatableStateMachine.add('get camera state',
										DetectPartCameraState(ref_frame="world", camera_topic="/omtp/logical_camera", camera_frame="logical_camera_frame"),
										transitions={'continue': 'log', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:1190 y:269
			OperatableStateMachine.add('log',
										LogKeyState(text="data:{}", severity=Logger.REPORT_HINT),
										transitions={'done': 'pregrasp position'},
										autonomy={'done': Autonomy.Off},
										remapping={'data': 'pose'})

			# x:1346 y:283
			OperatableStateMachine.add('pregrasp position',
										_sm_pregrasp_position_1,
										transitions={'failed': 'failed', 'reached': 'activate gripper', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'failed': Autonomy.Inherit, 'reached': Autonomy.Inherit, 'planning_failed': Autonomy.Inherit, 'control_failed': Autonomy.Inherit},
										remapping={'pose': 'pose'})

			# x:1311 y:391
			OperatableStateMachine.add('activate gripper',
										VacuumGripperControlState(enable=True, service_name="/gripper1/control"),
										transitions={'continue': 'grasp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1166 y:616
			OperatableStateMachine.add('go to bin',
										omtp_factory_flexbe_states__SrdfStateToMoveit(config_name="R1Place", move_group="robot1", action_topic='/move_group', robot_name=""),
										transitions={'reached': 'deactivate gripper', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1206 y:488
			OperatableStateMachine.add('grasp',
										_sm_grasp_0,
										transitions={'failed': 'failed', 'planning_failed': 'failed', 'control_failed': 'failed', 'reached': 'go to home'},
										autonomy={'failed': Autonomy.Inherit, 'planning_failed': Autonomy.Inherit, 'control_failed': Autonomy.Inherit, 'reached': Autonomy.Inherit},
										remapping={'pose': 'pose'})

			# x:1243 y:708
			OperatableStateMachine.add('deactivate gripper',
										VacuumGripperControlState(enable=False, service_name="/gripper1/control"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1024 y:559
			OperatableStateMachine.add('go to home',
										omtp_factory_flexbe_states__SrdfStateToMoveit(config_name="R1Home", move_group="robot1", action_topic='/move_group', robot_name=""),
										transitions={'reached': 'wait2', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:637 y:102
			OperatableStateMachine.add('wait',
										WaitState(wait_time=3),
										transitions={'done': 'stop adding objects'},
										autonomy={'done': Autonomy.Off})

			# x:734 y:94
			OperatableStateMachine.add('stop adding objects',
										ControlFeederState(activation=False),
										transitions={'succeeded': 'subscribe to beam', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1393 y:608
			OperatableStateMachine.add('wait2',
										WaitState(wait_time=1),
										transitions={'done': 'go to bin'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
