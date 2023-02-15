#!/usr/bin/env python

from math import pi
import sys

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list

## ============================================================
class MyMoveGroup( object):

	## ----------------------------------------
	def __init__( self, argv, _szGroup='panda_arm'):
		super( MyMoveGroup, self).__init__()

		## First initialize `moveit_commander`_ and a `rospy`_ node:
		moveit_commander.roscpp_initialize( argv)
		rospy.init_node( 'my_move_group', anonymous=False)

		## Instantiate a `RobotCommander`_ object. This object is the
		## outer-level interface to the robot:
		self.robot = moveit_commander.RobotCommander()

		## Instantiate a `PlanningSceneInterface`_ object. This object is an
		## interface to the world surrounding the robot:
		self.scene = moveit_commander.PlanningSceneInterface()

		## Instantiate a `MoveGroupCommander`_ object.
		## This object is an interface to one group of joints.
		## Set this value to the name of your robot arm planning group.
		## This interface can be used to plan and execute motions on the robot:
		self.group_name = _szGroup
		self.group = moveit_commander.MoveGroupCommander( self.group_name)

		## Create a `DisplayTrajectory`_ publisher which is used later to
		## publish trajectories for RViz to visualize:
		self.display_trajectory_publisher = rospy.Publisher(
			'/move_group/display_planned_path',
			moveit_msgs.msg.DisplayTrajectory,
			queue_size=20)

	## ----------------------------------------
	def run1( self, _bVerbose=False):

		# Can get the name of the reference frame for this robot:
		planning_frame = self.group.get_planning_frame()
		print( '# Reference frame: "{}"'.format( planning_frame))

		# Can also print the name of the end-effector link for this group:
		eef_link = self.group.get_end_effector_link()
		print( '# End effector: "{}"'.format( eef_link))

		# Can get a list of all the groups in the robot:
		print( '# Robot Groups: {}'.format( self.robot.get_group_names()))

		_sRet = None

		if not _bVerbose:
			return _sRet

		# Sometimes for debugging it is useful to print the entire state of the
		# robot:
		_sRet = self.robot.get_current_state()
		print( '# Robot state (isA {}):'.format( type( _sRet)))
		print( _sRet)
		print()

		return _sRet


	## ----------------------------------------
	def display_trajectory( self, plan, _stateLast=None):
		## You can ask RViz to visualize a plan (aka trajectory) for you.
		## But the group.plan() method does this automatically so this is not
		## that useful here (it just displays the same trajectory again):
		##
		## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start
		## and trajectory.
		## Populate the trajectory_start with current robot state to copy over
		## any AttachedCollisionObjects and add plan to the trajectory.
		_display_traj = moveit_msgs.msg.DisplayTrajectory()

		if _stateLast is None:
			_display_traj.trajectory_start = self.robot.get_current_state()
		else:
			_display_traj.trajectory_start = _stateLast


		_display_traj.trajectory.append( plan)

		# Publish
		self.display_trajectory_publisher.publish( _display_traj);


	## ----------------------------------------
	def goToNamedTarget( self, _szTarget='low_full_left_level'):
		# TODO:

		self.group.set_named_target( _szTarget)
		self.group.plan()
		self.group.go( wait=True)

		return

		_pszJoint = self.group.get_joints()
		print( '# get_joints() = {}'.format( _pszJoint))


	## ----------------------------------------
	def getNamedTargetJointState( self, _pfRet, _szTarget):

		_pszTargetNamed = self.group._g.get_named_targets()
		print( '# get_named_targets() [{}] = {}'.format(
			len( _pszTargetNamed), _pszTargetNamed))

#		_pfRet.clear()
		_pfRet[:] = []

		_pfTargetNamedJointVal = self.group._g.get_named_target_values(
			_szTarget)

		print( '# get_named_target_values( "{}") [{}] = {{'.format( _szTarget,
			len( _pfTargetNamedJointVal)))

		for _k, _v in _pfTargetNamedJointVal.items():
			print( '#	 {}: {}'.format( _k, _v))

		print( '# }')

		_pszJoint = self.group.get_joints()
		print( '# get_joints() = _pszJoint[{}] = {}'.format( len( _pszJoint),
			_pszJoint))

		# We can get the joint values from the group and adjust some of the
		# values:
		_pfRet[:] = self.group.get_current_joint_values()

		for _i, _szJ in enumerate( _pszJoint):
			if not _szJ in _pfTargetNamedJointVal:
				continue
			_pfRet[ _i] = _pfTargetNamedJointVal.get( _szJ)


	## ----------------------------------------
	def genPlanToNamedTarget( self, _szTarget, _szSource=''):

		_sTmp = self.robot.get_current_state()

		if _szSource:
			_pfNamedJointVal = self.group._g.get_named_target_values(
				_szSource)

			_pfNew = [0.0] * len( _sTmp.joint_state.name)
			print( '# WTF: type( .joint_state.position) = {} [{}]'.format(
				type( _sTmp.joint_state.position),
				len( _sTmp.joint_state.position)))

			for _i, _szJ in enumerate( _sTmp.joint_state.name):
				_bToSet = (_szJ in _pfNamedJointVal)

				print( '# [{}]: "{}"{}'.format( _i, _szJ,
					('',' - skip')[_bToSet]))

				if not _bToSet:
					continue

				_pfNew[ _i] = _pfNamedJointVal.get( _szJ)

			_sTmp.joint_state.position = _pfNew

		self.group.set_start_state( _sTmp)

		_pfTargetJointVal = self.group._g.get_named_target_values( _szTarget)
		_jsTarget = JointState()
		_jsTarget.name = _pfTargetJointVal.keys()
		_jsTarget.position = [ _pfTargetJointVal.get( _k)
			for _k in _jsTarget.name ]

		print( '# WTF-1a: {}'.format( _pfTargetJointVal))
		print( '# WTF-1b: {}'.format( type( _pfTargetJointVal)))
		print( '# WTF-1c: {}'.format(
			self.group.get_remembered_joint_values()))
		print( '# WTF-1d: {}'.format( _jsTarget))

		assert( len( _pfTargetJointVal))

		return _sTmp, self.group.plan( _jsTarget)
		return _sTmp, self.group.plan( _pfTargetJointVal)


	## ----------------------------------------
	def goToNamedTargetJointState( self, _szTarget='low_full_left_level'):

#		self.group.set_start_state_to_current_state()

		_pfJointGoal = []
		self.getNamedTargetJointState( _pfJointGoal, _szTarget)

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the
		# group
		self.group.go( _pfJointGoal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		self.group.stop()

#		self.group.set_start_state( _pfJointGoal)


	## ----------------------------------------
	def run2( self, _bExec=False, _bVerbose=False):

		print( 'Press ENTER ...')
		if sys.version_info.major == 2:
			## Python 2
			raw_input()
		else:
			## Python 3
			input()

		_pszTargetNamed = self.group._g.get_named_targets()
		print( '# get_named_targets() [{}] = {}'.format(
			len( _pszTargetNamed), _pszTargetNamed))

		_pszPoseSeq = [
			'low_full_left_level',
#			'low_half_left_level',
#			'low_flat_centre_level',
			'low_full_right_150_level',
#			'low_half_centre_level',
			'low_flat_left_level'
		]

		_szFrom = ''
		_nPose = len( _pszPoseSeq)

		for _i, _szTo in enumerate( _pszPoseSeq):
			print( '# Path {} of {} - to: "{}"'.format( _i, len( _pszPoseSeq),
				_szTo))

			assert( _szTo in _pszTargetNamed)

			_stateFrom, _path = self.genPlanToNamedTarget( _szTo, _szFrom)
			_szFrom = _szTo

			if type( _path) == tuple:
				## Noetic move_group::plan() returns tuple:
				## - success flag : boolean
				## - trajectory message : RobotTrajectory
				## - planning time : float
				## - error code : MoveitErrorCodes
				assert( _path[0])
				_path = _path[1]

#			print( '# _stateFrom: {}'.format( _stateFrom))
#			print( '# _path = {}'.format( _path))
			print( '# Path [{}]: points[ {}]'.format( _i,
				len( _path.joint_trajectory.points)))

			self.display_trajectory( _path, _stateFrom)

			if _bExec:
#				self.group.execute( _path)
				self.goToNamedTargetJointState( _szTo)

			if (_i + 1 < _nPose):
				rospy.sleep( 1)

	## ----------------------------------------
	@classmethod
	def main( cls, argv):
		mvGrpObj = MyMoveGroup( argv, 'ur5_arm')

#		mvGrpObj.run1( True)
		mvGrpObj.run1( False)

		mvGrpObj.run2( True)
#		mvGrpObj.run2()

		return 0

#		mvGrpObj.goToNamedTarget()

#		mvGrpObj.goToNamedTargetJointState( 'ready')
#		mvGrpObj.goToNamedTargetJointState( 'low_full_left_level')

		mvGrpObj.goToNamedTargetJointState( 'low_full_right_150_level')
		mvGrpObj.goToNamedTargetJointState( 'low_flat_right_level')

		return 0

## ====================
if __name__ == '__main__':
	import sys
	sys.exit( MyMoveGroup.main( sys.argv))

