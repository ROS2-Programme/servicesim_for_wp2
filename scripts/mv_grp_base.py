#!/usr/bin/env python

# from math import pi
# import sys

import rospy
import moveit_commander
import moveit_msgs.msg
# import geometry_msgs.msg

# from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# from moveit_commander.conversions import pose_to_list

## ============================================================
class MyMoveGroup( object):

	## ----------------------------------------
	def __init__( self, argv, _szGroup='panda_arm',
			_szNodeName='my_move_group', _bAnon=False,
			_szTrajVisTopic='/move_group/display_planned_path',
			_nTrajVisTopicQ=1, _bLatch=False):

		super( MyMoveGroup, self).__init__()

		## First initialize `moveit_commander`_ and a `rospy`_ node:
		moveit_commander.roscpp_initialize( argv)
		rospy.init_node( _szNodeName, anonymous=_bAnon)

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
			_szTrajVisTopic, moveit_msgs.msg.DisplayTrajectory,
			queue_size=_nTrajVisTopicQ, latch=_bLatch)


	## --------------------
	def __del__( self):
		print( '# __del__()')
		moveit_commander.roscpp_shutdown()
#		while not rospy.is_shutdown():
#			rospy.spin()

	## ----------------------------------------
	def displayTrajectory( self, plan, _stateLast=None):
		## You can ask RViz to visualize a plan (aka trajectory) for you.
		## But the group.plan() method does this automatically so this is not
		## that useful here (it just displays the same trajectory again):
		##
		## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start
		## and trajectory.
		## Populate the trajectory_start with current robot state to copy over
		## any AttachedCollisionObjects and add plan to the trajectory.
		_display_traj = moveit_msgs.msg.DisplayTrajectory()
		_display_traj.model_id = 'husky'

		if _stateLast is None:
			_display_traj.trajectory_start = self.robot.get_current_state()
		else:
			_display_traj.trajectory_start = _stateLast


		if type( plan) == str:
			print( '# WTF special "{}"'.format( plan))
			_jt = JointTrajectory()
#			_jt.header.frame_id = '/world'
			_jt.header.frame_id = _display_traj.trajectory_start.joint_state.header.frame_id
			_jt.points.append( JointTrajectoryPoint())
			self.getNamedTargetJointState(
				_jt.points[0].positions, plan, _jt.joint_names)
			_pfZero = [ 0.0 ] * len( _jt.joint_names)
			_jt.points[0].velocities = _pfZero
			_jt.points[0].accelerations = _pfZero
#			_jt.points[0].effort = _pfZero
			_display_traj.trajectory.append( moveit_msgs.msg.RobotTrajectory())
			_display_traj.trajectory[0].joint_trajectory = _jt

			print( _display_traj)
		else:
			_display_traj.trajectory.append( plan)

		# Publish
		self.display_trajectory_publisher.publish( _display_traj);

	## ----------------------------------------
	def callPlan( self, _jsTarget=None, _bRetTrajOnly=False,
			_jsStart=None):

		if not _jsStart:
			if _jsStart is None:
				pass
			else:
				self.group.set_start_state_to_current_state()
		else:
			self.group.set_start_state( _jsStart)

		_path = self.group.plan( _jsTarget)

		if _bRetTrajOnly and (type( _path) == tuple):
			## Noetic move_group::plan() returns tuple:
			## - success flag : boolean
			## - trajectory message : RobotTrajectory
			## - planning time : float
			## - error code : MoveitErrorCodes
			assert( _path[0])
			_path = _path[1]

#		if _bDropStart:
#			print( '# Org: _path.joint_trajectory.points[ {}]'.format(
#				len( _path.joint_trajectory.points)))
#			_path.joint_trajectory.points[ :1] = []
#			print( '# Rev: _path.joint_trajectory.points[ {}]'.format(
#				len( _path.joint_trajectory.points)))

		return _path

	## ----------------------------------------
	def goToNamedTarget( self, _szTarget='low_full_left_level'):
		# TODO:

		self.group.set_named_target( _szTarget)

		_path = self.callPlan( None, True)
#		_path = self.callPlan( None, True, True)
		self.group.execute( _path, wait=True)
#		self.group.plan()
#		self.group.go( wait=True)

		self.group.stop()

		return

		_pszJoint = self.group.get_joints()
		print( '# get_joints() = {}'.format( _pszJoint))

	## ----------------------------------------
	def getNamedTargetJointState( self, _pfRet, _szTarget, _pszRet=None):

		_pszTargetNamed = self.group._g.get_named_targets()
		print( '# get_named_targets() [{}] = {}'.format(
			len( _pszTargetNamed), _pszTargetNamed))

#		_pfRet.clear()
		_pfRet[:] = []

		_bRecJoint = (_pszRet is not None)
		if _bRecJoint:
			_pszRet[:] = []

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

		print( '# _pfRet[ {}] = {}'.format( len( _pfRet), str( _pfRet)))

		_i = 0

		for _szJ in _pszJoint:
			if not _szJ in _pfTargetNamedJointVal:
				continue
			_pfRet[ _i] = _pfTargetNamedJointVal.get( _szJ)
			if _bRecJoint:
				_pszRet.append( _szJ)
			_i += 1

		if _bRecJoint:
			print( '# _pszRet[ {}] = {}'.format( len( _pszRet), str( _pszRet)))


	## ----------------------------------------
	def genPlanToNamedTarget( self, _szTarget, _szSource='',
			_bRetTrajOnly=False):

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

		return _sTmp, self.callPlan( _jsTarget, _bRetTrajOnly)
		return _sTmp, self.callPlan( _jsTarget, _bRetTrajOnly, True)
		return _sTmp, self.group.plan( _jsTarget)
		return _sTmp, self.group.plan( _pfTargetJointVal)


	## ----------------------------------------
	def goToNamedTargetJointState( self, _szTarget='low_full_left_level'):

#		self.group.set_start_state_to_current_state()

		_pfJointGoal = []
		self.getNamedTargetJointState( _pfJointGoal, _szTarget)

		print( '# _pfJointGoal[ {}] = [ {} ]'.format( len( _pfJointGoal),
			str( _pfJointGoal)))

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the
		# group
		_path = self.callPlan( _pfJointGoal, True)
#		_path = self.callPlan( _pfJointGoal, True, True)
		self.group.execute( _path, wait=True)
#		self.group.go( _pfJointGoal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		self.group.stop()

#		self.group.set_start_state( _pfJointGoal)


	## ----------------------------------------
	@classmethod
	def main( cls, argv):
		print( '# argv[ {}] = {}'.format( len( argv), str( argv)))

		_nOpt = 1

		if len( argv) > 1:
			_nOpt = int( argv[ 1])

		mvGrpObj = MyMoveGroup( argv, 'MobileManipulator')

#		return 0
		if _nOpt == 0:
			mvGrpObj.displayTrajectory( 'ZeroPose')

		if _nOpt == 1:
#			mvGrpObj.goToNamedTarget()
			mvGrpObj.goToNamedTarget( 'Upright')
			mvGrpObj.goToNamedTarget( 'ForwardGrabbing')
			mvGrpObj.goToNamedTarget( 'ZeroPose')

			mvGrpObj.displayTrajectory( 'ZeroPose')

#		mvGrpObj.goToNamedTargetJointState( 'ready')
#		mvGrpObj.goToNamedTargetJointState( 'low_full_left_level')

		if _nOpt == 2:
			mvGrpObj.goToNamedTargetJointState( 'Upright')
			mvGrpObj.goToNamedTargetJointState( 'ForwardGrabbing')
			mvGrpObj.goToNamedTargetJointState( 'ZeroPose')

		return 0

## ====================
if __name__ == '__main__':
	import sys
	sys.exit( MyMoveGroup.main( sys.argv))

