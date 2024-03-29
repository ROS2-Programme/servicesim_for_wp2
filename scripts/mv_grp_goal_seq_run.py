#!/usr/bin/env python

from math import pi, fabs, sqrt, cos

import rospy
# from moveit_msgs.msg import RobotTrajectory
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

from moveit_commander.conversions import pose_to_list

from mv_grp_base import MyMoveGroup

## ============================================================
class MyMoveGroupGoalTrajRunner( MyMoveGroup):

	## ----------------------------------------
	def __init__( self, argv, _szGroup='MobileManipulator',
			_szNodeName='my_goal_seq_runner', _bAnon=False,
			_szGoalTrajTopic='wtf/whole_body', _fTolerance=0.01,
			_fToleranceOrient=None):

		super( MyMoveGroupGoalTrajRunner, self).__init__( argv, _szGroup,
			_szNodeName, _bAnon=None)

		if _fToleranceOrient is None:
			_fToleranmceOrient = cos( _fTolerance / 2.0)

		self.fTolerance = _fTolerance
		self.fToleranceCosPhiHalf = _fToleranceOrient

		self.pub4RobotTraj = rospy.Publisher( _szGoalTrajTopic,
			moveit_msgs.msg.RobotTrajectory, queue_size=1)

		rospy.init_node( _szNodeName, anonymous=_bAnon)


	## --------------------
	def __del__( self):
		print( '# {}::__del__(): {} connection(s) to topic "{}".'.format(
			type( self).__name__, self.pub4RobotTraj.get_num_connections(),
			self.pub4RobotTraj.resolved_name))

		if self.pub4RobotTraj.get_num_connections() == 0:
			print( ('# {}::__del__(): no connections to topic - ' +
				'safe to unregister()?').format( type( self).__name__))
			self.pub4RobotTraj.unregister()
			self.pub4RobotTraj = None

		super( MyMoveGroupGoalTrajRunner, self).__del__()


	## ----------------------------------------
	@classmethod
	def dist( cls, _pfA, _pfB):

		return sqrt( sum( pow( _a - _b, 2)) for _a, _b in zip( _pfA, _pfB))

	## ----------------------------------------
	@classmethod
	def mult( cls, _pfR, _pfC):

		return sum( _r * _c for _r, _c in zip( _pfR, _pfC))


	## ----------------------------------------
	## Modified from Move Group Python Interface tutorial
	## References:
	## https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
	## https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
	## ----------------------------------------
	def all_close( self, _goal, _actual, _fTolerance=0.001,
			_fToleranceCosPhiHalf=None, _pfErrRet=None, _bDebug=False):

		pass

		## TODO:

		"""
		Convenience method for testing if the values in two lists are within a
		tolerance of each other.
		For Pose and PoseStamped inputs, the angle between the two quaternions
		is compared (the angle between the identical orientations q and -q is
		calculated correctly).
		@param: goal       A list of floats, a Pose or a PoseStamped
		@param: actual     A list of floats, a Pose or a PoseStamped
		@param: tolerance  A float
		@returns: bool
		"""

		if _fToleranceCosPhiHalf is None:
			_fToleranceCosPhiHalf = cos( _fTolerance / 2.0)

		_bWithStat = (_pfErrRet is not None) and (type( _pfErrRet) is list)

		_bRet = True
		_fErr = 0.0

		if type( _goal) is list:
			if _bDebug:
				print( '# ac(): 1a) list')
			for index in range( len( _goal)):
				_fErr = abs( _actual[ index] - _goal[ index])
				if _fErr > _fTolerance:
					_bRet = False
					if not _bWithStat:
						break
				if _bWithStat:
					_pfErrRet.append( _fErr)

			if _bDebug:
				print( '# ac(): _bRet = {}'.format( _bRet))
			return _bRet

		elif type( _goal) is PoseStamped:
			if _bDebug:
				print( '# ac(): 1b) {}'.format( type( _goal).__name__))
			return self.all_close( _goal.pose, _actual.pose, _fTolerance,
				_fToleranceCosPhiHalf, _pfErrRet, _bDebug)

		elif type( _goal) is Pose:
			if _bDebug:
				print( '# ac(): 1c) {}'.format( type( _goal).__name__))
			_pfPxyzOxyzw0 = pose_to_list( _actual)
			_pfPxyzOxyzw1 = pose_to_list( _goal)

			## Euclidean distance
			_fErr = self.dist( _pfPxyzOxyzw0[:3], _pfPxyzOxyzw1[:3])
			if _fErr > _fTolerance:
				_bRet = False
				if not _bWithStat:
					return _bRet
				_pfErrRet.append( _fErr)

			## phi = angle between orientations
			_fErr = fabs( self.mult( _pfPxyzOxyzw0[3:], _pfPxyzOxyzw1[3:]))
			if _fErr < _fToleranceCosPhiHalf:
				if not _bWithStat:
					return _bRet
				_pfErrRet.append( _fErr)

			if _bDebug:
				print( '# ac(): _bRet = {}'.format( _bRet))
			return _bRet

		elif type( _goal) is JointState:
			if _bDebug:
				print( '# ac(): 1d) {}'.format( type( _goal).__name__))
			return self.all_close( _goal.position, _actual.position,
				_fTolerance, _fToleranceCosPhiHalf, _pfErrRet, _bDebug)

		else:
			assert( False), 'No handling of ' + str( type( _goal))

		if _bDebug:
			print( '# ac(): _bRet = TRUE')
		return True


	## ----------------------------------------
	def waitForGoalStateMatch( self, _jsTarget, _fMaxTime=10.0,
			_fInterval=1.0, _pfErrRet=None):

		_bInfinite = (_fMaxTime <= 0.0)
		_fMaxDuration = (0.0, _fMaxTime)[ _bInfinite]

		_dSleep = rospy.Duration.from_sec( _fInterval)

		if _bInfinite:
			_tmLimit = rospy.Time.now()
		else:
			_tmLimit = rospy.Time.now() + rospy.Duration.from_sec(
				_fMaxDuration)

		_tmNow = rospy.Time.now()

		print( ('# wFGSM(): waiting for current state to match goal state ' +
			'postion: {}').format( str( _jsTarget.position)))

		_s = 0

		while (not self.all_close( _jsTarget, getattr(
						self.robot.get_current_state(), 'joint_state'),
					self.fTolerance, self.fToleranceCosPhiHalf, None,
					not _bInfinite)) and (
				_bInfinite or (_tmNow < _tmLimit)):

			rospy.sleep( _dSleep)
			_s += 1
			_tmNow = rospy.Time.now()

		if _tmNow >= _tmLimit:
			print( ('# wFGSM(): timed out after {} cycle(s) (@ {}/{} ' +
				'sampling) ({}/{})').format( _s, _fInterval, _fMaxTime,
					_tmNow, _tmLimit))

		return self.all_close( _jsTarget, getattr(
				self.robot.get_current_state(), 'joint_state'),
			self.fTolerance, self.fToleranceCosPhiHalf, _pfErrRet)


	## ----------------------------------------
	@classmethod
	def main( cls, argv):
		print( '# argv[ {}] = {}'.format( len( argv), str( argv)))

		_fMaxTrajTime = 10

		if len( argv) > 1:
			_fMaxTrajTime = float( argv[ 1])

		mvGrpRun = MyMoveGroupGoalTrajRunner( argv, 'MobileManipulator')

		_pszPoseSeq = ['Upright', 'ForwardGrabbing', 'ZeroPose']
		_szPoseLast = None
		_jsLast = JointState()
		_pfErr = []
		_bReachedLast = False

		for _i, _szPoseNext in enumerate( _pszPoseSeq):
			if not _szPoseLast is None:
				_pfErr = []
				_bReachedLast = mvGrpRun.waitForGoalStateMatch( _jsLast,
					_fMaxTrajTime, _pfErrRet=_pfErr)

				if not _bReachedLast:
					print( '# _pfErr = {}'.format( str( _pfErr)))
					break

			print( ('# Attempting to generate plan to named target [{}] "{}"'
				).format( _i, _szPoseNext))

			_stateFrom, _path = mvGrpRun.genPlanToNamedTarget( _szPoseNext,
				_szPoseLast, True, _jsLast)

			mvGrpRun.displayTrajectory( _path, _stateFrom)
			mvGrpRun.pub4RobotTraj.publish( _path)

			_szPoseLast = _szPoseNext

		mvGrpRun.displayTrajectory( _szPoseLast)



## ====================
if __name__ == '__main__':
	import sys
	sys.exit( MyMoveGroupGoalTrajRunner.main( sys.argv))

