#!/usr/bin/env python

import rospy
import moveit_commander

from mv_grp_base import MyMoveGroup

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

## ============================================================
class MyMoveGroupSplitter( MyMoveGroup):

	## ----------------------------------------
	def __init__( self, argv, _szGroup='MobileManipulator',
			_szNodeName='my_traj_splitter', _bAnon=False,
			_szGroupSplitBase='wtf', _szWholeBodyRobotTrajTopic='whole_body',
			_szPartBodyRobotTrajBranch='partial', _pszSubGroupToTopic={
				'HuskyBase': 'traj4base', 'Manipulator': 'traj4arm'}):

		super( MyMoveGroupSplitter, self).__init__( argv, _szGroup,
			_szNodeName, _bAnon=_bAnon)

		if len( _szGroupSplitBase) > 0:
			_szGroupSplitBase = _szGroupSplitBase.rstrip( '/')

			_szWholeBodyRobotTrajTopic = '/'.join( [ _szGroupSplitBase,
				_szWholeBodyRobotTrajTopic ])
			_szPartBodyRobotTrajBranch = '/'.join( [ _szGroupSplitBase,
				_szPartBodyRobotTrajBranch ])

			if _szPartBodyRobotTrajBranch.endswith( '/'):
				_szPartBodyRobotTrajBranch = _szPartBodyRobotTrajBranch.rstrip(
					'/')

		self.nFakeEchoMax = 10
		self.nFakeEcho = 0

		self.pLastRobotTrajPerGroup = {}

		self.pszSubGroup = _pszSubGroupToTopic.keys()

		self.ppnJointIdxPerGroup = {}
		self.ppszJointSeqPerGroup = {}

		self.ppub4RobotTrajPart = {}


		for _szG, _szNS in _pszSubGroupToTopic.items():
			assert( _szG in self.robot.get_group_names())

			self.ppnJointIdxPerGroup[ _szG] = {}
			self.ppszJointSeqPerGroup[
				_szG] = moveit_commander.MoveGroupCommander(
					_szG).get_active_joints()[:]

			for _i, _szJ in enumerate( self.ppszJointSeqPerGroup.get( _szG)):
				print( '# group "{}", joint "{}" @ {}'.format( _szG, _szJ, _i))
				self.ppnJointIdxPerGroup[ _szG][ _szJ] = _i

			self.ppub4RobotTrajPart[ _szG] = rospy.Publisher(
				'/'.join( [ _szPartBodyRobotTrajBranch, _szNS ]),
				RobotTrajectory, queue_size=1, latch=True)

			print( '# Publisher for group "{}" to topic "{}"'.format( _szG,
				self.ppub4RobotTrajPart.get( _szG).resolved_name))


		self.ppnJointIdxPerGroup[ _szGroup] = {}

		for _i, _szJ in enumerate( self.group.get_active_joints()):
			print( '# group "{}", joint "{}" @ {}'.format( _szGroup, _szJ, _i))
			self.ppnJointIdxPerGroup[ _szGroup][ _szJ] = _i

		self.pfZeroes = [ 0.0 ] * len(
			self.ppnJointIdxPerGroup.get( _szGroup))


		self.sub4RobotTrajWholeBody = rospy.Subscriber(
			_szWholeBodyRobotTrajTopic, RobotTrajectory,
			self.handleRobotTrajMsg)


	## --------------------
	def __del__( self):
		## unregister publisher(s) & subscriber(s)

		print( '# {}::__del__(): {} connection(s) to topic "{}".'.format(
			type( self).__name__,
			self.sub4RobotTrajWholeBody.get_num_connections(),
			self.sub4RobotTrajWholeBody.resolved_name))

		if self.sub4RobotTrajWholeBody.get_num_connections() == 0:
			print( ('# {}::__del__(): no connections to topic - ' +
				'safe to unregister()?').format( type( self).__name__))
			self.sub4RobotTrajWholeBody.unregister()
			self.sub4RobotTrajWholeBody = None

		for _szG in self.ppub4RobotTrajPart.keys():
			_pub = self.ppub4RobotTrajPart.pop( _szG)
			print( '# Group "{}": {} connection(s) to topic "{}".'.format(
				_szG, _pub.get_num_connections(), _pub.resolved_name))
			_pub.unregister()
			_pub = None

		super( MyMoveGroupSplitter, self).__del__()


	## ----------------------------------------
	def genZeroes( self, _nLenTgt, _nLenOrg=0):

		if _nLenOrg <= 0:
			return []

		return self.pfZeroes[:_nLenTgt]

	## ----------------------------------------
	def copyOneJointTrajPointSubSet( self, _pnJntIdxTgt, _jtpSrc,
			_pnJntIdxSrc, _nDebug=0):

		_nSubLen = len( _pnJntIdxTgt)

		if _nDebug >= 1:
			print( ('# cOJTPSS(): len( _pnJntIdxTgt) = {}, len( _jtpSrc.'
				'positions) = {}, _nSubLen = {}').format( len( _pnJntIdxTgt),
					len( _jtpSrc.positions), _nSubLen))


		_jtpRet = JointTrajectoryPoint()

		_jtpRet.positions = self.genZeroes( _nSubLen, len( _jtpSrc.positions))
		_jtpRet.velocities = self.genZeroes( _nSubLen,
			len( _jtpSrc.velocities))
		_jtpRet.accelerations = self.genZeroes( _nSubLen,
			len( _jtpSrc.accelerations))
		_jtpRet.effort = self.genZeroes( _nSubLen, len( _jtpSrc.effort))

		for _szJ, _i in _pnJntIdxTgt.items():
			if _nDebug >= 2:
				print( '# cOJTPSS(): [{}] = "{}"'.format( _i, _szJ))

			_iSrc = _pnJntIdxSrc.get( _szJ)

			if len( _jtpRet.positions):
				_jtpRet.positions[ _i] = _jtpSrc.positions[ _iSrc]
			if len( _jtpRet.velocities):
				_jtpRet.velocities[ _i] = _jtpSrc.velocities[ _iSrc]
			if len( _jtpRet.accelerations):
				_jtpRet.accelerations[ _i] = _jtpSrc.accelerations[ _iSrc]
			if len( _jtpRet.effort):
				_jtpRet.effort[ _i] = _jtpSrc.effort[ _iSrc]

		_jtpRet.time_from_start = _jtpSrc.time_from_start

		return _jtpRet

	## ----------------------------------------
	def copyJointTrajPointSeqSubSet( self, _jtRet, _szSubGrp, _jtSrc, _szGrp,
			_nDebug=0):

		_jtRet.header.frame_id = _jtSrc.header.frame_id
		_jtRet.joint_names = self.ppszJointSeqPerGroup.get( _szSubGrp)[:]

		if _nDebug >= 1:
			print( '# cJTPSSS(): len( _jtSrc.points) = {}'.format(
				len( _jtSrc.points)))

		for _i, _jtp in enumerate( _jtSrc.points):
			if _nDebug >= 2:
				print( '# cJTPSSS(): [{}] {} -> {}'.format( _i, _szGrp,
					_szSubGrp))

			_jtRet.points.append( self.copyOneJointTrajPointSubSet(
				self.ppnJointIdxPerGroup.get( _szSubGrp), _jtp,
				self.ppnJointIdxPerGroup.get( _szGrp), _nDebug - 1)
			)

	## ----------------------------------------
	def handleRobotTrajMsg( self, _msg):
		_bVerbose = False

		self.nFakeEcho = 0

		print( ('# hRTM(): message received. joint_trajectory.points[ {}]{}{}'
			).format( len( _msg.joint_trajectory.points),
				('',' = ')[ _bVerbose],
				('', str( _msg.joint_trajectory))[ _bVerbose]))

		for _szSubGrp in self.pszSubGroup:
			_rt = RobotTrajectory()

			print( '# Generate robot trajetory for joint sub-set "{}"'.format(
				_szSubGrp))

			self.copyJointTrajPointSeqSubSet( _rt.joint_trajectory, _szSubGrp,
				_msg.joint_trajectory, self.group_name, 1)

#			print( _rt)

			self.ppub4RobotTrajPart.get( _szSubGrp).publish( _rt)

			self.pLastRobotTrajPerGroup[ _szSubGrp] = _rt


		self.nFakeEcho = self.nFakeEchoMax

	## ----------------------------------------
	def repeatMessage( self):

		if self.nFakeEcho <= 0:
			return False

		for _szSubGrp in self.pszSubGroup:
			self.ppub4RobotTrajPart.get( _szSubGrp).publish(
				self.pLastRobotTrajPerGroup.get( _szSubGrp))

		self.nFakeEcho -= 1
		return True

	## ----------------------------------------
	@classmethod
	def main( cls, argv):
		print( '# argv[ {}] = {}'.format( len( argv), str( argv)))

		mvGrpSplit = MyMoveGroupSplitter( argv, 'MobileManipulator')

		rospy.spin()
		return

		while not rospy.is_shutdown():
			mvGrpSplit.repeatMessage()
			rospy.sleep( 10.0)


## ====================
if __name__ == '__main__':
	import sys
	sys.exit( MyMoveGroupSplitter.main( sys.argv))

