#!/usr/bin/env python

from math import pi, fabs, sqrt, cos

import rospy
# import moveit_msgs.msg
# from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# from mv_grp_goal_seq_run import MyMoveGroupGoalTrajRunner
from mv_grp_base import MyMoveGroup

## ============================================================
# class MyMoveGroupWTF( MyMoveGroupGoalTrajRunner):
class MyMoveGroupWTF( MyMoveGroup):

	## ----------------------------------------
	def __init__( self, argv, _szGroup='MobileManipulator',
			_szNodeName='my_test_wtf', _bAnon=False,
			_szGoalTrajTopic='wtf/whole_body', _fTolerance=0.01,
			_szActorWayPtTopic='gazebo/actor/way_pt',
			_fToleranceOrient=None):

		super( MyMoveGroupWTF, self).__init__( argv, _szGroup,
			_szNodeName)

		self.bPause = True

		self.sub4ActorWayPt = rospy.Subscriber( _szActorWayPtTopic, Header,
			self.handleActorWayPtMsg)


	## --------------------
	def __del__( self):
		print( '# {}::__del__(): {} connection(s) to topic "{}".'.format(
			type( self).__name__,
			self.sub4ActorWayPt.get_num_connections(),
			self.sub4ActorWayPt.resolved_name))

		if self.sub4ActorWayPt.get_num_connections() == 0:
			print( ('# {}::__del__(): no connections to topic - ' +
				'safe to unregister()?').format( type( self).__name__))
			self.sub4ActorWayPt.unregister()
			self.sub4ActorWayPt = None

		super( MyMoveGroupWTF, self).__del__()


	## ----------------------------------------
	def handleActorWayPtMsg( self, _msg):
		_bPause = True

		if _msg.frame_id == 'human_94763':
			if _msg.seq == 15:
				_bPause = False
			else:
				pass
		else:
			pass

		self.bPause = _bPause


	## ----------------------------------------
	@classmethod
	def main( cls, argv):
		print( '# argv[ {}] = {}'.format( len( argv), str( argv)))

		_fMaxTrajTime = 10

		if len( argv) > 1:
			_fMaxTrajTime = float( argv[ 1])

		mvGrpWtf = MyMoveGroupWTF( argv, 'MobileManipulator')

		_pppPoseSeqCust = [
			[ 'ZeroPose', {
				'robot_tx': 17.2, 'robot_ty': 1.6,  'robot_rz': pi
			} ],
			[ 'ZeroPose', {
				'robot_tx': 15.3, 'robot_ty': 2.65, 'robot_rz': pi/2,
				'ur_arm_elbow_joint': pi/2,
				'ur_arm_shoulder_lift_joint': -pi/4,
				'ur_arm_wrist_1_joint': -pi/4,
				'ur_arm_wrist_2_joint': pi/2,
			} ],
			[ 'ZeroPose', {
				'robot_tx': 15.7, 'robot_ty': 3.25, 'robot_rz': pi/2,
				'ur_arm_elbow_joint': pi/2,
				'ur_arm_shoulder_lift_joint': -pi/4,
				'ur_arm_wrist_1_joint': -pi/4,
				'ur_arm_wrist_2_joint': pi/2,
			} ],
			[ 'Upright', {
#				'robot_tx': 16.2, 'robot_ty': 5.80, 'robot_rz': pi/2,
				'robot_tx': 14.4, 'robot_ty': 5.80, 'robot_rz': pi/2,
				'ur_arm_shoulder_lift_joint': -pi/2,
				'ur_arm_wrist_1_joint': pi/2,
#				'ur_arm_wrist_2_joint': pi,
				'ur_arm_wrist_2_joint': pi/2,
				'ur_arm_wrist_3_joint': 0,
			} ],
			[ 'ZeroPose', {
				'robot_tx': 16.6, 'robot_ty': 7.90, 'robot_rz': 0,
				'ur_arm_wrist_2_joint': pi/2,
			} ],
		]

		_szPoseLast = None
		_jsLast = JointState()

		while mvGrpWtf.bPause:
			rospy.sleep( 0.1)

		for _i, _ppPoseNextCust in enumerate( _pppPoseSeqCust):
			_szPoseNext = _ppPoseNextCust[0]
			_pfJointOverride = _ppPoseNextCust[1]

			print( '# way-point[ {}] "{}"'.format( _i, _szPoseNext))

			mvGrpWtf.getNamedTargetJointState( _jsLast, _szPoseNext)

			mvGrpWtf.updateJointArrayFromMap( _jsLast.position, _jsLast.name,
				_pfJointOverride)

			_path = mvGrpWtf.callPlan( _jsLast, True)

			mvGrpWtf.group.execute( _path, wait=True)
			mvGrpWtf.group.stop()

			_szPoseLast = _szPoseNext

			rospy.sleep( 0.2)
			rospy.sleep( 0.2)


## ====================
if __name__ == '__main__':
	import sys
	sys.exit( MyMoveGroupWTF.main( sys.argv))

