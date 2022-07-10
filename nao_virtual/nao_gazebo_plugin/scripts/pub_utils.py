import rospy

from math import pi
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def ra_jt (RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw):
    # Right Arm
    ra = JointTrajectory()
    ra.joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
    rapt = JointTrajectoryPoint()
    rapt.positions = [RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw]
    rapt.time_from_start = rospy.Duration.from_sec(1)
    ra.points.append(rapt)
    
    return ra

def la_jt (LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw):
    la = JointTrajectory()
    la.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
    lapt = JointTrajectoryPoint()
    lapt.positions = [LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw]
    lapt.time_from_start = rospy.Duration.from_sec(1)
    la.points.append(lapt)

    return la

def ll_jt (LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitchJoint, LAnkleRoll):
    # Pelvis
    p = JointTrajectory()
    p.joint_names = ['LHipYawPitch']
    ppt = JointTrajectoryPoint()
    ppt.positions = [LHipYawPitch]
    ppt.time_from_start = rospy.Duration.from_sec(1)
    p.points.append(ppt)

    # Left Leg
    ll = JointTrajectory()
    ll.joint_names = ['LHipRoll', 'LHipPitch', 'LKneePitch']
    llpt = JointTrajectoryPoint()
    llpt.positions = [LHipRoll, LHipPitch, LKneePitch]
    llpt.time_from_start = rospy.Duration.from_sec(1)
    ll.points.append(llpt)

    # Left Foot
    lf = JointTrajectory()
    lf.joint_names = ['LAnklePitchJoint', 'LAnkleRoll']
    lfpt = JointTrajectoryPoint()
    lfpt.positions = [LAnklePitchJoint, LAnkleRoll]
    print ('Left foot positions:',lfpt.positions) 
    lfpt.time_from_start = rospy.Duration.from_sec(1)
    lf.points.append(lfpt)
    print ('lf:', lf)
    return p, ll, lf

def rl_jt (RHipRoll, RHipPitch, RKneePitch, RAnklePitchJoint, RAnkleRoll):
    # Right Leg
    rl = JointTrajectory()
    rl.joint_names = ['RHipRoll', 'RHipPitch', 'RKneePitch']
    rlpt = JointTrajectoryPoint()
    rlpt.positions = [RHipRoll, RHipPitch, RKneePitch]
    rlpt.time_from_start = rospy.Duration.from_sec(1)
    rl.points.append(rlpt)

    # Right Foot
    rf = JointTrajectory()
    rf.joint_names = ['RAnklePitchJoint', 'RAnkleRoll']
    rfpt = JointTrajectoryPoint()
    rfpt.positions = [RAnklePitchJoint, RAnkleRoll]
    print ('Right foot positions:',rfpt.positions)
    rfpt.time_from_start = rospy.Duration.from_sec(1)
    
    rf.points.append(rfpt)
    print ('rf:', rf)
    return rl, rf
