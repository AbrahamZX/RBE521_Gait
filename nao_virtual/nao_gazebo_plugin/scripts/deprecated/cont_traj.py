#!/usr/bin/env python3
import rospy
from math import pi
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def pub_test (time):
	# Generating the trajectory messages for the controllers
    global m, n, r
    if m < pi/2:
        m += 0.05
        n += 0.05
    #print (m, n)

    if m > pi/2:
        r += 0.05

    # Left Arm
    la = JointTrajectory()
    la.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
    lapt = JointTrajectoryPoint()
    lapt.positions = [m, r, -n, 0, 0]
    lapt.velocities = [.8, .8, .8, .8, .8]
    lapt.time_from_start = rospy.Duration.from_sec(1)
    la.points.append(lapt)
    
    # Right Arm
    ra = JointTrajectory()
    ra.joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
    rapt = JointTrajectoryPoint()
    rapt.positions = [m, -r, n, 0, 0]
    rapt.time_from_start = rospy.Duration.from_sec(1)
    ra.points.append(rapt)

    # Left Hand
    lh = JointTrajectory()
    lh.joint_names = ['LHand']
    lhpt = JointTrajectoryPoint()
    lhpt.positions = [0]
    lhpt.time_from_start = rospy.Duration.from_sec(1)
    lh.points.append(lhpt)       
    
    # Right Hand
    rh = JointTrajectory()
    rh.joint_names = ['RHand']
    rhpt = JointTrajectoryPoint()
    rhpt.positions = [0]
    rhpt.time_from_start = rospy.Duration.from_sec(1)
    rh.points.append(rhpt)    
    
    # Pelvis
    p = JointTrajectory()
    p.joint_names = ['LHipYawPitch']
    ppt = JointTrajectoryPoint()
    ppt.positions = [0]
    ppt.time_from_start = rospy.Duration.from_sec(1)
    p.points.append(ppt)    
    
    # Left Leg
    ll = JointTrajectory()
    ll.joint_names = ['LHipRoll', 'LHipPitch', 'LKneePitch']
    llpt = JointTrajectoryPoint()
    llpt.positions = [0, 0, 0]
    llpt.time_from_start = rospy.Duration.from_sec(1)
    ll.points.append(llpt)    
    
    # Right Leg
    rl = JointTrajectory()
    rl.joint_names = ['RHipRoll', 'RHipPitch', 'RKneePitch']
    rlpt = JointTrajectoryPoint()
    rlpt.positions = [0, 0, 0]
    rlpt.time_from_start = rospy.Duration.from_sec(1)
    rl.points.append(rlpt)    
    
    # Left Foot
    lf = JointTrajectory()
    lf.joint_names = ['LAnklePitchJoint', 'LAnkleRoll']
    lfpt = JointTrajectoryPoint()
    lfpt.positions = [0, 0]
    lfpt.time_from_start = rospy.Duration.from_sec(1)
    lf.points.append(lfpt)    
    
    # Right Foot
    rf = JointTrajectory()
    rf.joint_names = ['RAnklePitchJoint', 'RAnkleRoll']
    rfpt = JointTrajectoryPoint()
    rfpt.positions = [0, 0]
    rfpt.time_from_start = rospy.Duration.from_sec(1)
    rf.points.append(rfpt)
        
    # Publishing all the joint position messages
    left_arm.publish(la)
    right_arm.publish(ra)
    left_hand.publish(lh)
    right_hand.publish(rh)
    pelvis.publish(p)
    left_leg.publish(ll)
    right_leg.publish(rl)
    left_foot.publish(lf)
    right_foot.publish(rf)
    
    print ('Published joint trajectory test messages.')


def trajectory_test ():
    # Defining the joint trajectory publishers as global variables to use them inside of functions
    global left_arm, right_arm, left_hand, right_hand, pelvis, left_leg, right_leg, left_foot, right_foot, m, n, r
    m = 0
    n = 0
    r = 0.05
    # Initializing the ROS Node 
    rospy.init_node("JointPub")
    
    # Defining the publishers for each of the controllers
    left_arm = rospy.Publisher('/nao_dcm/LeftArm_controller/command', JointTrajectory, queue_size=1)
    right_arm = rospy.Publisher('/nao_dcm/RightArm_controller/command', JointTrajectory, queue_size=1)
    left_hand = rospy.Publisher('/nao_dcm/LeftHand_controller/command', JointTrajectory, queue_size=1)
    right_hand = rospy.Publisher('/nao_dcm/RightHand_controller/command', JointTrajectory, queue_size=1)    
    pelvis = rospy.Publisher('/nao_dcm/Pelvis_controller/command', JointTrajectory, queue_size=1)    
    left_leg = rospy.Publisher('/nao_dcm/LeftLeg_controller/command', JointTrajectory, queue_size=1)
    right_leg = rospy.Publisher('/nao_dcm/RightLeg_controller/command', JointTrajectory, queue_size=1)    
    left_foot = rospy.Publisher('/nao_dcm/LeftFoot_controller/command', JointTrajectory, queue_size=1)    
    right_foot = rospy.Publisher('/nao_dcm/RightFoot_controller/command', JointTrajectory, queue_size=1)    
    
    # Waiting for all publishers to establish connection with the controller listeners
    while left_arm.get_num_connections() < 1 or right_arm.get_num_connections() < 1 or left_hand.get_num_connections() < 1 or right_hand.get_num_connections() < 1 or pelvis.get_num_connections() < 1 or left_leg.get_num_connections() < 1 or right_leg.get_num_connections() < 1 or left_foot.get_num_connections() < 1 or right_foot.get_num_connections() < 1:
        pass
    
    # Calling the function that generates the messages and pusblishes    
    #pub_test()
    
    rospy.Timer(rospy.Duration(.01), pub_test)
    rospy.spin()

if __name__ == '__main__':
    trajectory_test()
