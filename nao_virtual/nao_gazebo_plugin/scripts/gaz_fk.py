#!/usr/bin/env python3
import rospy
import numpy as np

from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

# Forward kinematics of the nao robot using the joint angles being inputted
# by the gazebo simulation to find the end effector pose.

def R_x (ang):
    return np.array([[1, 0, 0, 0],
                     [0, np.cos(ang), -np.sin(ang), 0],
                     [0, np.sin(ang), np.cos(ang), 0],
                     [0, 0, 0, 1]])
    
def R_y (ang):
    return np.array([[np.cos(ang), 0, np.sin(ang), 0],
                     [0, 1, 0, 0],
                     [-np.sin(ang), 0, np.cos(ang), 0],
                     [0, 0, 0, 1]])

def R_z (ang):
    return np.array([[np.cos(ang), -np.sin(ang), 0, 0],
                     [np.sin(ang), np.cos(ang), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def A (p):
    return np.array([[1, 0, 0, p[0]],
                     [0, 1, 0, p[1]],
                     [0, 0, 1, p[2]],
                     [0, 0, 0, 1]])

def T_n (a, alp, d, the, Tn = []):
    for n in range(len(a)):    
        Tn.append(np.array([[np.cos(the[n]), -np.sin(the[n]), 0, a[n]],
                             [np.sin(the[n])*np.cos(alp[n]), np.cos(the[n])*np.cos(alp[n]), -np.sin(alp[n]), -d[n]*np.sin(alp[n])],
                             [np.sin(the[n])*np.sin(alp[n]), np.cos(the[n])*np.sin(alp[n]), np.cos(alp[n]), d[n]*np.cos(alp[n])],
                             [0, 0, 0, 1]]))

    return Tn

def T_0n (Tn):
    T0n = Tn[0]
    for n in range (1, len(Tn)):
        T0n = np.dot(T0n,Tn[n])

    return T0n

def FK_LA (LShoulderPitch = 0, LShoulderRoll = 0, LElbowYaw = 0, LElbowRoll = 0):
    a = [0, 0, 15.00, 0]
    alp = [-np.pi/2, np.pi/2, np.pi/2, -np.pi/2]
    d = [0, 0, 105.00, 0]
    the = [LShoulderPitch, LShoulderRoll+np.pi/2, LElbowYaw, LElbowRoll]

    Ab = A([0, 98.00, 100.00])
    Ae = A([57.75+55.95, 0, 0])
    R = R_z(-np.pi/2)

    Tn = T_n(a, alp, d, the, [Ab])
    Tn.append(R)
    Tn.append(Ae)

    T0n = T_0n(Tn)
    
    return T0n

def FK_RA (RShoulderPitch = 0, RShoulderRoll = 0, RElbowYaw = 0, RElbowRoll = 0):
    a = [0, 0, -15.00, 0]
    alp = [-np.pi/2, np.pi/2, np.pi/2, -np.pi/2]
    d = [0, 0, 105.00, 0]
    the = [RShoulderPitch, RShoulderRoll+np.pi/2, RElbowYaw, RElbowRoll]

    Ab = A([0, -98.00, 100.00])
    Ae = A([57.55+55.95, 0, 0])
    R = R_z(-np.pi/2)
    
    Tn = T_n(a, alp, d, the, [Ab])
    Tn.append(R)
    Tn.append(Ae)

    T0n = T_0n(Tn)
    
    return T0n

def FK_LL (LHipYawPitch = 0, LHipRoll = 0, LHipPitch = 0, LKneePitch = 0, LAnklePitch = 0, LAnkleRoll = 0):
    a = [0, 0, 0, -100.00, -102.90, 0]
    alp = [-3*np.pi/4, -np.pi/2, np.pi/2, 0, 0, -np.pi/2]
    d = [0, 0, 0, 0, 0, 0]
    the = [LHipYawPitch-np.pi/2, LHipRoll+np.pi/4, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll]

    Ab = A([0, 50.00, -85.00])
    Ae = A([0, 0, -45.19])
    Rz = R_z(np.pi)
    Ry = R_y(-np.pi/2)

    Tn = T_n(a, alp, d, the, [Ab])
    Tn.append(Rz)
    Tn.append(Ry)
    Tn.append(Ae)

    T0n = T_0n(Tn)

    return T0n

def FK_RL (RHipYawPitch = 0, RHipRoll = 0, RHipPitch = 0, RKneePitch = 0, RAnklePitch = 0, RAnkleRoll = 0):
    a = [0, 0, 0, -100.00, -102.90, 0]
    alp =[-np.pi/4, -np.pi/2, np.pi/2, 0, 0, -np.pi/2]
    d = [0, 0, 0, 0, 0, 0]
    the = [RHipYawPitch-np.pi/2, RHipRoll-np.pi/4, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll]

    Ab = A([0, -50.00, -85.00])
    Ae = A([0, 0, -45.19])
    Rz = R_z(np.pi)
    Ry = R_y(-np.pi/2)

    Tn = T_n(a, alp, d, the, [Ab])
    Tn.append(Rz)
    Tn.append(Ry)
    Tn.append(Ae)

    T0n = T_0n(Tn)

    return T0n

def find_joint_val (msg, joints = []):
    joint_values = []
    for jn in joints:
        for jm in range(26):
            if msg.name[jm] == jn:
                joint_values.append(msg.position[jm])
    return joint_values

def fk (time):
    joint_msg = rospy.wait_for_message('/joint_states', JointState)

    la = find_joint_val(joint_msg, ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"])
    LA = FK_LA(la[0], la[1], la[2], la[3])

    ra = find_joint_val(joint_msg, ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"])
    RA = FK_RA(ra[0], ra[1], ra[2], ra[3])
    
    ll = find_joint_val(joint_msg, ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitchJoint", "LAnkleRoll"])
    LL = FK_LL(ll[0], ll[1], ll[2], ll[3], ll[4], ll[5])

    rl = find_joint_val(joint_msg, ["RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitchJoint", "RAnkleRoll"])
    RL = FK_RL(rl[0], rl[1], rl[2], rl[3], rl[4], rl[5])

    return RL, LL

if __name__ == '__main__':
    rospy.init_node("nao_fk")
    rospy.Timer(rospy.Duration(.01), fk)
    rospy.spin()
