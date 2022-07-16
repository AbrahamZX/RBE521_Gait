#!/usr/bin/env python3
import numpy as np
import rospy
import nao_fk as fk

# Abandonded: it was meant to take the robot EE in the gazebo simulation and output
# the joint angles.

def IK_LL(T=fk.FK_LL()):
    ThighLength = 100
    TibiaLength = 102.9
    FootHeight = 45.19
    HipOffsetZ = 85
    HipOffsetY = 50
    Ab = fk.A([0, HipOffsetY, -HipOffsetZ]) 
    Ae = fk.A([0, 0, -FootHeight]) 
    T_hat = np.linalg.pinv(Ab)@T@np.linalg.pinv(Ae)
    T_tilde = fk.R_x(np.pi/4)@T_hat
    T_prime = np.linalg.pinv(T_tilde)
    #d = np.sqrt(T_prime[0,3]**2+T_prime[1,3]**2+T_prime[2,3]**2)
    #the4prime = np.arccos((ThighLength**2 + TibiaLength**2 - d**2)/(2*TibiaLength*ThighLength))
    
    d = T_prime[0,3]**2+T_prime[1,3]**2+T_prime[2,3]**2
    the4prime = np.arccos(np.round((ThighLength**2 + TibiaLength**2 - d)/(2*TibiaLength*ThighLength),4)) #rounding here prevents numerical error from pushing us to slighly out of arccos domain
    
    the4 = np.pi-the4prime

    the6 = np.arctan2(T_prime[1,3],T_prime[2,3]) 

    T56 = T_n_s(0,-np.pi/2,0,the6)
    T_prime_tilde = T_tilde@np.linalg.pinv(T56@fk.R_z(np.pi)@fk.R_y(-np.pi/2))
    T_dprime = np.linalg.pinv(T_prime_tilde)

    top = T_dprime[1,3]*TibiaLength+ThighLength*np.cos(the4) + ThighLength*T_dprime[0,3]*np.sin(the4)
    bot = ThighLength**2*np.sin(the4)**2 + (TibiaLength+ThighLength*np.cos(the4))**2

    the5 = np.arcsin(-top/bot) #page 60....l1 = thigh, l2 = tibia

    T34 = T_n_s(-ThighLength,0,0,the4)
    T45 = T_n_s(-TibiaLength,0,0,the5)

    T_tprime = T_prime_tilde@np.linalg.pinv(T34@T45)

    the2 = np.arccos(T_tprime[1,3])-np.pi/4

    the3 = np.arcsin(T_tprime[2,2]/np.sin(the2+np.pi/4))

    the1 = np.pi/2 + np.arccos(T_tprime[1,3]/np.sin(the2+np.pi/4))

    return [the1,the2,the3,the4,the5,the6] #Maybe use a different format? 


def IK_RL(T=fk.FK_RL()):

    #From http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html --> double check that these are correct
    ThighLength = 100
    TibiaLength = 102.9
    FootHeight = 45.19
    HipOffsetZ = 85
    HipOffsetY = 50
    Ab = fk.A([0, -HipOffsetY, -HipOffsetZ]) 
    Ae = fk.A([0, 0, -FootHeight]) 
    T_hat = np.linalg.pinv(Ab)@T@np.linalg.pinv(Ae)
    T_tilde = fk.R_x(-np.pi/4)@T_hat


    T_prime = np.linalg.pinv(T_tilde)
    #d = np.sqrt(T_prime[0,3]**2+T_prime[1,3]**2+T_prime[2,3]**2)
    #the4prime = np.arccos((ThighLength**2 + TibiaLength**2 - d**2)/(2*TibiaLength*ThighLength))

    d = T_prime[0,3]**2+T_prime[1,3]**2+T_prime[2,3]**2

    the4prime = np.arccos(np.round((ThighLength**2 + TibiaLength**2 - d)/(2*TibiaLength*ThighLength),4)) #rounding here prevents numerical error from pushing us to slighly out of arccos domain
    the4 = np.pi-the4prime

    the6 = np.arctan2(T_prime[1,3],T_prime[2,3]) 

    T56 = T_n_s(0,-np.pi/2,0,the6)
    T_prime_tilde = T_tilde@np.linalg.pinv(T56@fk.R_z(np.pi)@fk.R_y(-np.pi/2))
    
    T_dprime = np.linalg.pinv(T_prime_tilde)
   

    top = T_dprime[1,3]*TibiaLength+ThighLength*np.cos(the4) + ThighLength*T_dprime[0,3]*np.sin(the4)
    bot = ThighLength**2*np.sin(the4)**2 + (TibiaLength+ThighLength*np.cos(the4))**2

    the5 = np.arcsin(-top/bot) #page 60....l1 = thigh, l2 = tibia

    T34 = T_n_s(-ThighLength,0,0,the4)
    T45 = T_n_s(-TibiaLength,0,0,the5)

    T_tprime = T_prime_tilde@np.linalg.pinv(T34@T45)

    the2 = np.arccos(T_tprime[2,3])-np.pi/4

    the3 = np.arcsin(T_tprime[2,2]/np.sin(the2+np.pi/4))

    the1 = np.pi/2 + np.arccos(T_tprime[1,3]/np.sin(the2+np.pi/4))

    return [the1,the2,the3,the4,the5,the6] #Maybe use a different format? 


def IK_RA(T=fk.FK_RA()):

    Ab = fk.A([0, -98.00, 100.00])
    Ae = fk.A([57.75+55.95, 0, 0])

    UpperArmLength = 105
    ElbowOffsetY = 15

    l1 = -ElbowOffsetY
    l2 = UpperArmLength

    T_prime = np.linalg.pinv(Ab)@T@np.linalg.pinv(Ae)
    T_dprime = np.linalg.pinv(T_prime)@np.linalg.pinv(fk.R_z(np.pi/2))

    the3 = np.arcsin(T_dprime[2,3]/l1)
    the4 = np.arccos((l2*T_dprime[1,3]-l1*T_dprime[0,3]*np.cos(the3))/(l2**2 + l1**2*np.cos(the3)**2))
    T23 = T_n_s(-ElbowOffsetY,np.pi/2,UpperArmLength,the3)
    T34 = T_n_s(0,-np.pi/2,0,the4)
    T_tprime = T_prime@np.linalg.pinv(T23)@np.linalg.pinv(T34)
    the2 = np.arctan2(T_tprime[1,0],T_tprime[1,1])-np.pi/2
    the1 = np.arctan2(T_tprime[0,2],T_tprime[2,2])
    return [the1, the2, the3, the4]


def IK_LA(T=fk.FK_LA()):

    Ab = fk.A([0, -98.00, 100.00])
    Ae = fk.A([57.75+55.95, 0, 0])

    UpperArmLength = 105
    ElbowOffsetY = 15

    l1 = ElbowOffsetY
    l2 = UpperArmLength

    T_prime = np.linalg.pinv(Ab)@T@np.linalg.pinv(Ae)
    T_dprime = np.linalg.pinv(T_prime)@np.linalg.pinv(fk.R_z(np.pi/2))

    the3 = np.arcsin(T_dprime[2,3]/l1)
    the4 = np.arccos((l2*T_dprime[1,3]-l1*T_dprime[0,3]*np.cos(the3))/(l2**2 + l1**2*np.cos(the3)**2))
    T23 = T_n_s(ElbowOffsetY,np.pi/2,UpperArmLength,the3)
    T34 = T_n_s(0,-np.pi/2,0,the4)
    T_tprime = T_prime@np.linalg.pinv(T23)@np.linalg.pinv(T34)
    the2 = np.arctan2(T_tprime[1,0],T_tprime[1,1])-np.pi/2
    the1 = np.arctan2(T_tprime[0,2],T_tprime[2,2])
    return [the1, the2, the3, the4]


def T_n_s (a, alp, d, the):
    # CANNOT SUBSTITUTE fk.T_n --> creates 3D arrays and relies on list structure
    Tn = np.array([[np.cos(the), -np.sin(the), 0, a],
                        [np.sin(the)*np.cos(alp), np.cos(the)*np.cos(alp), -np.sin(alp), -d*np.sin(alp)],
                        [np.sin(the)*np.sin(alp), np.cos(the)*np.sin(alp), np.cos(alp), d*np.cos(alp)],
                        [0, 0, 0, 1]])

    return Tn


def ik (time):
    print (1)
    '''
    #LAT = [[ 3.46783781e-02, -9.99398524e-01,  1.95160683e-07,  3.92084978e+00],
    #      [ 3.10778460e-04,  1.09790573e-05,  9.99999952e-01,  1.13067987e+02],
    #      [-9.99398476e-01, -3.46783764e-02,  3.10972270e-04, -1.18626935e+02],
    #      [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
    LAT = [[ 0, -1,  0,  0],
          [0,  0,  1,  113.0],
          [-1, 0, 0, -118.7],
          [0,  0,  0,  1]]

    LA = IK_LA(LAT)
    print ("Left arm:\n", LA,"\n")
    
    ra = find_joint_val(joint_msg, ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"])
    RA = FK_RA(ra[0], ra[1], ra[2], ra[3])
    print ("Right arm:\n", RA,"\n")
    '''

    #ll = find_joint_val(joint_msg, ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitchJoint", "LAnkleRoll"])
    LLT =  [[ 9.91492293e-01,  2.87930223e-04, -1.30165091e-01,  4.71580516e+00],
            [-2.75089191e-04,  9.99999955e-01,  1.16631872e-04,  4.99708743e+01],
            [ 1.30165119e-01, -7.98325924e-05,  9.91492327e-01, -3.17259889e+02],
            [0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
    LL = IK_LL(LLT)
    print ("Left leg:\n", LL,"\n")
    
    RLT =  [[9.91480769e-01, -2.05148045e-04, -1.30252994e-01, 4.73454748e+00],
            [2.38906243e-04, 9.99999942e-01, 2.43548435e-04, -5.00486859e+01],
            [1.30252936e-01, -2.72591843e-04, 9.91480760e-01, -3.17259507e+02],
            [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
    RL = IK_RL(RLT)
    print ("Right leg:\n", RL,"\n")
    
    '''
    rl = find_joint_val(joint_msg, ["RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitchJoint", "RAnkleRoll"])
    RL = FK_RL(rl[0], rl[1], rl[2], rl[3], rl[4], rl[5])
    print ("Right leg:\n", RL,"\n")
    '''


if __name__ == '__main__':
    rospy.init_node("nao_ik")
    rospy.Timer(rospy.Duration(1), ik)
    rospy.spin()
'''
if __name__ == '__main__':
    IKLL = IK_LL ()
    print ("Left leg:\n",IKLL,"\n")
    IKRL = IK_RL ()
    print ("Right leg:\n",IKRL,"\n")
    IKLA = IK_LA ()
    print("Left Arm: \n",IKLA,"\n")
    IKRA = IK_RA ()
    print("Right Arm: \n",IKRA,"\n")
'''
