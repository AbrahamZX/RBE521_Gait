import rospy
import numpy as np
import nao_fk as fk
import nao_ik_v3 as ik
import trans_utils as tu
import pub_utils as pub

from math import pi
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class nao_sim:
    def __init__(self):
        self.ll = [0, 0, 0, 0, 0, 0]
        self.rl = [0, 0, 0, 0, 0, 0]
        self.la = [0, 0, 0, 0, 0]
        self.ra = [0, 0, 0, 0, 0]
        self.goal_ll = [0, 0, -pi/8, pi/4, -pi/6, 0]
        self.goal_rl = [0, 0, -pi/8, pi/4, -pi/6, 0]
        self.goal_la = [pi/2, 0, -pi/2, 0, 0]
        self.goal_ra = [pi/2, 0, pi/2, 0, 0]
        self.left_arm = rospy.Publisher('/nao_dcm/LeftArm_controller/command', JointTrajectory, queue_size=1)
        self.right_arm = rospy.Publisher('/nao_dcm/RightArm_controller/command', JointTrajectory, queue_size=1)
        self.left_hand = rospy.Publisher('/nao_dcm/LeftHand_controller/command', JointTrajectory, queue_size=1)
        self.right_hand = rospy.Publisher('/nao_dcm/RightHand_controller/command', JointTrajectory, queue_size=1)
        self.pelvis = rospy.Publisher('/nao_dcm/Pelvis_controller/command', JointTrajectory, queue_size=1)
        self.left_leg = rospy.Publisher('/nao_dcm/LeftLeg_controller/command', JointTrajectory, queue_size=1)
        self.right_leg = rospy.Publisher('/nao_dcm/RightLeg_controller/command', JointTrajectory, queue_size=1)
        self.left_foot = rospy.Publisher('/nao_dcm/LeftFoot_controller/command', JointTrajectory, queue_size=1)
        self.right_foot = rospy.Publisher('/nao_dcm/RightFoot_controller/command', JointTrajectory, queue_size=1)

    def sim_test(self, time):
        if self.la != self.goal_la:
            self.la = self.goal_la
            #Tla = fk.FK_LA(self.la[0], self.la[1], self.la[2], self.la[3])
            #Ala = ik.IK_LA(Tla)
            Ala = self.la
            JTla = pub.la_jt(Ala[0], Ala[1], Ala[2], Ala[3], Ala[4])
            self.left_arm.publish(JTla)

        if self.ra != self.goal_ra:
            self.ra = self.goal_ra
            #Tra = fk.FK_RA(self.ra[0], self.ra[1], self.ra[2], self.ra[3])
            #Ara = ik.IK_RA(Tra)
            Ara = self.ra
            JTra = pub.ra_jt(Ara[0], Ara[1], Ara[2], Ara[3], Ara[4])
            self.right_arm.publish(JTra)
        
        if self.ll != self.goal_ll:
            for i in range(len(self.ll)):
                if self.goal_ll[i] < 0 and self.ll[i] > self.goal_ll[i]:
                    self.ll[i] -= 0.05
                if self.goal_ll[i] > 0 and self.ll[i] < self.goal_ll[i]:
                    self.ll[i] += 0.1
            print ('Left leg:',self.ll)
            Tll = fk.FK_LL(self.ll[0], self.ll[1], self.ll[2], self.ll[3], self.ll[4], self.ll[5])
            print (Tll)
            All = ik.IK_LL(Tll)
            print (All)
            JTp, JTll, JTlf = pub.ll_jt(All[0], All[1], All[2], All[3], All[4], All[5])
            self.pelvis.publish(JTp)
            self.left_leg.publish(JTll)
            self.right_foot.publish(JTlf)
 
        if self.rl != self.goal_rl:
            for i in range(len(self.rl)):
                if self.goal_rl[i] < 0 and self.rl[i] > self.goal_rl[i]:
                    self.rl[i] -= 0.05
                if self.goal_rl[i] > 0 and self.rl[i] < self.goal_rl[i]:
                    self.rl[i] += 0.1
            print ('Right leg:',self.rl)
            Trl = fk.FK_RL(self.rl[0], self.rl[1], self.rl[2], self.rl[3], self.rl[4], self.rl[5])
            print (Trl)
            Arl = ik.IK_RL(Trl)
            JTrl, JTrf = pub.rl_jt(Arl[1], Arl[2], Arl[3], Arl[4], Arl[5])
            self.right_leg.publish(JTrl)
            self.right_foot.publish(JTrf)
        
def trajectory_sim ():
    # Initializing the ROS Node 
    rospy.init_node("JointPub")
    n = nao_sim()

    # Waiting for all publishers to establish connection with the controller listeners
    while n.left_arm.get_num_connections() < 1 or n.right_arm.get_num_connections() < 1 or n.left_hand.get_num_connections() < 1 or n.right_hand.get_num_connections() < 1 or n.pelvis.get_num_connections() < 1 or n.left_leg.get_num_connections() < 1 or n.right_leg.get_num_connections() < 1 or n.left_foot.get_num_connections() < 1 or n.right_foot.get_num_connections() < 1:
        pass

    # Calling the function that generates the messages and pusblishes    

    rospy.Timer(rospy.Duration(.5), n.sim_test)
    rospy.spin()

if __name__ == '__main__':
    trajectory_sim()
