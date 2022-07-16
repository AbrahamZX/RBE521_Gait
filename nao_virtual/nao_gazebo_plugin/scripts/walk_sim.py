import rospy
import numpy as np
import gaz_fk as fk
import nao_ik_v3 as ik
import trans_utils as tu
import pub_utils as pub

from math import pi
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ModelStates
from scipy.spatial.transform import Rotation as R

class nao_sim:
    def __init__(self):
        self.ll = [0, 0, 0, 0, 0, 0]
        self.rl = [0, 0, 0, 0, 0, 0]
        self.la = [0, 0, 0, 0, 0]
        self.ra = [0, 0, 0, 0, 0]
        self.goal_la = [pi/2, 0, -pi/2, 0, 0]
        self.goal_ra = [pi/2, 0, pi/2, 0, 0]
        self.gait = -1
        self.init = True
        self.skir = 0
        self.skil = 0
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
        msg = rospy.wait_for_message('/gazebo/model_states', ModelStates).pose[1]
        
        if self.la != self.goal_la:
            self.la = self.goal_la
            Ala = self.la
            JTla = pub.la_jt(Ala[0], Ala[1], Ala[2], Ala[3], Ala[4])
            self.left_arm.publish(JTla)

        if self.ra != self.goal_ra:
            self.ra = self.goal_ra
            Ara = self.ra
            JTra = pub.ra_jt(Ara[0], Ara[1], Ara[2], Ara[3], Ara[4])
            self.right_arm.publish(JTra)
        
        pos = msg.position
        #print (pos)
        o = msg.orientation
        #Tw = tu.quat2transmat(pos.x, pos.y, pos.z, o.x, o.y, o.z, o.w)
        #print (Tw)

        self.gait += 4
        
        Trl, Tll = tu.csv2transmat('gait_steps.csv', self.gait) #T_offset
        '''
        if self.gait == 0 and self.init == True:
            Trl1 = Trl
            Tll1 = Tll
            Trl = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
            Tll = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
            self.init = False
            self.gait = -1
        else:
        '''
        Trl1, Tll1 = tu.csv2transmat('gait_steps.csv', self.gait+4) #T_offset_+1

        print ('Offset Trl: ',Trl, Trl1)
        print ('Offset Tll: ',Tll, Tll1)

        Tdrl = Trl1 - Trl
        Tdll = Tll1 - Tll
        
        #Trl = Trl@Tw
        #Tll = Tll@Tw

        print ('Tdrl', Tdrl)
        print ('Tdll', Tdll)

        Tfkrl, Tfkll = fk.fk (1) #T_right_leg, T_left_leg
        print ('Tfkrl', Tfkrl)
        print ('Tfkll', Tfkll)

        
        world2rob = R.from_quat([o.x,o.y,o.z,o.w]) #T_w2r
        print ('world2rob: ', world2rob)
        rotm = world2rob.as_matrix()
        Tdrl[0:3,0:3] = Tdrl[0:3,0:3]@rotm 
        Tdll[0:3,0:3] = Tdll[0:3,0:3]@rotm

        Tfkrl[0,3] = Tfkrl[0,3] + Tdrl[0,3]
        Tfkrl[2,3] = Tfkrl[2,3] + Tdrl[2,3]
        Tfkll[0,3] = Tfkll[0,3] + Tdll[0,3]
        Tfkll[2,3] = Tfkll[2,3] + Tdll[2,3]

        print ('Mod Tfkrl', Tfkrl)
        print ('Mod Tfkll', Tfkll)

        All = ik.IK_LL(Tfkll)
        print (All)
        Arl = ik.IK_RL(Tfkrl)
        print (Arl)

        if All != [] and Arl != []:
            JTp, JTll, JTlf = pub.ll_jt(All[0], All[1], All[2], All[3], All[4], All[5])
            self.pelvis.publish(JTp)
            self.left_leg.publish(JTll)
            self.left_foot.publish(JTlf)
            
            JTrl, JTrf = pub.rl_jt(Arl[1], Arl[2], Arl[3], Arl[4], Arl[5])
            self.right_leg.publish(JTrl)    
            self.right_foot.publish(JTrf)

        else:
            if All == []:
                self.skil += 1
            if Arl == []:
                self.skir += 1
        print ('Skipped right: ', self.skir, 'Skipped left: ', self.skil)
        
def trajectory_sim ():
    # Initializing the ROS Node 
    rospy.init_node("JointPub")
    n = nao_sim()

    # Waiting for all publishers to establish connection with the controller listeners
    while n.left_arm.get_num_connections() < 1 or n.right_arm.get_num_connections() < 1 or n.left_hand.get_num_connections() < 1 or n.right_hand.get_num_connections() < 1 or n.pelvis.get_num_connections() < 1 or n.left_leg.get_num_connections() < 1 or n.right_leg.get_num_connections() < 1 or n.left_foot.get_num_connections() < 1 or n.right_foot.get_num_connections() < 1:
        pass

    # Calling the function that generates the messages and pusblishes    

    rospy.Timer(rospy.Duration(.001), n.sim_test)
    rospy.spin()

if __name__ == '__main__':
    trajectory_sim()
