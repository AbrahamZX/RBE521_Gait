import numpy as np
import nao_fk as fk

def IK_LL(T=fk.FK_LL()):

    #From http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html --> double check that these are correct
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

def T_n_s (a, alp, d, the):
    # CANNOT SUBSTITUTE fk.T_n --> creates 3D arrays and relies on list structure
    Tn = np.array([[np.cos(the), -np.sin(the), 0, a],
                        [np.sin(the)*np.cos(alp), np.cos(the)*np.cos(alp), -np.sin(alp), -d*np.sin(alp)],
                        [np.sin(the)*np.cos(alp), np.cos(the)*np.sin(alp), np.cos(alp), d*np.cos(alp)],
                        [0, 0, 0, 1]])

    return Tn

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

if __name__ == '__main__':
    IKLL = IK_LL ()
    print ("Left leg:\n",IKLL,"\n")
    IKRL = IK_RL ()
    print ("Right leg:\n",IKRL,"\n")
    IKLA = IK_LA ()
    print("Left Arm: \n",IKLA,"\n")
    IKRA = IK_RA ()
    print("Right Arm: \n",IKRA,"\n")
