import numpy as np

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
                             [np.sin(the[n])*np.cos(alp[n]), np.cos(the[n])*np.sin(alp[n]), np.cos(alp[n]), d[n]*np.cos(alp[n])],
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

if __name__ == '__main__':
    FKLA = FK_LA ()
    print ("Left arm:\n",FKLA,"\n")
    FKRA = FK_RA ()
    print ("Right arm:\n",FKRA,"\n")
    FKLL = FK_LL ()
    print ("Left leg:\n",FKLL,"\n")
    FKRL = FK_RL ()
    print ("Right leg:\n",FKRL,"\n")
