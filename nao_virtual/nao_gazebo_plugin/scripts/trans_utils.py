import numpy as np
import csv
from scipy.spatial.transform import Rotation as R

def sixvec2transmat(P,eul='xyz',deg=False):
    rot = R.from_euler(eul,P[3:],degrees=deg)
    T = rot.as_matrix()
    T = np.vstack((T,[0,0,0]))
    vec = np.array([[P[0]], [P[1]], [P[2]], [1]])
    T = np.hstack((T,vec))
    return T

def transmat2sixvec(T,eul='xyz',deg=False):
    P = [0]*5
    rot = R.from_matrix(T[0:3,0:3])
    P[0:3] = T[0:3,3]
    P[3:5] = rot.as_euler(eul,deg)
    return P

def quat2transmat(x,y,z,i,j,k,l):
    rot = R.from_quat([i,j,k,l])
    T = rot.as_matrix()
    T = np.vstack((T,[0,0,0]))
    vec = np.array([[x], [y], [z], [1]])
    T = np.hstack((T,vec))
    return T

def csv2transmat(file,n,f):
    T_r = np.zeros((4,4))
    T_l = np.zeros((4,4))
    coords=[]
    with open(file) as csvfile:  #uses csv package to take in data
        data = csv.reader(csvfile,delimiter=',')
        for row in data: 
            coords.append(row[n])
    normalizing_factor = (float(coords[1])+float(coords[3]))/2
    xr = float(coords[1]) - normalizing_factor
    xl = float(coords[3]) - normalizing_factor
    T_r[0,3] = xr*1000*f
    if T_r[0,3] < 0:
        T_r[0,3] = 0

    T_r[1,3] = 0
    T_r[2,3] = float(coords[2])*1000*f - (85+100+102.9+45.19)
    T_r[3,3] = 1

    T_r[0,0] = 1
    T_r[1,1] = 1
    T_r[2,2] = 1

    T_l[0,3] = xl*1000*f
    if T_l[0,3] < 0:
        T_l[0,3] = 0
    T_l[1,3] = 0
    T_l[2,3] = float(coords[4])*1000*f - (85+100+102.9+45.19)
    T_l[3,3] = 1

    T_l[0,0] = 1
    T_l[1,1] = 1
    T_l[2,2] = 1

    return T_r, T_l




if __name__=='__main__':
    a = [1,2,3,0.1,0.2,0.3]
    print(a)
    b = sixvec2transmat(a)
    print(b)
    c = transmat2sixvec(b)
    print(c)
    T_r, T_l = csv2transmat('gait_steps.csv',5)
    print(T_r)
    print(T_l)

