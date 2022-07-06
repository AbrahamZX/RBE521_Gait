import numpy as np
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

if __name__=='__main__':
    a = [1,2,3,0.1,0.2,0.3]
    print(a)
    b = sixvec2transmat(a)
    print(b)
    c = transmat2sixvec(b)
    print(c)