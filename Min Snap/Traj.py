import numpy as np


#reference: https://timodenk.com/blog/cubic-spline-interpolation/
def matrix_generation(ts):
    # 0,1,2,3,4,5,6,7 order derivatives
    b = np.array([[1, ts,  ts**2, ts**3,    ts**4,    ts**5,     ts**6,     ts**7],
                  [0, 1, 2*ts,  3*ts**2,  4*ts**3,  5*ts**4,   6*ts**5,   7*ts**6],
                  [0, 0, 2,     6*ts,    12*ts**2, 20*ts**3,  30*ts**4,  42*ts**5],
                  [0, 0, 0,     6,       24*ts,    60*ts**2, 120*ts**3, 210*ts**4],
                  [0, 0, 0,     0,       24,      120*ts,    360*ts**2, 840*ts**3],
                  [0, 0, 0,     0,       0,       120,       720*ts,   2520*ts**2],
                  [0, 0, 0,     0,       0,       0,         720,      5040*ts],
                  [0, 0, 0,     0,       0,       0,         0,        5040]])

    return b

n_segments = 2
m = np.zeros((8 * n_segments, 8 * n_segments))

n = 3
order = 4
pos_traj = np.zeros((n, 3))
pos_traj[0, :] = np.linspace(0.0, 10.0, n) #x
pos_traj[1, :] = np.linspace(0.0, 6.0, n) #y
pos_traj[2, :] = np.linspace(0.0, 3.0, n) #z
yaw_traj = np.zeros((1,3)) #yaw
y_ref = np.concatenate((pos_traj,yaw_traj))

flat_outputs = y_ref.shape[0]
segments = y_ref.shape[1] - 1

# Number of coefficients based on polynomial order
No_of_Coeff = order*2

#print (y_ref)
#print (y_ref[0, :])
#print (flat_outputs,segments)
poly_coefficients = np.zeros((segments, No_of_Coeff, flat_outputs)) # 1,8,3

print (poly_coefficients.shape)

def rhs_generation(x):
    # x is a tuple
    n = x.shape[0] - 1 # 2

    big_x = np.zeros((8 * n))
    big_x[:4] = np.array([x[0], 0, 0, 0]).T
    big_x[-4:] = np.array([x[-1], 0, 0, 0]).T

    for i in range(1, n):
        big_x[8 * (i - 1) + 4:8 * (i - 1) + 8 + 4] = np.array([x[i], 0, 0, 0, 0, 0, 0, x[i]]).T

    return big_x

rhs = rhs_generation(y_ref[0, :])

print (rhs)




""" # intermediary condition of the first curve (4:11,0:8)
b = matrix_generation(1.0)
#m[8 * i + 4:8 * i + 7 + 4, 8 * i:8 * i + 8] = b[:-1, :]
middle = b[:-1, :]
#print (middle)

# starting condition of the second curve position and derivatives
b = matrix_generation(-1.0)
#m[8 * i + 4 + 1:8 * i + 4 + 7, 8 * (i + 1):8 * (i + 1) + 8] = -b[1:-1, :]
#m[8 * i + 4 + 7:8 * i + 4 + 8, 8 * (i + 1):8 * (i + 1) + 8] = b[0, :]
end_a = -b[1:-1, :]
#print (end_a)
end_b = b[0,:]
#print (end_b)

#print (b[:4, :] ) """