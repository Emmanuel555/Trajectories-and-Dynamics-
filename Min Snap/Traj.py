import numpy as np
import matplotlib.pyplot as plt

# segment parameters
specified_time_per_segment = 1.0
control_pts = 5
n_segments = control_pts - 1

# kinematic constraints up to snap
order = 4

# 'A' matrix
a = np.zeros((2 * order * n_segments, 2 * order * n_segments)) # 16 * 16
#print (a)

# 'B' matrix 
b = np.zeros((2 * order * n_segments, 1))

# desired outputs of 4 (flat)
dim = 3
pos_traj = np.zeros((dim, control_pts))
pos_traj[0, :] = np.array([0.0, 4.0, 4.0, 3.0, -2.0]) #x
pos_traj[1, :] = np.array([0.0, 1.0, 2.0, 4.0, 3.0]) #y
pos_traj[2, :] = np.array([0.0, 3.0, 3.0, 3.0, 3.0]) #z, might not need a min snap for this tbh, will fluctuate the altitude
yaw_traj = np.zeros((1,control_pts)) #yaw
y_ref = np.concatenate((pos_traj,yaw_traj))
no_of_flat_outputs = y_ref.shape[0]

print (y_ref)

# Number of coefficients based on polynomial order
No_of_Coeff = order * 2

#print (y_ref[0, :])
#print (flat_outputs,segments)
poly_coefficients = np.zeros((n_segments, No_of_Coeff, no_of_flat_outputs)) # 1,8,3

print (poly_coefficients.shape)

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


def a_matrix(a,n_segments):
    m = matrix_generation(specified_time_per_segment)

    row_count = 1
    col_count = 0
    for i in range(n_segments):
        if row_count % 3 == 0:
            col_count += 1
             
        a[i*order:i+1*order, col_count*8:col_count+1*8] = m[0:4,:]
        row_count +=1    

    return a

    
def state_generation(x):
    # x is any dim in y_ref, a tuple
    n = x.shape[0] - 1 # number of segments [0,5,10] eg. = 2 segments 

    state_x_beginning = np.zeros((8 * 1))
    state_x_beginning[:8] = np.array([x[0], 0, 0, 0, 0, 0, 0, 0]).T # kinematic requirements at the start

    state_x_middle = np.zeros((8 * 1))
    state_x_middle[:8] = np.array([x[1], 0, 0, 0, 0, 0, 0, 0]).T # kinematic requirements in the middle
    
    state_x_end = np.zeros((8 * 1))
    state_x_end[:8] = np.array([x[2], 0, 0, 0, 0, 0, 0, 0]).T # kinematic requirements at the end

    
    
    
    print (state_x_beginning, state_x_middle, state_x_end)

    
    
    
    
    
    #big_x[-4:] = np.array([x[-1], 0, 0, 0]).T

    #for i in range(1, n):
    #    big_x[8 * (i - 1) + 4:8 * (i - 1) + 8 + 4] = np.array([x[i], 0, 0, 0, 0, 0, 0, x[i]]).T

    #return big_x

print (matrix_generation(1)[0:4,:].shape)
print (a_matrix(a,n_segments))











