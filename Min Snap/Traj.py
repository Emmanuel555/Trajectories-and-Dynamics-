import numpy as np

def matrix_generation(ts):
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

i = 0
# initial condition of the first curve (0:4,0:8) 
b = matrix_generation(0.0)
#m[8 * i:8 * i + 4, 8 * i:8 * i + 8] = b[:4, :] 
init = b[:4, :]
print (init) 

# intermediary condition of the first curve (4:11,0:8)
b = matrix_generation(1.0)
#m[8 * i + 4:8 * i + 7 + 4, 8 * i:8 * i + 8] = b[:-1, :]
middle = b[:-1, :]
print (middle)

# starting condition of the second curve position and derivatives
b = matrix_generation(-1.0)
#m[8 * i + 4 + 1:8 * i + 4 + 7, 8 * (i + 1):8 * (i + 1) + 8] = -b[1:-1, :]
#m[8 * i + 4 + 7:8 * i + 4 + 8, 8 * (i + 1):8 * (i + 1) + 8] = b[0, :]
end_a = -b[1:-1, :]
print (end_a)
end_b = b[0,:]
print (end_b)

#print (b[:4, :] )