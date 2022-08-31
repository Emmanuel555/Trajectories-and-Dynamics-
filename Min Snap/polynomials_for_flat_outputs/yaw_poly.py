import numpy as np
import matplotlib.pyplot as plt

# segment parameters
class yaw_polynomial:

    def __init__(self,order_no):
        
        """ specified_time_per_segment = 1.0
        control_pts = 5
        n_segments = control_pts - 1 """

        # kinematic constraints up to snap
        self.order = order_no

        # Initialise 'A' matrix
        # a = np.zeros((2 * order * n_segments, 2 * order * n_segments)) # 16 * 16
        # print (a)

        # Initialise 'B' matrix 
        # b = np.zeros((2 * order * n_segments, 1))

        # desired outputs of 4 (flat)
        """ dim = 3
        pos_traj = np.zeros((dim, control_pts))
        pos_traj[0, :] = np.array([0.0, 4.0, 4.0, 3.0, -2.0]) #x
        pos_traj[1, :] = np.array([0.0, 1.0, 2.0, 4.0, 3.0]) #y
        pos_traj[2, :] = np.array([0.0, 3.0, 3.0, 3.0, 3.0]) #z, might not need a min snap for this tbh, will fluctuate the altitude
        yaw_traj = np.zeros((1,control_pts)) #yaw
        y_ref = np.concatenate((pos_traj,yaw_traj))
        no_of_flat_outputs = y_ref.shape[0]
        velocity = 1.0 """
        # print (y_ref)

        # Number of coefficients based on polynomial order
        # No_of_Coeff = order * 2

        # print (y_ref[0, :])
        # print (flat_outputs,segments)
        # poly_coefficients = np.zeros((n_segments, No_of_Coeff, no_of_flat_outputs)) # 1,8,3

        # print (poly_coefficients.shape)

    #reference: https://timodenk.com/blog/cubic-spline-interpolation/
    def matrix_generation(self,ts):
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


    def a_matrix(self,a,n_segments):
        #time = np.zeros((n_segments * 2,1)) # assuming each segment only takes a second
        time = np.array([0,1,0,1,0,1,0,1])

        col_count = 0
        for i in range(n_segments*2):
            m = self.matrix_generation(time[i])
            if i % 2 == 0 and i > 0:
                col_count += 1
            #print (a[i*order:i+1*order, col_count*8:col_count+1*8])
            #print (a[i*order:i+1*order, col_count*8:col_count+1*8].shape)
            #print ("rows: ", i*order, "to", (i+1)*order, "cols: ", col_count*8, "to", (col_count+1)*8)
            #print (m[0:4,:])
            #print (m[0:4,:].shape)       
            a[i*self.order:(i+1)*self.order, col_count*8:(col_count+1)*8] = m[0:4,:]
            
        return a

        
    def state_generation(self,y_ref,velocity): # b matrix
        # x is any dim in y_ref, a tuple
        n = y_ref.shape[1] 
        y_ref_extended = np.zeros(((n-1),1,2))
        for c in range(n-1):
            y_ref_extended[c,0,0] = y_ref[0,c]
            y_ref_extended[c,0,1] = y_ref[0,c+1]

        state = np.zeros(((n-1)*2*self.order,1))

        # Assuming no acc or jerk requirements, both would be set to zero for now  
        # only testing for 1 dimension thus far....

        for i in range(n-1):
            if i == 0:
                init_velocity = 0.0
                end_velocity = velocity 
            elif i == (n-2):
                init_velocity = velocity
                end_velocity = 0.0
            else:
                init_velocity = velocity
                end_velocity = velocity 

            #print ((state[i*order*2:(i+1)*order*2,:]).shape)
            desired_states = np.zeros((self.order*2,1))
            #print (np.array([[y_ref_extended[i,0,0]], [velocity], [0], [0], [y_ref_extended[i,0,1]], [velocity], [0], [0]]).shape)      
            state[i*self.order*2:(i+1)*self.order*2] = np.array([[y_ref_extended[i,0,0]], [init_velocity], [0], [0], [y_ref_extended[i,0,1]], [end_velocity], [0], [0]])
        
        return state


    def test():
        a = np.array([[1,0,0,0,0,0],[1,1,1,1,1,1],[0,1,0,0,0,0],[0,1,2,3,4,5],[0,0,2,0,0,0],[0,0,2,6,12,20]])
        b= np.array([[0],[5],[0],[0],[0],[0]])
        x = np.linalg.solve(a,b) 

        # ans: x = 50t**3 - 75t**4 + 30t**5

        return x 


    def polynomial_generation(self,n_segments,velocity,y_ref):

        # Initialise 'A' matrix
        A = np.zeros((2 * self.order * n_segments, 2 * self.order * n_segments)) # 16 * 16
        #print (a)

        a = self.a_matrix(A,n_segments,velocity)
        b = self.state_generation(y_ref,velocity)
        #print (a[0:8,:])
        
        if a.shape[0] != b.shape[0]:
            print ("a & b matrices dun share the same dimensions")
            return None

        if np.linalg.det(a) <= 0:
            print ("singular matrix detected")
        else:
            coeff = np.linalg.solve(a,b) # order = (c0,c1,c2,c3,c4...)
        
        no_of_poly = np.zeros((int((coeff.shape[0])/(self.order*2)),self.order*2))
        print (no_of_poly.shape)
        for i in range(self.order):
            no_of_poly[i,:] = coeff.T[:,(i*self.order*2):(((i+1)*self.order*2))] 

        return no_of_poly

        #print (polynomial_generation(a,n_segments,velocity,order,y_ref))
    











