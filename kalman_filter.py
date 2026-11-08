import numpy as np



# Part 3: Comment the code explaining each part
class kalman_filter:
    
    # Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        self.P=P # State Covariance Matrix
        self.Q=Q # Process Noise/Uncertainty
        self.R=R # Sensor Noise/Uncertainty
        self.x=x # State Vector
        self.dt=dt # Time Step
        
    # Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        self.A = self.jacobian_A() #6x6 Update motion jacobian in A
        self.motion_model() #6x1 update state vector with prediction based on motion model
        
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q #P-6x6 Update Covariance matrix based on jacobian of A 

    # Part 3: Calculate the Kalman gain and correct your estimates
    def update(self, z):
        
        self.C = self.jacobian_H() #C-(4x6) - Update measurement jacobian and store in C
                
        self.S = np.dot(np.dot(self.C,self.P),self.C.T) + self.R #S-4x4 R-4x4 Intermediate solving step for kalman gain
        kalman_gain= np.dot(np.dot(self.P,self.C.T),np.linalg.inv(self.S)) #k-(6x4) Solve for kalman gain
        
        self.x= self.x + np.dot(kalman_gain,(z-self.measurement_model())) #x-(6x1) Update state vector with measurement correction
        
        I = np.eye(6) #Identity matrix (6x6)
        self.P= np.dot((I - np.dot(kalman_gain, self.C)),self.P) #P-6x6 Update covariance matrix
        
    
    # Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# Linear Velocity
            w,# Angular Velocity
            vdot, # ax - Linear acceleration in driving direction
            v*w, # ay - Linear acceleration perpendicular to driving direction
        ])
        
    # Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + v * np.cos(th) * dt, # x position
            y + v * np.sin(th) * dt, # y position
            th + w * dt, # Theta
            w, # Angular velocity
            v  + vdot*dt, # Velocity
            vdot, # Acceleration
        ])
        


    
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        # The jacobian matrix is taken by finding the 
        # partial derivative of each index
        # It is the transpose of the format in notes
        
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0,    -v*np.sin(th)*dt, 0, np.cos(th)*dt,  0], # Partial derivatives of x position
            [0, 1,    v*np.cos(th)*dt, 0,  np.sin(th)*dt,  0], # Partial derivatives of y position
            [0, 0,                1, dt,           0,  0],     # Partial derivatives of theta
            [0, 0,                0, 1,            0,  0],     # Partial derivatives of angular velocity
            [0, 0,                0, 0,            1,  dt],    # Partial derivatives of velocity
            [0, 0,                0, 0,            0,  1 ]     # Partial derivatives of acceleration
        ])
    
    
    # Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        
        # The jacobian matrix is taken by finding the partial derivative of each index
        # Transpose of the format in notes
                
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # Partial derivatives of v
            [0,0,0  , 1, 0, 0], # Partial derivatives of w
            [0,0,0  , 0, 0, 1], # Partial derivatives of ax
            [0,0,0  , v, w, 0], # Partial derivatives of ay
        ])
        
    # Part 3: return the states here    
    def get_states(self):
        return self.x # Return the state vector
