from math import exp

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1
# Type of trajectory
PARABOLA = 0; SIGMOID = 1
TRAJECTORY_SHAPE = PARABOLA


class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, n=50):
        points = []

        if TRAJECTORY_SHAPE == PARABOLA: #Parabola point definition
            # Bounds x=[0, 1.5]
            for i in range(n+1):
                x = 1.5*i/n
                y = x**2
                points.append([x, y])
        if TRAJECTORY_SHAPE == SIGMOID: #Sigmoid point definition
            # Bounds x=[0, 2.5]
            for i in range(n+1):
                x = 2.5*i/n
                y = 2 / (1 + exp(-2 * x)) - 1
                points.append([x, y])      

        # Return the list of point for trajectory
        return points

