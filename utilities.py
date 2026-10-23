from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename

        with open(self.filename, 'w') as file:
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            #Convert each value into a string, join with comma.
            line = ", ".join(str(val) for val in values_list)
            
            # write to file, add new line.
            file.write(line + "\n")
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table

#Function to convert from quaternion to euler angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]

    """
    #Unpack X, Y, Z, W values from quaternion
    x, y, z, w = quat.x, quat.y, quat.z, quat.w

    # Yaw calculation (Rot about z)
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    return yaw
    
def calculate_linear_error(current_pose, goal_pose):
    """
    Calculate the error of the xy position of the robot   
    """  
    error_linear= ((goal_pose[0]-current_pose[0])**2+(goal_pose[1]-current_pose[1])**2)**0.5

    return error_linear

def calculate_angular_error(current_pose, goal_pose):
    """
    Calculate the error of the angular orientation of the robot   
    """  
    error_angular = atan2(goal_pose[1], goal_pose[0])-current_pose[2]

    while error_angular > M_PI:
        error_angular -= 2*M_PI
    while error_angular < M_PI:
        error_angular +=2*M_PI
    
    return error_angular
