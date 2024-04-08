import pybullet
import pybullet_data
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd
import struct 
import random

#from classes.pybullet_utils import LinkState
from pybullet_utils import LinkState
from pybullet_utils import JointState

from leg_swing_stance_angle_generator import Generate_angle
from leg_trot import Leg

pybullet.connect(pybullet.GUI)
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setGravity(0, 0, -9.81)
# Set a smaller time step for more accurate simulation
#pybullet.setTimeStep(1 / 100.0)
startPos = [0,0,0.35]
#startOrientation = pybullet.getQuaternionFromEuler([0,0,0])
# Load robot URDF file created in solidworks
#planeId = pybullet.loadURDF("plane.urdf")


""" Following code generates Heightfield

numHeightfieldRows = 256
numHeightfieldColumns = 256
heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns 
for j in range (int(numHeightfieldColumns/2)):
    for i in range (int(numHeightfieldRows/2) ):
      height = random.uniform(0,0.05)
      heightfieldData[2*i+2*j*numHeightfieldRows]=height
      heightfieldData[2*i+1+2*j*numHeightfieldRows]=height
      heightfieldData[2*i+(2*j+1)*numHeightfieldRows]=height
      heightfieldData[2*i+1+(2*j+1)*numHeightfieldRows]=height
    
terrainShape = pybullet.createCollisionShape(shapeType = pybullet.GEOM_HEIGHTFIELD, meshScale=[.05,.05,1], heightfieldTextureScaling=(numHeightfieldRows-1)/2, heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns)
terrain  = pybullet.createMultiBody(0, terrainShape)
pybullet.resetBasePositionAndOrientation(terrain,[0,0,0], [0,0,0,1])
"""

robot = pybullet.loadURDF(r'Paws/urdf/Paws.urdf',startPos,useFixedBase=0)
#cubePos, cubeOrn = pybullet.getBasePositionAndOrientation(robot)


# Assuming you have the collision shape IDs for each toe joint
TOE_JOINT_INDEX_FR = 3
TOE_JOINT_INDEX_FL = 7
TOE_JOINT_INDEX_RR = 11
TOE_JOINT_INDEX_RL = 15

# Set friction coefficients
lateral_friction = 1.0  # Adjust this value as needed
spinning_friction = 0.1  # Adjust this value as needed

knee_min_position = -1.57
knee_max_position = 1.57
foot_lower_limit = -2.2689
foot_upper_limit = 2.2689

sit_angles = np.array ([0.38])
stand_angles = np.array ([0.785,0.698])

hip_angles_SLR = np.array ([0.1537])
knee_angles_SLR = np.array ([-1.222])
foot_angles_SLR = np.array ([1.815])
hip_angles_PRO = np.array ([0.3074])
knee_angles_PRO = np.array ([-0.413])
foot_angles_PRO = np.array ([1.815])
hip_angles_FT = np.array ([0.1537])
knee_angles_FT = np.array ([-0.262]) #calculated from real dog angles
foot_angles_FT = np.array ([1.427]) #calculated from real dog angles

knee_angles_STANCE = np.array ([knee_min_position + stand_angles[0]])
foot_angles_STANCE = np.array ([foot_upper_limit - stand_angles[1]])
hip_angles_STANCE = np.array ([0.0015])

target_position_knee = 0
target_position_foot = 0
target_position_hip = 0

previous_position_knee = 0
previous_position_foot = 0
previous_position_hip = 0

# Define joint indexes
HIP_JOINT_INDEX_FR = 0
KNEE_JOINT_INDEX_FR = 1
FOOT_JOINT_INDEX_FR = 2
TOE_JOINT_INDEX_FR = 3
HIP_JOINT_INDEX_FL = 4
KNEE_JOINT_INDEX_FL = 5
FOOT_JOINT_INDEX_FL = 6
TOE_JOINT_INDEX_FL = 7
HIP_JOINT_INDEX_RR = 8
KNEE_JOINT_INDEX_RR = 9
FOOT_JOINT_INDEX_RR = 10
TOE_JOINT_INDEX_RR = 11
HIP_JOINT_INDEX_RL = 12
KNEE_JOINT_INDEX_RL = 13
FOOT_JOINT_INDEX_RL = 14
TOE_JOINT_INDEX_RL = 15

# Define link indexes
HIP_LINK_INDEX_FR = 0
KNEE_LINK_INDEX_FR = 1
FOOT_LINK_INDEX_FR = 2
TOE_LINK_INDEX_FR = 3
HIP_LINK_INDEX_FL = 4
KNEE_LINK_INDEX_FL = 5
FOOT_LINK_INDEX_FL = 6
TOE_LINK_INDEX_FL = 7
HIP_LINK_INDEX_RR = 8
KNEE_LINK_INDEX_RR = 9
FOOT_LINK_INDEX_RR = 10
TOE_LINK_INDEX_RR = 11
HIP_LINK_INDEX_RL = 12
KNEE_LINK_INDEX_RL = 13
FOOT_LINK_INDEX_RL = 14
TOE_LINK_INDEX_RL = 15

# Set the range of motion 
knee_min_position = -1.57
knee_max_position = 1.57
foot_lower_limit = -2.2689
foot_upper_limit = 2.2689

L1 = 0.15
L2 = 0.20
D = 0.25
PI = math.pi
STEP_LENGTH = 0.1
HIP_ANGLE = 0.0015
previous_pitch = 0
SWING_PHASE = False

target_position_knee = 0
target_position_foot = 0
target_position_hip = 0


start_time = 0
end_time = 0
elapsed_time = 0
previous_time = 0

state_FR = 0
FR_Stance = False
SW_F_knee_link_id = 0
SW_F_foot_link_id = 0 
SW_R_knee_link_id = 0
SW_R_foot_link_id = 0
ST_F_knee_link_id = 0
ST_F_foot_link_id = 0
ST_R_knee_link_id = 0
ST_R_foot_link_id = 0


swing_x = []
swing_z = []
stance_x = []
stance_z = []

data_list = []
data_list_COM = []
previous_time_FR_Hip = 0
previous_time_FL_Hip = 0
previous_time_RR_Hip = 0
previous_time_RL_Hip = 0

previous_time_FR_Knee = 0
previous_time_FL_Knee = 0
previous_time_RR_Knee = 0
previous_time_RL_Knee= 0

previous_time_FR_Foot = 0
previous_time_FL_Foot = 0
previous_time_RR_Foot = 0
previous_time_RL_Foot = 0

previous_angle_FR_Hip= 0
previous_angle_FL_Hip= 0
previous_angle_RR_Hip= 0
previous_angle_RL_Hip= 0

previous_angle_FR_Knee = 0
previous_angle_FL_Knee = 0
previous_angle_RR_Knee = 0
previous_angle_RL_Knee = 0

previous_angle_FR_Foot = 0
previous_angle_FL_Foot = 0
previous_angle_RR_Foot = 0
previous_angle_RL_Foot = 0


# Load data from the CSV file
# trot curve
data = np.loadtxt('quadruped_parabolic_arc_coordinates.csv', delimiter=',', skiprows=1)  # Assuming headers are present
modified_data = data[::1]
total_rows = modified_data.shape[0]
half_rows = total_rows // 2

swing_data = modified_data[:half_rows]
stance_data = modified_data[half_rows:]
stance_data_x = stance_data[:,0]
swing_data_x = swing_data[:,0]
stance_data_z = stance_data[:,1]
swing_data_z = swing_data[:,1]

# Plot the first dataset with red color and solid line
""""
plt.plot(stance_data_x, stance_data_z, color='red', linestyle='--')
plt.plot(swing_data_x, swing_data_z, color='blue', linestyle='--')
plt.xlabel('X-axis (m)')
plt.ylabel('Z-axis (m)')
plt.title('Parabolic Arc Trajectory')
plt.grid(True)
plt.legend(['Stance ', 'Swing'])
plt.show()
"""
#np.savetxt('swing.csv', swing_data, delimiter=',', header='x, z', comments='')
#np.savetxt('stance.csv', stance_data, delimiter=',', header='x, z', comments='')

com_x = []
com_y = []
com_z = []

com_roll = []
com_pitch = []
com_yaw = []

com_time = []

def isclose(a, b, rel_tol=1e-9, abs_tol=0.0): #Calculates if set position is close to current position
    
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

def record_COM_angle(robot , previous_time): #Takes various Cente of Mss (COM) data
    
    # Get the current simulation time
    current_time = time.time() - start_time
    time_diff = current_time - previous_time
    com_pos, com_orientation = pybullet.getBasePositionAndOrientation(robot)

    com_roll.append(com_orientation[0])
    com_pitch.append(com_orientation[1])
    com_yaw.append(com_orientation[2])
    
    # Append COM coordinates to lists
    com_x.append(com_pos[0])
    com_y.append(com_pos[1])
    com_z.append(com_pos[2])

    com_time.append(current_time)
    previous_time = current_time 
    
 
    return com_time, com_x, com_y, com_z, com_roll, com_pitch, com_yaw

def record_joint_angle(robot , joint_index, I): #Definition records joint angles and time recorded for a cycle
    # Get the current simulation time
    current_time = time.time() - start_time
    # Get joint state
    joint_states = pybullet.getJointStates(robot, [joint_index])[0]
    joint_info = pybullet.getJointInfo(robot, joint_index)
    joint_name = joint_info[1]
    joint_name_str = joint_name.decode('utf-8')
    joint_angle = joint_states[0]  # Joint angle is the first element of the tuple
    data_list.append({ 'I': I, 'Current Time': current_time, 'Joint Name': joint_name_str, 'Joint Angle': joint_angle})
    
    
hip_link_state_FR = LinkState(pybullet.getLinkState(robot, HIP_LINK_INDEX_FR))
knee_link_state_FR = LinkState(pybullet.getLinkState(robot, KNEE_LINK_INDEX_FR))
foot_link_state_FR = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_FR))
toe_link_state_FR = LinkState(pybullet.getLinkState(robot, TOE_LINK_INDEX_FR))

hip_link_state_RR = LinkState(pybullet.getLinkState(robot, HIP_LINK_INDEX_RR))
knee_link_state_RR = LinkState(pybullet.getLinkState(robot, KNEE_LINK_INDEX_RR))
foot_link_state_RR = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_RR))
toe_link_state_RR = LinkState(pybullet.getLinkState(robot, TOE_LINK_INDEX_RR))

hip_link_state_FL = LinkState(pybullet.getLinkState(robot, HIP_LINK_INDEX_FL))
knee_link_state_FL = LinkState(pybullet.getLinkState(robot, KNEE_LINK_INDEX_FL))
foot_link_state_FL = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_FL))
toe_link_state_FL = LinkState(pybullet.getLinkState(robot, TOE_LINK_INDEX_FL))

hip_link_state_RL = LinkState(pybullet.getLinkState(robot, HIP_LINK_INDEX_RL))
knee_link_state_RL = LinkState(pybullet.getLinkState(robot, KNEE_LINK_INDEX_RL))
foot_link_state_RL = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_RL))
toe_link_state_RL = LinkState(pybullet.getLinkState(robot, TOE_LINK_INDEX_RL))

hip_joint_state_FR = JointState(pybullet.getJointState(robot, HIP_JOINT_INDEX_FR))
knee_joint_state_FR = JointState(pybullet.getJointState(robot, KNEE_JOINT_INDEX_FR))
foot_joint_state_FR = JointState(pybullet.getJointState(robot, FOOT_JOINT_INDEX_FR))
toe_joint_state_FR = JointState(pybullet.getJointState(robot, TOE_JOINT_INDEX_FR))
    
hip_joint_state_RR = JointState(pybullet.getJointState(robot, HIP_JOINT_INDEX_RR))
knee_joint_state_RR = JointState(pybullet.getJointState(robot, KNEE_JOINT_INDEX_RR))
foot_joint_state_RR = JointState(pybullet.getJointState(robot, FOOT_JOINT_INDEX_RR))
toe_joint_state_RR = JointState(pybullet.getJointState(robot, TOE_JOINT_INDEX_RR)) 
hip_joint_state_FR = JointState(pybullet.getJointState(robot, HIP_JOINT_INDEX_FR))
knee_joint_state_FR = JointState(pybullet.getJointState(robot, KNEE_JOINT_INDEX_FR))
foot_joint_state_FR = JointState(pybullet.getJointState(robot, FOOT_JOINT_INDEX_FR))
toe_joint_state_FR = JointState(pybullet.getJointState(robot, TOE_JOINT_INDEX_FR))
        
hip_joint_state_RR = JointState(pybullet.getJointState(robot, HIP_JOINT_INDEX_RR))
knee_joint_state_RR = JointState(pybullet.getJointState(robot, KNEE_JOINT_INDEX_RR))
foot_joint_state_RR = JointState(pybullet.getJointState(robot, FOOT_JOINT_INDEX_RR))
toe_joint_state_RR = JointState(pybullet.getJointState(robot, TOE_JOINT_INDEX_RR)) 


#Finding original Body Length
length_hip_distance_z = (hip_link_state_FR.link_world_position_z - hip_link_state_RR.link_world_position_z)**2
length_hip_distance_x = (hip_link_state_FR.link_world_position_x - hip_link_state_RR.link_world_position_x)**2
length_hip_distance_y = (hip_link_state_FR.link_world_position_y - hip_link_state_RR.link_world_position_y)**2
OG_Body_length = math.sqrt(length_hip_distance_x + length_hip_distance_z)

#Finding original Body width
width_hip_distance_z = (hip_link_state_FR.link_world_position_z - hip_link_state_FL.link_world_position_z)**2
width_hip_distance_x = (hip_link_state_FR.link_world_position_x - hip_link_state_FL.link_world_position_x)**2
width_hip_distance_y = (hip_link_state_FR.link_world_position_y - hip_link_state_FL.link_world_position_y)**2
OG_Body_width = math.sqrt(width_hip_distance_y + width_hip_distance_z)

foot_link_state_FR = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_FR))
foot_link_state_RR = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_RR))
foot_link_state_FL = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_FL))

#Finding original distance between front/rear & left/right foot end effectors
OG_foot_length = toe_link_state_FR.link_world_position_x - toe_link_state_RR.link_world_position_x
OG_foot_width = toe_link_state_FR.link_world_position_y - toe_link_state_FL.link_world_position_y

start_time = time.time() 

while True:

    hip_link_state = LinkState(pybullet.getLinkState(robot, HIP_LINK_INDEX_FR))
    knee_link_state = LinkState(pybullet.getLinkState(robot, KNEE_LINK_INDEX_FR))
    foot_link_state = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_FR))
    toe_link_state = LinkState(pybullet.getLinkState(robot, TOE_LINK_INDEX_FR))

    hip_joint_state = JointState(pybullet.getJointState(robot, HIP_JOINT_INDEX_FR))
    knee_joint_state = JointState(pybullet.getJointState(robot, KNEE_JOINT_INDEX_FR))
    foot_joint_state = JointState(pybullet.getJointState(robot, FOOT_JOINT_INDEX_FR))
    toe_joint_state = JointState(pybullet.getJointState(robot, TOE_JOINT_INDEX_FR)) 


##########################################################################################################################################
    if (state_FR == -1):
        
        target_position_knee =knee_min_position + stand_angles[0]
        target_position_foot =foot_upper_limit - stand_angles[1]


        pybullet.setJointMotorControl2(robot, KNEE_LINK_INDEX_FR, pybullet.POSITION_CONTROL, targetPosition=target_position_knee)
        pybullet.setJointMotorControl2(robot, FOOT_LINK_INDEX_FR, pybullet.POSITION_CONTROL, targetPosition=target_position_foot)
        pybullet.setJointMotorControl2(robot, KNEE_LINK_INDEX_FL, pybullet.POSITION_CONTROL, targetPosition=-(target_position_knee))
        pybullet.setJointMotorControl2(robot, FOOT_LINK_INDEX_FL, pybullet.POSITION_CONTROL, targetPosition= target_position_foot)
        pybullet.setJointMotorControl2(robot, KNEE_LINK_INDEX_RR, pybullet.POSITION_CONTROL, targetPosition=target_position_knee)
        pybullet.setJointMotorControl2(robot, FOOT_LINK_INDEX_RR, pybullet.POSITION_CONTROL, targetPosition=target_position_foot)                              
        pybullet.setJointMotorControl2(robot, KNEE_LINK_INDEX_RL, pybullet.POSITION_CONTROL, targetPosition=-(target_position_knee))
        pybullet.setJointMotorControl2(robot, FOOT_LINK_INDEX_RL, pybullet.POSITION_CONTROL, targetPosition= target_position_foot)
        print('STATE', state_FR)
        pybullet.stepSimulation()
        time.sleep(0.1)
############################################################################################################################################

    if (state_FR == 0):

        #first step

        swing_arc_x = swing_data[1,0]
        swing_arc_z = swing_data[1,1]
        stance_arc_x = stance_data[1,0]
        stance_arc_z = stance_data[1,1]
            
        dist_hip_toe_z = (hip_link_state_FR.link_world_position_z - toe_link_state_FR.link_world_position_z)**2
        dist_hip_toe_x = (hip_link_state_FR.link_world_position_x - toe_link_state_FR.link_world_position_x)**2
        dist_hip_toe = math.sqrt(dist_hip_toe_z + dist_hip_toe_x)

        hip_offset_z = (hip_link_state_FR.link_world_position_z - knee_link_state_FR.link_world_position_z)**2
        hip_offset_y = (hip_link_state_FR.link_world_position_y - knee_link_state_FR.link_world_position_y)**2
        hip_offset = math.sqrt(hip_offset_z + hip_offset_y)

 ###########################################################################################################################################
        #classes to generate the swing and stance angles based on co-ords 

        SWING_Arc = Generate_angle(swing_arc_x, swing_arc_z , OG_foot_length, OG_Body_width, OG_Body_length, robot)
        STANCE_Arc = Generate_angle(stance_arc_x, stance_arc_z, OG_foot_length, OG_Body_width, OG_Body_length, robot)

        SW_theta_kneeA_F,SW_theta_footA_F,SW_theta_kneeA_R,SW_theta_footA_R,SW_theta_kneeB_F,SW_theta_footB_F,SW_theta_kneeB_R,SW_theta_footB_R, roll_Angle = SWING_Arc.generate_angle() 
        ST_theta_kneeA_F,ST_theta_footA_F,ST_theta_kneeA_R,ST_theta_footA_R,ST_theta_kneeB_F,ST_theta_footB_F,ST_theta_kneeB_R,ST_theta_footB_R,roll_Angle = STANCE_Arc.generate_angle()
############################################################################################################################################
        knee_angles_SWING  = [SW_theta_kneeA_F,SW_theta_kneeA_R,SW_theta_kneeB_F,SW_theta_kneeB_R]
        foot_angles_SWING  = [SW_theta_footA_F,SW_theta_footA_R,SW_theta_footB_F,SW_theta_footB_R]
        knee_angles_STANCE = [ST_theta_kneeA_F,ST_theta_kneeA_R,ST_theta_kneeB_F,ST_theta_kneeB_R]
        foot_angles_STANCE = [ST_theta_footA_F,ST_theta_footA_R,ST_theta_footB_F,ST_theta_footB_R]
############################################################################################################################################
        target_position_knee =  knee_angles_SWING[0]    
############################################################################################################################################

        FR_RL_pair = Leg(1 ,roll_Angle, -roll_Angle, knee_angles_SWING[0], foot_angles_SWING[0], knee_angles_SWING[3], foot_angles_SWING[3], knee_angles_STANCE[2], foot_angles_STANCE[2],knee_angles_STANCE[1], foot_angles_STANCE[1],HIP_LINK_INDEX_FR,HIP_LINK_INDEX_RL,HIP_LINK_INDEX_FL,HIP_LINK_INDEX_RR,
                        KNEE_LINK_INDEX_FR, FOOT_LINK_INDEX_FR,KNEE_LINK_INDEX_RL, FOOT_LINK_INDEX_RL, KNEE_LINK_INDEX_FL, FOOT_LINK_INDEX_FL, KNEE_LINK_INDEX_RR, FOOT_LINK_INDEX_RR, FR_Stance, robot)
        FL_RR_pair = Leg(1,roll_Angle, -roll_Angle, knee_angles_SWING[2], foot_angles_SWING[2],knee_angles_SWING[1], foot_angles_SWING[1], knee_angles_STANCE[0], foot_angles_STANCE[0], knee_angles_STANCE[3], foot_angles_STANCE[3],HIP_LINK_INDEX_FL,HIP_LINK_INDEX_RR,HIP_LINK_INDEX_FR,HIP_LINK_INDEX_RL,
                         KNEE_LINK_INDEX_FL, FOOT_LINK_INDEX_FL,KNEE_LINK_INDEX_RR, FOOT_LINK_INDEX_RR, KNEE_LINK_INDEX_FR, FOOT_LINK_INDEX_FR, KNEE_LINK_INDEX_RL, FOOT_LINK_INDEX_RL, FR_Stance, robot)
                
        FL_RR_pair.move_motors()
        
        pybullet.stepSimulation()
        time.sleep(0.1)
        state_FR = 1
        
##################################################################################################################################
    
    
    if isclose(knee_joint_state.joint_position, target_position_knee , rel_tol=1e-03) :
         state_FR = +1
         if state_FR >= 1:
            state_FR = 1


    cycle = 0
        
    if (state_FR >= 1):
        
        for I in range (0, 400):

            hip_link_state_FR = LinkState(pybullet.getLinkState(robot, HIP_LINK_INDEX_FR))
            hip_link_state_RR = LinkState(pybullet.getLinkState(robot, HIP_LINK_INDEX_RR))
            hip_link_state_FL = LinkState(pybullet.getLinkState(robot, HIP_LINK_INDEX_FL))
            hip_link_state_RL = LinkState(pybullet.getLinkState(robot, HIP_LINK_INDEX_RL))

            knee_link_state_FR = LinkState(pybullet.getLinkState(robot, KNEE_LINK_INDEX_FR))
            foot_link_state_FR = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_FR))
            toe_link_state_FR = LinkState(pybullet.getLinkState(robot, TOE_LINK_INDEX_FR))
            knee_link_state_RR = LinkState(pybullet.getLinkState(robot, KNEE_LINK_INDEX_RR))
            foot_link_state_RR = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_RR))
            toe_link_state_RR = LinkState(pybullet.getLinkState(robot, TOE_LINK_INDEX_RR))

            hip_joint_state_FR = JointState(pybullet.getJointState(robot, HIP_JOINT_INDEX_FR))
            knee_joint_state_FR = JointState(pybullet.getJointState(robot, KNEE_JOINT_INDEX_FR))
            foot_joint_state_FR = JointState(pybullet.getJointState(robot, FOOT_JOINT_INDEX_FR))
            toe_joint_state_FR = JointState(pybullet.getJointState(robot, TOE_JOINT_INDEX_FR)) 
            hip_joint_state_RR = JointState(pybullet.getJointState(robot, HIP_JOINT_INDEX_RR))
            knee_joint_state_RR = JointState(pybullet.getJointState(robot, KNEE_JOINT_INDEX_RR))
            foot_joint_state_RR = JointState(pybullet.getJointState(robot, FOOT_JOINT_INDEX_RR))
            toe_joint_state_RR = JointState(pybullet.getJointState(robot, TOE_JOINT_INDEX_RR)) 
            

            swing_arc_x = swing_data[I,0]
            swing_arc_z = swing_data[I,1]
            stance_arc_x = stance_data[I,0]
            stance_arc_z = stance_data[I,1]

            
            dist_hip_toe_z = (hip_link_state_FR.link_world_position_z - toe_link_state_FR.link_world_position_z)**2
            dist_hip_toe_x = (hip_link_state_FR.link_world_position_x - toe_link_state_FR.link_world_position_x)**2
            dist_hip_toe = math.sqrt(dist_hip_toe_z + dist_hip_toe_x)

            hip_offset_z = (hip_link_state_FR.link_world_position_z - knee_link_state_FR.link_world_position_z)**2
            hip_offset_y = (hip_link_state_FR.link_world_position_y - knee_link_state_FR.link_world_position_y)**2
            hip_offset = math.sqrt(hip_offset_z + hip_offset_y)

            ############################################################################################################################################
            #classes to generate the swing and stance angles based on co-ords 

            SWING_Arc = Generate_angle(swing_arc_x, swing_arc_z , OG_foot_length, OG_Body_width, OG_Body_length, robot)
            STANCE_Arc = Generate_angle(stance_arc_x, stance_arc_z, OG_foot_length, OG_Body_width, OG_Body_length, robot)

            SW_theta_kneeA_F,SW_theta_footA_F,SW_theta_kneeA_R,SW_theta_footA_R,SW_theta_kneeB_F,SW_theta_footB_F,SW_theta_kneeB_R,SW_theta_footB_R, roll_Angle = SWING_Arc.generate_angle() 
            ST_theta_kneeA_F,ST_theta_footA_F,ST_theta_kneeA_R,ST_theta_footA_R,ST_theta_kneeB_F,ST_theta_footB_F,ST_theta_kneeB_R,ST_theta_footB_R,roll_Angle = STANCE_Arc.generate_angle()
            
            #############################################################################################################################################
                                
            #############################################################################################################################################
            #using stance and swing angles generate movemnet depnding on phases
            knee_angles_SWING  = [SW_theta_kneeA_F,SW_theta_kneeA_R,SW_theta_kneeB_F,SW_theta_kneeB_R]
            foot_angles_SWING  = [SW_theta_footA_F,SW_theta_footA_R,SW_theta_footB_F,SW_theta_footB_R]
            knee_angles_STANCE = [ST_theta_kneeA_F,ST_theta_kneeA_R,ST_theta_kneeB_F,ST_theta_kneeB_R]
            foot_angles_STANCE = [ST_theta_footA_F,ST_theta_footA_R,ST_theta_footB_F,ST_theta_footB_R]


            FR_RL_pair = Leg(I ,roll_Angle, -roll_Angle, knee_angles_SWING[0], foot_angles_SWING[0], knee_angles_SWING[3], foot_angles_SWING[3], knee_angles_STANCE[2], foot_angles_STANCE[2],knee_angles_STANCE[1], foot_angles_STANCE[1],HIP_LINK_INDEX_FR,HIP_LINK_INDEX_RL,HIP_LINK_INDEX_FL,HIP_LINK_INDEX_RR,
                                KNEE_LINK_INDEX_FR, FOOT_LINK_INDEX_FR,KNEE_LINK_INDEX_RL, FOOT_LINK_INDEX_RL, KNEE_LINK_INDEX_FL, FOOT_LINK_INDEX_FL, KNEE_LINK_INDEX_RR, FOOT_LINK_INDEX_RR, FR_Stance, robot)
            FL_RR_pair = Leg(I,roll_Angle, -roll_Angle, knee_angles_SWING[2], foot_angles_SWING[2],knee_angles_SWING[1], foot_angles_SWING[1], knee_angles_STANCE[0], foot_angles_STANCE[0], knee_angles_STANCE[3], foot_angles_STANCE[3],HIP_LINK_INDEX_FL,HIP_LINK_INDEX_RR,HIP_LINK_INDEX_FR,HIP_LINK_INDEX_RL,
                                KNEE_LINK_INDEX_FL, FOOT_LINK_INDEX_FL,KNEE_LINK_INDEX_RR, FOOT_LINK_INDEX_RR, KNEE_LINK_INDEX_FR, FOOT_LINK_INDEX_FR, KNEE_LINK_INDEX_RL, FOOT_LINK_INDEX_RL, FR_Stance, robot)
         
            com_pos, _ = pybullet.getBasePositionAndOrientation(robot)
                
            if (FR_Stance ==  True):
                    
                    FR_RL_pair.move_motors()
                    COM_time = time.time()
                    com_time, com_x, com_y, com_z, com_roll, com_pitch, com_yaw= record_COM_angle(robot ,previous_time)
                            
            else:
                    
                    FL_RR_pair.move_motors()
                    COM_time = time.time()
                    com_time, com_x, com_y, com_z, com_roll, com_pitch, com_yaw= record_COM_angle(robot ,previous_time)

                    
            pybullet.stepSimulation()
            if (I == 399):
                FR_Stance = not FR_Stance
                
                cycle = cycle + 1

            if (cycle % 400 == 0):
                print(len(com_time))
                


            
            
             
                
            
                
            
            


        
            

    
    
    
    

    