import pybullet
import pybullet_data
import time
import math
import numpy as np

from pybullet_utils import LinkState
from pybullet_utils import JointState
pybullet.connect(pybullet.GUI)
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setGravity(0, 0, -9.81)

startPos = [0,0,0.4]
planeId = pybullet.loadURDF("plane.urdf")
robot = pybullet.loadURDF(r'Paws/urdf/Paws.urdf',startPos,useFixedBase=0)

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

sit_angles = np.array ([0.349])
stand_angles = np.array ([0.785,0.698])


hip_angles_SLR = np.array ([0.3074])
knee_angles_SLR = np.array ([-1.222])
foot_angles_SLR = np.array ([1.815])
hip_angles_PRO = np.array ([0.3074])
knee_angles_PRO = np.array ([-0.413])
foot_angles_PRO = np.array ([1.815])
hip_angles_FT = np.array ([0.3074])
knee_angles_FT = np.array ([-0.262]) #calculated from real dog angles
foot_angles_FT = np.array ([1.427]) #calculated from real dog angles

target_position_knee = 0
target_position_foot = 0
target_position_hip = 0

previous_position_knee = 0
previous_position_foot = 0
previous_position_hip = 0
state = 1
L1 = 0.15
L2 = 0.20
D = 0.25
PI = math.pi
STEP_LENGTH = 0.1
HIP_ANGLE = 0.0015
PITCH_ANGLE = 0


debug_param_D = pybullet.addUserDebugParameter("Height",0.1, 0.4, 0.25)
debug_param_Step_Length = pybullet.addUserDebugParameter("Step length",-0.2, 0.2, 0.0)
debug_param_Hip_Angle = pybullet.addUserDebugParameter("Hip Angle",-0.8, 0.8, 0.0)
debug_param_pitch_Angle = pybullet.addUserDebugParameter("pitch Angle",-0.4, 0.4, 0.0)
debug_param_roll_Angle = pybullet.addUserDebugParameter("roll Angle",-0.8, 0.8, 0.0)

base_pos, base_orientation = pybullet.getBasePositionAndOrientation(robot)
base_euler = pybullet.getEulerFromQuaternion(base_orientation)
pitch = abs(base_euler[1])  # Pitch is the second element in the euler angles (yaw, pitch, roll)

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
#Finding world position distance between front and rear hips to calculate body length
length_hip_distance_z = (hip_link_state_FR.link_world_position_z - hip_link_state_RR.link_world_position_z)**2
length_hip_distance_x = (hip_link_state_FR.link_world_position_x - hip_link_state_RR.link_world_position_x)**2
length_hip_distance_y = (hip_link_state_FR.link_world_position_y - hip_link_state_RR.link_world_position_y)**2
OG_Body_length = math.sqrt(length_hip_distance_x + length_hip_distance_z)

#Finding world position distance between front and rear hips to calculate body length
width_hip_distance_z = (hip_link_state_FR.link_world_position_z - hip_link_state_FL.link_world_position_z)**2
width_hip_distance_x = (hip_link_state_FR.link_world_position_x - hip_link_state_FL.link_world_position_x)**2
width_hip_distance_y = (hip_link_state_FR.link_world_position_y - hip_link_state_FL.link_world_position_y)**2
OG_Body_width = math.sqrt(width_hip_distance_y + width_hip_distance_z)

foot_link_state_FR = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_FR))
foot_link_state_RR = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_RR))
foot_link_state_FL = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_FL))

OG_foot_length = toe_link_state_FR.link_world_position_x - toe_link_state_RR.link_world_position_x
OG_foot_width = toe_link_state_FR.link_world_position_y - toe_link_state_FL.link_world_position_y


while True:
    #Initialising Sliders
    D = pybullet.readUserDebugParameter(debug_param_D)
    STEP_LENGTH = pybullet.readUserDebugParameter(debug_param_Step_Length)
    HIP_ANGLE = pybullet.readUserDebugParameter(debug_param_Hip_Angle)
    PITCH_ANGLE = pybullet.readUserDebugParameter(debug_param_pitch_Angle)
    ROLL_ANGLE = pybullet.readUserDebugParameter(debug_param_roll_Angle)

    #Link and joint states
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
    hip_joint_state_FL = JointState(pybullet.getJointState(robot, HIP_JOINT_INDEX_FL))
    knee_joint_state_FL = JointState(pybullet.getJointState(robot, KNEE_JOINT_INDEX_FL))
    foot_joint_state_FL = JointState(pybullet.getJointState(robot, FOOT_JOINT_INDEX_FL))
    toe_joint_state_FL = JointState(pybullet.getJointState(robot, TOE_JOINT_INDEX_FL))
    hip_joint_state_RL = JointState(pybullet.getJointState(robot, HIP_JOINT_INDEX_RL))
    knee_joint_state_RL = JointState(pybullet.getJointState(robot, KNEE_JOINT_INDEX_RL))
    foot_joint_state_RL = JointState(pybullet.getJointState(robot, FOOT_JOINT_INDEX_RL))
    toe_joint_state_RL = JointState(pybullet.getJointState(robot, TOE_JOINT_INDEX_RL)) 

    base_pos, base_orientation = pybullet.getBasePositionAndOrientation(robot)
    base_euler = pybullet.getEulerFromQuaternion(base_orientation)
    print("base_pos",base_pos)
    #Finding roll and pitch values of COM
    roll =  base_euler[0]
    pitch = base_euler[1]
    
    #Finding world position distance between front and rear hips to calculate body length
    length_hip_distance_z = (hip_link_state_FR.link_world_position_z - hip_link_state_RR.link_world_position_z)**2
    length_hip_distance_x = (hip_link_state_FR.link_world_position_x - hip_link_state_RR.link_world_position_x)**2
    length_hip_distance_y = (hip_link_state_FR.link_world_position_y - hip_link_state_RR.link_world_position_y)**2
    Body_length = math.sqrt(length_hip_distance_x + length_hip_distance_z)

    #Finding world position distance between front and rear hips to calculate body length
    width_hip_distance_z = (hip_link_state_FR.link_world_position_z - hip_link_state_FL.link_world_position_z)**2
    width_hip_distance_x = (hip_link_state_FR.link_world_position_x - hip_link_state_FL.link_world_position_x)**2
    width_hip_distance_y = (hip_link_state_FR.link_world_position_y - hip_link_state_FL.link_world_position_y)**2
    Body_width = math.sqrt(width_hip_distance_y + width_hip_distance_z)

    ############################################################################################################################
    #Pitch Calculations
    pitch_Angle = PITCH_ANGLE #- pitch
    D_Pitch = 0.5*(Body_length)*math.sin(pitch_Angle)
    Foot_Pitch = 0.5*(Body_length)*math.sin(pitch_Angle)
    D_Front = D + D_Pitch
    D_Rear =  D - D_Pitch

    foot_length = toe_link_state_FR.link_world_position_x - toe_link_state_RR.link_world_position_x
    Foot_Pitch_Offset = 0.5*(OG_foot_length - foot_length)
    
    Theta_pitch_offset_F = math.atan(Foot_Pitch_Offset/D_Front)
    Theta_pitch_offset_R = math.atan(Foot_Pitch_Offset/D_Rear)
    print("Theta_pitch_offset_F",Theta_pitch_offset_F)
    
    Shoulder_Angle_F = pitch_Angle - Theta_pitch_offset_F 
    Shoulder_Angle_R  = pitch_Angle + Theta_pitch_offset_R

    H_Front = D_Front/math.cos(Theta_pitch_offset_F)
    H_Rear  = D_Rear/math.cos(Theta_pitch_offset_R)

    Z_P_F = H_Front*math.cos(Shoulder_Angle_F)
    Z_P_R = H_Rear*math.cos(Shoulder_Angle_R)
    print("Z_P_R", Z_P_R)
    X_P_F = H_Front*math.sin(Shoulder_Angle_F)
    X_P_R = H_Rear*math.sin(Shoulder_Angle_R)
    
    STEP_LENGTH_F = STEP_LENGTH - X_P_F
    print("SL_F",STEP_LENGTH_F)
    STEP_LENGTH_R = STEP_LENGTH - X_P_R
    print("SL_R",STEP_LENGTH_R)
   
    ############################################################################################################################
    #Roll Calculations
    print("ogBODY WIDTH", OG_Body_width)
    print("BODY WIDTH", Body_width)
    print("ogBODY length", OG_Body_length)
    print("BODY length", Body_length)
    Roll_D = math.sin(ROLL_ANGLE)*0.5*Body_width
    Roll_Body = math.cos(ROLL_ANGLE)*0.5*Body_width
    print("ROLL_D", Roll_D)
    print("ROLL_Body", Roll_Body)
    DrF = Z_P_F - Roll_D
    DrR = Z_P_R - Roll_D
    DlF = Z_P_F + Roll_D
    DlR = Z_P_R + Roll_D

    foot_width = toe_link_state_FR.link_world_position_y - toe_link_state_FL.link_world_position_y
    #Foot_Roll_Offset = 0.5*(OG_foot_width - foot_width)
    Foot_Roll_Offset = 0.5*(OG_Body_width) - Roll_Body
    print("roll_tot_offset", Foot_Roll_Offset)
    hip_offset_z = (hip_link_state_FR.link_world_position_z - knee_link_state_FR.link_world_position_z)**2
    hip_offset_y = (hip_link_state_FR.link_world_position_y - knee_link_state_FR.link_world_position_y)**2
    hip_offset = math.sqrt(hip_offset_z + hip_offset_y)
    
    total_offset = hip_offset + Foot_Roll_Offset

    theta_roll_rF = math.atan(total_offset /DrF)
    theta_roll_rR = math.atan(total_offset /DrR)
    theta_roll_lF = math.atan(total_offset /DlF)
    theta_roll_lR = math.atan(total_offset /DlR)

    H_rF = DrF/math.cos(theta_roll_rF)
    H_rR = DrR/math.cos(theta_roll_rR)
    H_lF = DlF/math.cos(theta_roll_lF)
    H_lR = DlR/math.cos(theta_roll_lR)

    Z_rF = H_rF*math.cos(theta_roll_rF + ROLL_ANGLE)
    Z_rR = H_rR*math.cos(theta_roll_rR + ROLL_ANGLE)
    Z_lF = H_lF*math.cos(theta_roll_lF - ROLL_ANGLE)
    Z_lR = H_lR*math.cos( theta_roll_lR - ROLL_ANGLE)
    print(theta_roll_rF + ROLL_ANGLE,ROLL_ANGLE - theta_roll_lF)
    Y_rF = H_rF*math.sin(theta_roll_rF + ROLL_ANGLE) 
    Y_rR = H_rR*math.sin(theta_roll_rR + ROLL_ANGLE) 
    Y_lF = H_lF*math.sin(theta_roll_lF - ROLL_ANGLE ) 
    Y_lR = H_lR*math.sin(theta_roll_lR - ROLL_ANGLE ) 
    print(total_offset, hip_offset , Foot_Roll_Offset )
    print('total_offset_rF', 'hip_offset', 'Foot_Roll_Offset' )
    print(DrF, DlF, DrR, DlR)
    print('DrF', 'DlF', 'DrR', 'DlR')
    print(H_rF, H_lF, H_rR, H_lR)
    print('H_rF', 'H_lF', 'DrR', 'DlR')
   
    
   
    
#################################################################################################################################
    #Lean Calculations
    print("hip_offset",hip_offset, "Y_lF",Y_lF)
    theta_6_rF = math.atan(Y_rF /Z_rF)
    theta_6_rR = math.atan(Y_rR /Z_rR)
    theta_6_lF = math.atan(Y_lF/Z_lF)
    theta_6_lR = math.atan(Y_lR /Z_lR)
       
    H1_F = Z_rF/(math.cos(theta_6_rF + HIP_ANGLE))
    H2_F = Z_lF/(math.cos(theta_6_lF - HIP_ANGLE))
    H1_R = Z_rR/(math.cos(theta_6_rR + HIP_ANGLE))
    H2_R = Z_lR/(math.cos(theta_6_lR - HIP_ANGLE))
        
    #theta_7_rF = PI/2 - theta_6_rF
    #theta_7_rR = PI/2 - theta_6_rR
    #theta_7_lF = PI/2 - theta_6_lF
    #theta_7_lR = PI/2 - theta_6_lR
        
    #Z_A_F = math.sin(theta_7_rF)*H1_F
    #Z_B_F = math.sin(theta_7_lF)*H2_F
    #Z_A_R = math.sin(theta_7_rR)*H1_R
    #Z_B_R = math.sin(theta_7_lR)*H2_R

    theta_7_rF = math.asin(hip_offset/H1_F)
    theta_7_rR = math.asin(hip_offset/H1_R)
    theta_7_lF = math.asin(hip_offset/H2_F)
    theta_7_lR = math.asin(hip_offset/H2_R)

    Z_A_F = hip_offset/math.tan( theta_7_rF)
    Z_B_F = hip_offset/math.tan( theta_7_lF)
    Z_A_R = hip_offset/math.tan( theta_7_rR)
    Z_B_R = hip_offset/math.tan( theta_7_lR)

    print("ZA_F",Z_A_F)
    print("ZB_F",Z_B_F)
    print("ZA_r",Z_A_R)
    print("ZB_r",Z_A_R)
###########################################################################################################################################
    #Finding the extended leg distance Z, based on step length
    theta_3A_F = math.atan(STEP_LENGTH_F/Z_A_F)
    theta_3B_F = math.atan(STEP_LENGTH_F/Z_B_F)
    theta_3A_R = math.atan(STEP_LENGTH_R/Z_A_R)
    theta_3B_R = math.atan(STEP_LENGTH_R/Z_B_R)

    Zaf = Z_A_F/ math.cos(theta_3A_F)
    Zbf = Z_B_F/ math.cos(theta_3B_F)
    Zar = Z_A_R/ math.cos(theta_3A_R)
    Zbr = Z_B_R/ math.cos(theta_3B_R)

##########################################################################################################################################
    #Calculating Joint Positions
    theta_1A_F = math.acos((L1**2 + Zaf**2 - L2**2)/(2*Zaf*L1))
    theta_2A_F = math.acos((L1**2 + L2**2 - Zaf**2)/(2*L1*L2))
    theta_1A_R = math.acos((L1**2 + Zar**2 - L2**2)/(2*Zar*L1))
    theta_2A_R = math.acos((L1**2 + L2**2 - Zar**2)/(2*L1*L2))

    theta_1B_F = math.acos((L1**2 + Zbf**2 - L2**2)/(2*Zbf*L1))
    theta_2B_F = math.acos((L1**2 + L2**2 - Zbf**2)/(2*L1*L2))
    theta_1B_R = math.acos((L1**2 + Zbr**2 - L2**2)/(2*Zbr*L1))
    theta_2B_R = math.acos((L1**2 + L2**2 - Zbr**2)/(2*L1*L2))
###########################################################################################################################################

    theta_kneeA_F =  -(theta_1A_F) + theta_3A_F
    theta_footA_F = PI - theta_2A_F
    theta_kneeA_R =  -(theta_1A_R) + theta_3A_R
    theta_footA_R = PI - theta_2A_R

    theta_kneeB_F =  -(theta_1B_F) + theta_3B_F
    theta_footB_F = PI - theta_2B_F
    theta_kneeB_R =  -(theta_1B_R) + theta_3B_R
    theta_footB_R = PI - theta_2B_R

    if (state == 1):
        #setting sit down position
        
        target_position_hip = HIP_ANGLE + ROLL_ANGLE
        #target_position_hip = 0
        pybullet.setJointMotorControl2(robot, HIP_LINK_INDEX_FR, pybullet.POSITION_CONTROL, targetPosition=target_position_hip)
        pybullet.setJointMotorControl2(robot, HIP_LINK_INDEX_FL, pybullet.POSITION_CONTROL, targetPosition=target_position_hip)
        pybullet.setJointMotorControl2(robot, HIP_LINK_INDEX_RR, pybullet.POSITION_CONTROL, targetPosition=-(target_position_hip))
        pybullet.setJointMotorControl2(robot, HIP_LINK_INDEX_RL, pybullet.POSITION_CONTROL, targetPosition=-(target_position_hip))

        pybullet.setJointMotorControl2(robot, KNEE_LINK_INDEX_FR, pybullet.POSITION_CONTROL, targetPosition=theta_kneeA_F)
        pybullet.setJointMotorControl2(robot, FOOT_LINK_INDEX_FR, pybullet.POSITION_CONTROL, targetPosition=theta_footA_F)
        pybullet.setJointMotorControl2(robot, KNEE_LINK_INDEX_FL, pybullet.POSITION_CONTROL, targetPosition=-(theta_kneeB_F))
        pybullet.setJointMotorControl2(robot, FOOT_LINK_INDEX_FL, pybullet.POSITION_CONTROL, targetPosition= theta_footB_F)
        pybullet.setJointMotorControl2(robot, KNEE_LINK_INDEX_RR, pybullet.POSITION_CONTROL, targetPosition=theta_kneeA_R)
        pybullet.setJointMotorControl2(robot, FOOT_LINK_INDEX_RR, pybullet.POSITION_CONTROL, targetPosition=theta_footA_R)                              
        pybullet.setJointMotorControl2(robot, KNEE_LINK_INDEX_RL, pybullet.POSITION_CONTROL, targetPosition=-(theta_kneeB_R))
        pybullet.setJointMotorControl2(robot, FOOT_LINK_INDEX_RL, pybullet.POSITION_CONTROL, targetPosition= theta_footB_R)

        pybullet.stepSimulation()
        
    time.sleep(0.01)

    
    
    





