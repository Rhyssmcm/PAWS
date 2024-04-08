import pybullet
import pybullet_data
import time
import math
import numpy as np
import matplotlib.pyplot as plt

from pybullet_utils import LinkState
from pybullet_utils import JointState

class Generate_angle :
    """
    What is this class? Why does it exist?
    TODO: Answer above questions here
    """

    def __init__(self, arc_x, arc_z, OG_foot_length, OG_Body_width, OG_Body_length, robot):
        """      
        Takes the x and z coordinates & original body length/width to calculate changes to PAWS leg lengths based on rotation & end_effector trajectory
        """
        self.arc_x = arc_x
        self.arc_z = arc_z
        self.knee_angles_SWING  = 0
        self.knee_angles_STANCE = 0
        self.foot_angles_SWING = 0
        self.foot_angles_STANCE = 0
       
        self.target_position_knee = 0
        self.target_position_foot = 0

        self.OG_foot_length = OG_foot_length
   
        self.points = 0
        self.leg_state = 0
        self.cycle_counter = 0
        self.cycle_finished = False
        self.robot = robot

        self.OG_Body_width = OG_Body_width
        self.OG_Body_lenth = OG_Body_length
        


    def generate_angle (self):
    
        from pybullet_utils import LinkState
        from pybullet_utils import JointState

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

        hip_link_state_FR = LinkState(pybullet.getLinkState(self.robot, HIP_LINK_INDEX_FR))
        knee_link_state_FR = LinkState(pybullet.getLinkState(self.robot, KNEE_LINK_INDEX_FR))
        foot_link_state_FR = LinkState(pybullet.getLinkState(self.robot, FOOT_LINK_INDEX_FR))
        toe_link_state_FR = LinkState(pybullet.getLinkState(self.robot, TOE_LINK_INDEX_FR))

        hip_link_state_RR = LinkState(pybullet.getLinkState(self.robot, HIP_LINK_INDEX_RR))
        knee_link_state_RR = LinkState(pybullet.getLinkState(self.robot, KNEE_LINK_INDEX_RR))
        foot_link_state_RR = LinkState(pybullet.getLinkState(self.robot, FOOT_LINK_INDEX_RR))
        toe_link_state_RR = LinkState(pybullet.getLinkState(self.robot, TOE_LINK_INDEX_RR))

        hip_joint_state_FR = JointState(pybullet.getJointState(self.robot, HIP_JOINT_INDEX_FR))
        knee_joint_state_FR = JointState(pybullet.getJointState(self.robot, KNEE_JOINT_INDEX_FR))
        foot_joint_state_FR = JointState(pybullet.getJointState(self.robot, FOOT_JOINT_INDEX_FR))
        toe_joint_state_FR = JointState(pybullet.getJointState(self.robot, TOE_JOINT_INDEX_FR))
        
        hip_joint_state_RR = JointState(pybullet.getJointState(self.robot, HIP_JOINT_INDEX_RR))
        knee_joint_state_RR = JointState(pybullet.getJointState(self.robot, KNEE_JOINT_INDEX_RR))
        foot_joint_state_RR = JointState(pybullet.getJointState(self.robot, FOOT_JOINT_INDEX_RR))
        toe_joint_state_RR = JointState(pybullet.getJointState(self.robot, TOE_JOINT_INDEX_RR)) 

        hip_link_state_FL = LinkState(pybullet.getLinkState(self.robot, HIP_LINK_INDEX_FL))
        knee_link_state_FL = LinkState(pybullet.getLinkState(self.robot, KNEE_LINK_INDEX_FL))
        foot_link_state_FL = LinkState(pybullet.getLinkState(self.robot, FOOT_LINK_INDEX_FL))
        toe_link_state_FL = LinkState(pybullet.getLinkState(self.robot, TOE_LINK_INDEX_FL))

        hip_link_state_RL = LinkState(pybullet.getLinkState(self.robot, HIP_LINK_INDEX_RL))
        knee_link_state_RL = LinkState(pybullet.getLinkState(self.robot, KNEE_LINK_INDEX_RL))
        foot_link_state_RL = LinkState(pybullet.getLinkState(self.robot, FOOT_LINK_INDEX_RL))
        toe_link_state_RL = LinkState(pybullet.getLinkState(self.robot, TOE_LINK_INDEX_RL))

        
       
        L1 = 0.15
        L2 = 0.20
        PI = math.pi
        D = - self.arc_z
        STEP_LENGTH = self.arc_x

        knee_distance_z = (knee_link_state_FR.link_world_position_z - knee_link_state_RR.link_world_position_z)**2
        knee_distance_x = (knee_link_state_FR.link_world_position_x - knee_link_state_RR.link_world_position_x)**2
        Body_length = math.sqrt(knee_distance_x + knee_distance_z)

        foot_length = toe_link_state_FR.link_world_position_x - toe_link_state_RR.link_world_position_x
        base_pos, base_orientation = pybullet.getBasePositionAndOrientation(self.robot)
        base_euler = pybullet.getEulerFromQuaternion(base_orientation)
        pitch_Angle = base_euler[1]  
        roll_Angle =  -base_euler[0]  

        D_Pitch = 0.5*(Body_length)*math.sin(pitch_Angle)
        new_Body_length =  0.5*(Body_length)*math.cos(pitch_Angle)
        
        D_Front = D + D_Pitch
        D_Rear =  D - D_Pitch

        foot_length = toe_link_state_FR.link_world_position_x - toe_link_state_RR.link_world_position_x
        Foot_Offset = 0.5*(self.OG_foot_length) - new_Body_length
        
        Theta_pitch_offset_F = math.atan(Foot_Offset/D_Front)
        Theta_pitch_offset_R = math.atan(Foot_Offset/D_Rear)
        
        Shoulder_Angle_F = pitch_Angle - Theta_pitch_offset_F
        Shoulder_Angle_R  = pitch_Angle + Theta_pitch_offset_R

        H_Front = D_Front/math.cos(Theta_pitch_offset_F)
        H_Rear  = D_Rear/math.cos(Theta_pitch_offset_R)

        Z_P_F = H_Front*math.cos(Shoulder_Angle_F)
        Z_P_R = H_Rear*math.cos(Shoulder_Angle_R)

        X_P_F = H_Front*math.sin(Shoulder_Angle_F)
        X_P_R = H_Rear*math.sin(Shoulder_Angle_R)
       
        STEP_LENGTH_F = STEP_LENGTH - X_P_F
        STEP_LENGTH_R = STEP_LENGTH - X_P_R
      

        hip_distance_z = (hip_link_state_FR.link_world_position_z - hip_link_state_RR.link_world_position_z)**2
        hip_distance_x = (hip_link_state_FR.link_world_position_x - hip_link_state_RR.link_world_position_x)**2
        hip_distance_y = (hip_link_state_FR.link_world_position_y - hip_link_state_RR.link_world_position_y)**2
        Body_length = math.sqrt(hip_distance_x + hip_distance_z)
        Body_width = math.sqrt(hip_distance_y + hip_distance_z)
        
        ############################################################################################################################
         #Roll Calculations
       
        Roll_D = math.sin(roll_Angle)*0.5*Body_width
        Roll_Body = math.cos(roll_Angle)*0.5*Body_width

        DrF = Z_P_F - Roll_D
        DrR = Z_P_R - Roll_D
        DlF = Z_P_F + Roll_D
        DlR = Z_P_R + Roll_D


        Foot_Roll_Offset = 0.5*(self.OG_Body_width) - Roll_Body
       
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

        Z_rF = H_rF*math.cos(theta_roll_rF + roll_Angle)
        Z_rR = H_rR*math.cos(theta_roll_rR + roll_Angle)
        Z_lF = H_lF*math.cos(theta_roll_lF - roll_Angle)
        Z_lR = H_lR*math.cos( theta_roll_lR - roll_Angle)
      
        Y_rF = H_rF*math.sin(theta_roll_rF + roll_Angle) 
        Y_rR = H_rR*math.sin(theta_roll_rR + roll_Angle) 
        Y_lF = H_lF*math.sin(theta_roll_lF - roll_Angle ) 
        Y_lR = H_lR*math.sin(theta_roll_lR - roll_Angle ) 
   
    #################################################################################################################################
        #Side to Side translation
        
        theta_6_rF = math.atan(Y_rF /Z_rF)
        theta_6_rR = math.atan(Y_rR /Z_rR)
        theta_6_lF = math.atan(Y_lF/Z_lF)
        theta_6_lR = math.atan(Y_lR /Z_lR)

        lean_angle = -roll_Angle
        H1_F = Z_rF/(math.cos(theta_6_rF + lean_angle))
        H2_F = Z_lF/(math.cos(theta_6_lF - lean_angle))
        H1_R = Z_rR/(math.cos(theta_6_rR + lean_angle))
        H2_R = Z_lR/(math.cos(theta_6_lR - lean_angle))
            

        theta_7_rF = math.asin(hip_offset/H1_F)
        theta_7_rR = math.asin(hip_offset/H1_R)
        theta_7_lF = math.asin(hip_offset/H2_F)
        theta_7_lR = math.asin(hip_offset/H2_R)

        Z_A_F = hip_offset/math.tan( theta_7_rF)
        Z_B_F = hip_offset/math.tan( theta_7_lF)
        Z_A_R = hip_offset/math.tan( theta_7_rR)
        Z_B_R = hip_offset/math.tan( theta_7_lR)

    ###########################################################################################################################################
     #Extended Step Length translation
        theta_3A_F = math.atan(STEP_LENGTH_F/Z_A_F)
        theta_3B_F = math.atan(STEP_LENGTH_F/Z_B_F)
        theta_3A_R = math.atan(STEP_LENGTH_R/Z_A_R)
        theta_3B_R = math.atan(STEP_LENGTH_R/Z_B_R)

       
        Zaf = Z_A_F/ math.cos(theta_3A_F)
        Zbf = Z_B_F/ math.cos(theta_3B_F)
        Zar = Z_A_R/ math.cos(theta_3A_R)
        Zbr = Z_B_R/ math.cos(theta_3B_R)

    ##########################################################################################################################################
    #Finding Angles Based on leg length
        theta_1A_F = math.acos((L1**2 + Zaf**2 - L2**2)/(2*Zaf*L1))
        theta_2A_F = math.acos((L1**2 + L2**2 - Zaf**2)/(2*L1*L2))
        theta_1A_R = math.acos((L1**2 + Zar**2 - L2**2)/(2*Zar*L1))
        theta_2A_R = math.acos((L1**2 + L2**2 - Zar**2)/(2*L1*L2))

        theta_1B_F = math.acos((L1**2 + Zbf**2 - L2**2)/(2*Zbf*L1))
        theta_2B_F = math.acos((L1**2 + L2**2 - Zbf**2)/(2*L1*L2))
        theta_1B_R = math.acos((L1**2 + Zbr**2 - L2**2)/(2*Zbr*L1))
        theta_2B_R = math.acos((L1**2 + L2**2 - Zbr**2)/(2*L1*L2))

    ###########################################################################################################################################
    #Joint Position Calcualtion for knee and foot joints including offsets 
        theta_kneeA_F =  -(theta_1A_F) + theta_3A_F
        theta_footA_F = PI - theta_2A_F
        theta_kneeA_R =  -(theta_1A_R) + theta_3A_R
        theta_footA_R = PI - theta_2A_R

        theta_kneeB_F =  -(-(theta_1B_F) + theta_3B_F)
        theta_footB_F = PI - theta_2B_F
        theta_kneeB_R =  -(-(theta_1B_R) + theta_3B_R)
        theta_footB_R = PI - theta_2B_R

        return theta_kneeA_F,theta_footA_F,theta_kneeA_R,theta_footA_R,theta_kneeB_F,theta_footB_F,theta_kneeB_R, theta_footB_R, roll_Angle
            
            


    