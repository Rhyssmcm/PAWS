import pybullet
import time
import math
class Leg:
    """
    This class inputs joint angles based on the input Swing and Stance co-ordinates that sets each motor joint position
    TODO: Answer above questions here
    """
    def __init__(self, I, roll,hip_angle, knee_angles_F_SWING, foot_angles_F_SWING, knee_angles_R_SWING, foot_angles_R_SWING ,knee_angles_F_STANCE, foot_angles_F_STANCE ,knee_angles_R_STANCE, foot_angles_R_STANCE, SW_F_HIP_link_id, SW_R_HIP_link_id, ST_F_HIP_link_id,ST_R_HIP_link_id,
                  SW_F_knee_link_id, SW_F_foot_link_id, SW_R_knee_link_id, SW_R_foot_link_id, ST_F_knee_link_id, ST_F_foot_link_id, ST_R_knee_link_id, ST_R_foot_link_id, FR_Stance,robot):
        """      
        Initialises the leg object
        """
        self.SW_F_HIP_link_id = SW_F_HIP_link_id
        self.SW_R_HIP_link_id = SW_R_HIP_link_id
        self.ST_F_HIP_link_id = ST_F_HIP_link_id
        self.ST_R_HIP_link_id = ST_R_HIP_link_id
        self.knee_angles_F_SWING  = knee_angles_F_SWING
        self.foot_angles_F_SWING = foot_angles_F_SWING
        self.knee_angles_R_SWING = knee_angles_R_SWING
        self.foot_angles_R_SWING = foot_angles_R_SWING
        
        self.knee_angles_F_STANCE  = knee_angles_F_STANCE
        self.foot_angles_F_STANCE = foot_angles_F_STANCE
        self.knee_angles_R_STANCE= knee_angles_R_STANCE
        self.foot_angles_R_STANCE = foot_angles_R_STANCE
        
        self.SW_F_knee_link_id = SW_F_knee_link_id
        self.SW_F_foot_link_id = SW_F_foot_link_id
        self.SW_R_knee_link_id = SW_R_knee_link_id
        self.SW_R_foot_link_id = SW_R_foot_link_id

        self.ST_F_knee_link_id = ST_F_knee_link_id
        self.ST_F_foot_link_id = ST_F_foot_link_id
        self.ST_R_knee_link_id = ST_R_knee_link_id
        self.ST_R_foot_link_id = ST_R_foot_link_id
        self.FR_Stance = FR_Stance
        self.roll = roll
        self.hip_angle = hip_angle
        self.robot = robot
        self.I = I

        self.current_angles = [self.hip_angle, self.knee_angles_F_SWING, self.foot_angles_F_SWING, self.knee_angles_R_SWING , self.foot_angles_R_SWING,
                                self.foot_angles_F_STANCE, self.foot_angles_F_STANCE , self.knee_angles_R_STANCE  , self.foot_angles_R_STANCE ]
        
        
        
    
    def record_velocity(self, start_time, previous_velocity_time, previous_angles):
    # Get the current simulation time
       

        velocity_list = []
        current_velocity_time = time.time() - start_time
       
    
        for i in range (0,7):

            angle_diff = self.current_angles[i] - previous_angles[i]
            time_diff = current_velocity_time - previous_velocity_time

            velocity = abs(angle_diff/time_diff)
            velocity_list.append(velocity)

        previous_angles = self.current_angles
        previous_velocity_time = current_velocity_time 

        return velocity_list ,previous_angles,current_velocity_time

    def move_motors(self):
        
        

        target_position_hip = self.roll + self.hip_angle
        
        pybullet.setJointMotorControl2(self.robot, 0, pybullet.POSITION_CONTROL, targetPosition= target_position_hip)
        pybullet.setJointMotorControl2(self.robot, 4, pybullet.POSITION_CONTROL, targetPosition= target_position_hip)
        pybullet.setJointMotorControl2(self.robot, 8, pybullet.POSITION_CONTROL, targetPosition=  -target_position_hip)
        pybullet.setJointMotorControl2(self.robot, 12, pybullet.POSITION_CONTROL, targetPosition= -target_position_hip)

        #settting swing pair
        pybullet.setJointMotorControl2(self.robot, self.SW_F_knee_link_id, pybullet.POSITION_CONTROL, targetPosition=self.knee_angles_F_SWING)
        pybullet.setJointMotorControl2(self.robot, self.SW_F_foot_link_id, pybullet.POSITION_CONTROL, targetPosition=self.foot_angles_F_SWING)
        pybullet.setJointMotorControl2(self.robot, self.SW_R_knee_link_id, pybullet.POSITION_CONTROL, targetPosition=self.knee_angles_R_SWING)
        pybullet.setJointMotorControl2(self.robot, self.SW_R_foot_link_id, pybullet.POSITION_CONTROL, targetPosition=self.foot_angles_R_SWING)
        #settting stance pair
        pybullet.setJointMotorControl2(self.robot, self.ST_F_knee_link_id, pybullet.POSITION_CONTROL, targetPosition=self.knee_angles_F_STANCE)
        pybullet.setJointMotorControl2(self.robot, self.ST_F_foot_link_id, pybullet.POSITION_CONTROL, targetPosition=self.foot_angles_F_STANCE)
        pybullet.setJointMotorControl2(self.robot, self.ST_R_knee_link_id, pybullet.POSITION_CONTROL, targetPosition=self.knee_angles_R_STANCE)
        pybullet.setJointMotorControl2(self.robot, self.ST_R_foot_link_id, pybullet.POSITION_CONTROL, targetPosition=self.foot_angles_R_STANCE)
        
        #previous_velocities = velocity_list

        return self.FR_Stance, self.SW_F_knee_link_id, self.SW_F_foot_link_id, self.SW_R_knee_link_id, self.SW_R_foot_link_id, self.ST_F_knee_link_id, self.ST_F_foot_link_id, self.ST_R_knee_link_id, self.ST_R_foot_link_id,