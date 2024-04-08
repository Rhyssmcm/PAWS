import pybullet
import pybullet_data
import time
import math
import numpy as np
import matplotlib.pyplot as plt

from pybullet_utils import LinkState
from pybullet_utils import JointState
pybullet.connect(pybullet.GUI)
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setGravity(0, 0, -9.81)

startPos = [0,0,0.35]
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

#hip_angles_SLR = np.array ([0.1537])
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
state = -1
L1 = 0.15
L2 = 0.20
D = 0.25
PI = math.pi
STEP_LENGTH = 0.1
HIP_ANGLE = 0.0015
PITCH_ANGLE = 0
stepsize=0.01


def semicircle_points(step_length, num_points):

    radius = 0.5*step_length
    theta = np.linspace(0, np.pi, num_points)
    x = radius* np.cos(theta) 
    z = radius * np.sin(theta) - 0.25
   
    return x, z


hip_link_state_FR = LinkState(pybullet.getLinkState(robot, HIP_LINK_INDEX_FR))
knee_link_state_FR = LinkState(pybullet.getLinkState(robot, KNEE_LINK_INDEX_FR))
foot_link_state_FR = LinkState(pybullet.getLinkState(robot, FOOT_LINK_INDEX_FR))
toe_link_state_FR = LinkState(pybullet.getLinkState(robot, TOE_LINK_INDEX_FR))

hip_link_state_RR = LinkState(pybullet.getLinkState(robot, HIP_LINK_INDEX_RR))
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

# Set step_length and step_height
step_length = 0.10
step_height = 0.03
num_points  = 400

x_arc, z_arc = semicircle_points(step_length, step_height, num_points)
x_stance = x_arc[::-1]  
z_stance = -0.25*np.ones(400)
x = np.concatenate((x_arc, x_stance))
z = np.concatenate((z_arc, z_stance))


data = np.column_stack((x, z))
data = data
np.savetxt('quadruped_semi_circle_arc_coordinates.csv', data, delimiter=',', header='X,Z', comments='')


#Plot the curve
plt.plot(x,z)
plt.title('Semicircle')
plt.xlabel('X-axis')
plt.ylabel('z-axis')
plt.axis('equal')  # Equal scaling to maintain circular appearance
plt.grid(True)
plt.show()