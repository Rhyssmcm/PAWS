class LinkState:
    """
    Stores the data from a given PyBullet link.
    All state data accessible through parameters.
    See getLinkState(s) PyBullet Quickstart Guide
    """
    def __init__(self, link_state_tuple):
        """
        Create a LinkState object.
        :param link_state_tuple: A link tuple created by pybullet.getLinkState()
        """
        self.link_world_position = link_state_tuple[0]
        """Cartesian position of center of mass as a tuple of length 3 in form (x, y, z)"""

        self.link_world_orientation = link_state_tuple[1]
        self.local_intertial_frame_position = link_state_tuple[2]
        self.local_inertial_frame_orientation = link_state_tuple[3]
        self.world_link_frame_position = link_state_tuple[4]
        self.world_link_frame_orientation = link_state_tuple[5]
       # self.world_link_linear_velocity = link_state_tuple[6]
       # self.world_link_angular_velocity = link_state_tuple[7]

        self.link_world_position_x = self.link_world_position[0]
        """x position as a float"""

        self.link_world_position_y = self.link_world_position[1]
        """y position as a float"""

        self.link_world_position_z = self.link_world_position[2]
        """z position as a float"""
        self.local_intertial_frame_position_x = self.local_intertial_frame_position[0]
        self.local_intertial_frame_position_y = self.local_intertial_frame_position[1]
        self.local_intertial_frame_position_z = self.local_intertial_frame_position[2]

class JointState:
    """
    Stores the data from a given PyBullet joint.
    All state data accessible through parameters.
    See getJointState(s) PyBullet Quickstart Guide
    """
    def __init__(self, joint_state_tuple):
        """
        Create a JointState object.
        :param joint_state_tuple: A link tuple created by pybullet.getJointState()
        """
        self.joint_position = joint_state_tuple[0]
        """Position in rads"""

        self.joint_velocity = joint_state_tuple[1]
        """Joint velocity as a float (????)"""

        self.joint_reaction_forces = joint_state_tuple[2]
        self.applied_joint_motor_torque = joint_state_tuple[3]
