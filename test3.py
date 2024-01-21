import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

# Define the robot arm's joint parameters (lengths and angles in radians)
joint_parameters = [
    [0.0, 0.0, 0.0],  # Joint 1 (Base)
    [0.0, 0.0, 1.0],  # Joint 2 (Link 1)
    [0.0, 0.0, 1.0],  # Joint 3 (Link 2)
]

# Create the robot arm chain
arm_chain = Chain(name="my_arm_chain", links=[
    OriginLink(),  # The base link
    URDFLink(
        name="joint1",
        origin_translation=np.array([0, 0, 0]),
        origin_orientation=np.array([0, 0, 0]),
        rotation=np.array([0, 1, 0]),
        translation=np.array([0, 0, 0]),
        bounds=(-np.pi, np.pi),
        joint_type='revolute',  # Specify joint type
    ),
    URDFLink(
        name="joint2",
        origin_translation=np.array([0, 0, 1.0]),
        origin_orientation=np.array([0, 0, 0]),
        rotation=np.array([0, 1, 0]),
        translation=np.array([0, 0, 0]),
        bounds=(-np.pi, np.pi),
        joint_type='revolute',  # Specify joint type
    ),
    URDFLink(
        name="joint3",
        origin_translation=np.array([0, 0, 1.0]),
        origin_orientation=np.array([0, 0, 0]),
        rotation=np.array([0, 1, 0]),
        translation=np.array([0, 0, 0]),
        bounds=(-np.pi, np.pi),
        joint_type='revolute',  # Specify joint type
    ),
])

# Set the joint parameters for the robot arm
arm_chain.links[1].parameters = joint_parameters[0]
arm_chain.links[2].parameters = joint_parameters[1]
arm_chain.links[3].parameters = joint_parameters[2]

# Calculate the forward kinematics
end_effector_position = arm_chain.forward_kinematics()

print("End-effector position:")
print(end_effector_position)
