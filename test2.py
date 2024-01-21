import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

# Define the robot structure
arm_chain = Chain(name='leg', links=[
    OriginLink(),
    URDFLink(
      name="coxa",
      origin_translation=[0, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 1],
      bounds=(-np.pi, np.pi)
    ),
    URDFLink(
      name="femur",
      origin_translation=[0, 0, 55],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(-np.pi/2, np.pi/2)
    ),
    URDFLink(
      name="tibia",
      origin_translation=[0, 0, 100],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(-np.pi/2, np.pi/2)
    ),
    # Add an end effector with a negligible rotation to avoid errors
    URDFLink(
      name="end_effector",
      origin_translation=[0, 0, 125],  # Adjust this based on the actual length beyond the tibia joint
      origin_orientation=[0, 0, 0],
      rotation=[1e-5, 0, 0]  # Minimal rotation to avoid the ValueError
    )
], active_links_mask=[False, True, True, True, True])  # Exclude the OriginLink from active links

# Define the target position
target_position = [0, 0, -100]

# Calculate the inverse kinematics
angles = arm_chain.inverse_kinematics(target_position)

# angles will contain the angles for each joint
print("Computed angles for the joints are : ", angles[1:])  # Exclude the first angle corresponding to the OriginLink
