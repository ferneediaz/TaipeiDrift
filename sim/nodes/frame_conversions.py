"""Frame conversions fixed by the Gazebo SDF and ROS optical-camera convention."""
import numpy as np


def gazebo_optical_to_flu():
    """Return R_bc mapping optical (right, down, forward) vectors into body FLU.

    The camera sensor pose in midair_quad/model.sdf is identity in the Gazebo
    sensor-link axes (+X forward, +Y left, +Z up). ROS optical axes are
    (+X right, +Y down, +Z forward), hence camera basis maps to body as:
    camera +X -> body -Y, camera +Y -> body -Z, camera +Z -> body +X.
    """
    return np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def gazebo_down_optical_to_flu():
    """R for the downward camera's SDF pitch of +90 degrees."""
    return np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
