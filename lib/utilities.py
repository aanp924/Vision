import math

import cv2
import numpy as np

cameraMatrix = np.asarray(
    [
        [887.68675656, 0.0, 800.62430596],
        [0.0, 887.66067058, 601.73557764],
        [0.0, 0.0, 1.0],
    ]
)

distCoeffs = np.asarray(
    [
        [
            -0.12984718,
            0.10905513,
            0.00036732,
            -0.00033288,
            0.00078861,
            0.17031277,
            0.05104419,
            0.02662509,
        ]
    ]
)


def euler_from_quaternion_xyz(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def euler_from_quaternion_yxz(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around y in radians (counterclockwise)
    pitch is rotation around x in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = -2.0 * (x * z - y * w)
    t1 = +2.0 * (w * w + z * z) - 1.0
    roll_y = math.atan2(t0, t1)

    t2 = +2.0 * (x * w + y * z)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_x = math.asin(t2)

    t3 = -2.0 * (x * y - z * w)
    t4 = +2.0 * (y * y + w * w) - 1.0
    yaw_z = math.atan2(t3, t4)

    return roll_y, pitch_x, yaw_z  # in radians


def R_from_angles_body(angles, convention="xyz"):
    axes = {
        "x": np.asarray([1.0, 0.0, 0.0]),
        "y": np.asarray([0.0, 1.0, 0.0]),
        "z": np.asarray([0.0, 0.0, 1.0]),
    }

    r3, r2, r1 = convention
    R1 = cv2.Rodrigues(axes[r1] * angles[2])[0]
    ax2_prime = R1 @ axes[r2]
    R2 = cv2.Rodrigues(ax2_prime * angles[1])[0]
    ax3_prime = R2 @ R1 @ axes[r3]
    R3 = cv2.Rodrigues(ax3_prime * angles[0])[0]

    return R3 @ R2 @ R1


"""
The transform_imu_to_world function below demonstrates how to use these quaternions to transform
data from the IMU's local coordinate system to the world coordinate system.
"""

from scipy.spatial.transform import Rotation as R

def transform_imu_to_world(imu_coordinates, imu_quaternions):
    # This array contains a timeseries of transformation matrices,
    # as calculated from the IMU's timeseries of quaternions values.
    imu_to_world_matrices = R.from_quat(
        imu_quaternions,
        scalar_first=True,
    ).as_matrix()

    if np.ndim(imu_coordinates) == 1:
        return imu_to_world_matrices @ imu_coordinates
    else:
        return np.array([
            imu_to_world @ imu_coord
            for imu_to_world, imu_coord in zip(
                imu_to_world_matrices, imu_coordinates
            )
        ])
    
"""
The transform_imu_to_world function can be used to calculate heading vectors of the IMU in world 
coordinates. The heading vector essentially describes the direction the IMU is facing.
If we imagine the IMU inside the Neon module while it is worn on sombody's head, 
the heading vector describes the direction the wearer's face is pointing.

The "forward-facing axis" is the y-axis, so we can calculate the heading vector
by transforming the (0, 1, 0) vector.

Neutral orientation (i.e. an identity rotation in the quaternion) of the IMU 
would correspond to a heading vector that points at magnetic North and that is
oriented perpendicular to the line of gravity.

"""

def imu_heading_in_world(imu_quaternions):
    heading_neutral_in_imu_coords = np.array([0.0, 1.0, 0.0])
    return transform_imu_to_world(
        heading_neutral_in_imu_coords, imu_quaternions
    )

def transform_scene_to_imu(coords_in_scene, translation_in_imu=np.array([0.0, -1.3, -6.62])):
    imu_scene_rotation_diff = np.deg2rad(-90 - 12)
    scene_to_imu = np.array(
        [
            [1.0, 0.0, 0.0],
            [
                0.0,
                np.cos(imu_scene_rotation_diff),
                -np.sin(imu_scene_rotation_diff),
            ],
            [
                0.0,
                np.sin(imu_scene_rotation_diff),
                np.cos(imu_scene_rotation_diff),
            ],
        ]
    )

    coords_in_imu = scene_to_imu @ coords_in_scene.T

    coords_in_imu[0, :] += translation_in_imu[0]
    coords_in_imu[1, :] += translation_in_imu[1]
    coords_in_imu[2, :] += translation_in_imu[2]

    return coords_in_imu.T


"""
Combining the transform_scene_to_imu function with the transform_imu_to_world 
function allows us to go all the way from the scene camera coordinate system to 
the world coordinate system.

"""

def transform_scene_to_world(coords_in_scene, imu_quaternions, translation_in_imu=np.array([0.0, -1.3, -6.62])):
    coords_in_imu = transform_scene_to_imu(coords_in_scene, translation_in_imu)
    return transform_imu_to_world(coords_in_imu, imu_quaternions)

