from typing import Dict
from dataclasses import dataclass, field

from lerobot.robots.config import RobotConfig
from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.motors import Motor, MotorNormMode


@RobotConfig.register_subclass("boostert1_aio_ros2")
@dataclass
class BoosterT1AioRos2RobotConfig(RobotConfig):
    use_degrees = True
    norm_mode_body = (
        MotorNormMode.DEGREES if use_degrees else MotorNormMode.RANGE_M100_100
    )

    # 按组件分组：{ comp_id: { joint_name: Motor, ... }, ... }
    leader_motors: Dict[str, Dict[str, Motor]] = field(
        default_factory=lambda norm_mode_body=norm_mode_body: {
            "leader_left_wrist": {
                "leader_left_wrist_x": Motor(1, "robot_motor", norm_mode_body),
                "leader_left_wrist_y": Motor(2, "robot_motor", norm_mode_body),
                "leader_left_wrist_z": Motor(3, "robot_motor", norm_mode_body),
                "leader_left_wrist_roll": Motor(4, "robot_motor", norm_mode_body),
                "leader_left_wrist_pitch": Motor(5, "robot_motor", norm_mode_body),
                "leader_left_wrist_yaw": Motor(6, "robot_motor", norm_mode_body),
            },
            "leader_right_wrist": {
                "leader_right_wrist_x": Motor(1, "robot_motor", norm_mode_body),
                "leader_right_wrist_y": Motor(2, "robot_motor", norm_mode_body),
                "leader_right_wrist_z": Motor(3, "robot_motor", norm_mode_body),
                "leader_right_wrist_roll": Motor(4, "robot_motor", norm_mode_body),
                "leader_right_wrist_pitch": Motor(5, "robot_motor", norm_mode_body),
                "leader_right_wrist_yaw": Motor(6, "robot_motor", norm_mode_body),
            },
            "leader_head": {
                "leader_head_x": Motor(1, "robot_motor", norm_mode_body),
                "leader_head_y": Motor(2, "robot_motor", norm_mode_body),
                "leader_head_z": Motor(3, "robot_motor", norm_mode_body),
                "leader_head_roll": Motor(4, "robot_motor", norm_mode_body),
                "leader_head_pitch": Motor(5, "robot_motor", norm_mode_body),
                "leader_head_yaw": Motor(6, "robot_motor", norm_mode_body),
            },
            "leader_left_gripper": {  
                "leader_left_gripper": Motor(1, "robot_motor", norm_mode_body),
            },
            "leader_right_gripper": {
                "leader_right_gripper": Motor(1, "robot_motor", norm_mode_body),
            },                  
        }
    )

    follower_motors: Dict[str, Dict[str, Motor]] = field(
        default_factory=lambda norm_mode_body=norm_mode_body: {
            "follower_joint_states": {
                "AAHead_yaw":            Motor(1,  "robot_motor", norm_mode_body),
                "Head_pitch":            Motor(2,  "robot_motor", norm_mode_body),

                "Left_Shoulder_Pitch":   Motor(3,  "robot_motor", norm_mode_body),
                "Left_Shoulder_Roll":    Motor(4,  "robot_motor", norm_mode_body),
                "Left_Elbow_Pitch":      Motor(5,  "robot_motor", norm_mode_body),
                "Left_Elbow_Yaw":        Motor(6,  "robot_motor", norm_mode_body),
                "Left_Wrist_Pitch":      Motor(7,  "robot_motor", norm_mode_body),
                "Left_Wrist_Yaw":        Motor(8,  "robot_motor", norm_mode_body),
                "Left_Hand_Roll":        Motor(9,  "robot_motor", norm_mode_body),

                "Right_Shoulder_Pitch":  Motor(10, "robot_motor", norm_mode_body),
                "Right_Shoulder_Roll":   Motor(11, "robot_motor", norm_mode_body),
                "Right_Elbow_Pitch":     Motor(12, "robot_motor", norm_mode_body),
                "Right_Elbow_Yaw":       Motor(13, "robot_motor", norm_mode_body),
                "Right_Wrist_Pitch":     Motor(14, "robot_motor", norm_mode_body),
                "Right_Wrist_Yaw":       Motor(15, "robot_motor", norm_mode_body),
                "Right_Hand_Roll":       Motor(16, "robot_motor", norm_mode_body),

                "Waist":                 Motor(17, "robot_motor", norm_mode_body),

                "Left_Hip_Pitch":        Motor(18, "robot_motor", norm_mode_body),
                "Left_Hip_Roll":         Motor(19, "robot_motor", norm_mode_body),
                "Left_Hip_Yaw":          Motor(20, "robot_motor", norm_mode_body),
                "Left_Knee_Pitch":       Motor(21, "robot_motor", norm_mode_body),
                "Left_Ankle_Pitch":      Motor(22, "robot_motor", norm_mode_body),
                "Left_Ankle_Roll":       Motor(23, "robot_motor", norm_mode_body),

                "Right_Hip_Pitch":       Motor(24, "robot_motor", norm_mode_body),
                "Right_Hip_Roll":        Motor(25, "robot_motor", norm_mode_body),
                "Right_Hip_Yaw":         Motor(26, "robot_motor", norm_mode_body),
                "Right_Knee_Pitch":      Motor(27, "robot_motor", norm_mode_body),
                "Right_Ankle_Pitch":     Motor(28, "robot_motor", norm_mode_body),
                "Right_Ankle_Roll":      Motor(29, "robot_motor", norm_mode_body),
            },
        }
    )

    cameras: Dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "image_top": OpenCVCameraConfig(index_or_path=0, fps=30, width=640, height=480),
        }
    )

    use_videos: bool = False

    microphones: Dict[str, int] = field(
        default_factory=lambda: {}
    )
