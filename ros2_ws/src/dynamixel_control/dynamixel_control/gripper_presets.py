#!/usr/bin/env python3
"""그리퍼 모듈별 설정 (moveit_dynamixel_bridge / arm_fsm_node 공용).

robot_arm_description의 urdf/grippers/<gripper>.xacro 모듈화(xacro:arg gripper)에
대응하는 코드 레벨 preset. 새 그리퍼 모듈을 추가할 때는 두 노드를 건드리지 않고
이 dict에 항목만 추가하면 됨.
"""

GRIPPER_PRESETS = {
    "gripper_a": {
        "gripper_joints": ["gripper_a_joint5", "gripper_a_joint6"],
        "gripper_ids": [5],
        "gripper_open_tick": 2446,    # HW-8 실측 (215도, 2026-07-08)
        "gripper_close_tick": 3186,   # HW-8 실측 (280도, 2026-07-08)
        "gripper_open_m": 0.02,       # 선형보간 기준점 placeholder — 실측 캘리브 필요
        "gripper_close_m": 0.0,
        "grasp_effort_thresh": 80.0,  # ≈215mA, placeholder
        "drop_effort_thresh": 20.0,   # ≈54mA, placeholder
        "gripper_action_time": 1.0,
    },
}

DEFAULT_GRIPPER = "gripper_a"


def get_preset(gripper_type, logger=None):
    preset = GRIPPER_PRESETS.get(gripper_type)
    if preset is None:
        if logger is not None:
            logger.warn(
                f"Unknown gripper_type '{gripper_type}', falling back to '{DEFAULT_GRIPPER}'"
            )
        preset = GRIPPER_PRESETS[DEFAULT_GRIPPER]
    return preset
