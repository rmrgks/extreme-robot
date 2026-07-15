#!/usr/bin/env python3
"""그리퍼 모듈별 설정 (moveit_dynamixel_bridge / arm_fsm_node 공용).

robot_arm_description의 urdf/grippers/<gripper>.xacro 모듈화(xacro:arg gripper)에
대응하는 코드 레벨 preset. 새 그리퍼 모듈을 추가할 때는 두 노드를 건드리지 않고
이 dict에 항목만 추가하면 됨.
"""

GRIPPER_PRESETS = {
    "gripper_a": {
        # 2026-07-15 Isaac Sim 재export(robotarm_urdf_20260711.urdf) 기준 — 그리퍼가
        # gripper_drive_joint 하나만 실제 구동되고 나머지 8개(크랭크/조 관절)는 URDF의
        # <mimic> 태그로 자동 종속(평행 4절링크 구속을 URDF 레벨에서 정식 표현).
        # 기존 gripper_a_joint5/6(두 관절을 동일 값으로 미러링하던 방식)에서 갈아탐.
        "gripper_joints": ["gripper_drive_joint"],
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
