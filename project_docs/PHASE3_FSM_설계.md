# Phase 3 — 로봇팔 FSM 설계 (drawio `2026_FSM_국방.drawio` 기반)

> 작성: 2026-06-29 / 기준: `2026_FSM_국방.drawio`(5구간 미션 FSM) + `CLAUDE_Plan.md` §1·§2·§3 인터페이스
> 범위: **설계 문서만** (구현 전 단계). 코딩은 이 문서 합의 후 진행.

---

## 1. 핵심 발견 — 팔이 실제로 일하는 구간은 "구간2" 하나

drawio는 **전체 로봇 5구간 미션 FSM**(주행+인식+팔이 한 흐름에 섞임). 구간별 로봇팔 관여도:

| 구간 | 미션 | 팔 역할 |
|------|------|---------|
| 1 봄_정찰(IFF) | 연막 주행 → 군복 피아식별 → LED | ❌ 자세 락만 |
| **2 여름_구호물자운반** | 박스 파지 → 운반 → 하역 | ✅ **핵심 픽 작업 (ARM_GRASP_BOX)** |
| 3 가을_장애물식별 | 비전마커 5개 식별 | ❌ 자세 락만 |
| 4 겨울_경로확보 | 빙판/제설 주행 | ❌ 자세 락만 (제설 방식 미정 — 팔로 치울지 논의 필요) |
| 5 정찰동행_추종 | 선도봇 PID 추종 | ❌ 자세 락만 |

→ **Phase 3 로봇팔 구현의 90%는 구간2의 `ARM_GRASP_BOX` 서브루틴.** 나머지 구간은 팔 FSM 입장에서 "CARRY/LOCK 상태 유지"로 동일하게 처리.

> ⚠️ 구간4 제설: drawio 논의사항에 "치우며 갈지 / 밟고 갈지" 미정. 팔로 제설하면 두 번째 팔 작업이 생김 → **선결 결정 필요**(아래 §6-D).

---

## 2. 노드 책임 분리 (drawio는 전체 로봇 → 우리는 팔 노드만)

`CLAUDE_Plan.md`는 **팔 FSM 노드 / 파워트레인 노드**를 토픽으로 분리하는 구조. drawio의 한 흐름을 둘로 자른다:

```
[파워트레인 소유]                          [로봇팔 FSM 소유]
DETECT_BOX, ALIGN_BOX, 주행/지형/신호등 ──/arrival_status──▶ ARM_GRASP_BOX 전체
SAND/GRAVEL/WATER/ICE, FOLLOW_LEAD     ──/chassis_mode───▶ 자세 락 트리거
                                       ◀──/arm_status────  파지 완료/실패 보고
```

- **파워트레인 소유**: 박스 탐지(`DETECT_BOX`)·정렬(`ALIGN_BOX`)·모든 주행/지형/신호등/추종 상태.
  - 단, `DETECT_BOX`의 YOLO는 인식 노드(`robot_arm_perception`)가 단일 소스로 publish → 파워트레인은 `/detected_objects` 구독, 팔은 `/pick_target` 구독 (Phase 2에서 구현됨).
- **로봇팔 FSM 소유**: `ALIGN_BOX` 완료 신호를 받은 뒤의 파지 시퀀스 전체 + 운반 중 전류 감시 + DROP 재파지.

---

## 3. 토픽 인터페이스 (Phase 2에서 일부 확정)

| 토픽 | 타입 | 방향 | 팔 FSM | 상태 |
|------|------|------|--------|------|
| `/pick_target` | DetectedObject | 인식→팔 | 파지 타겟 pose (transient_local) | ✅ Phase 2 구현 |
| `/arrival_status` | ArrivalStatus | 파워트레인→팔 | `ARRIVED_PICKUP`/`ARRIVED_DROP`로 FSM 트리거 | ⚠️ enum 미합의 |
| `/chassis_mode` | ChassisMode | 파워트레인→팔 | 지형/추종 모드 → 자세 락 | ⚠️ enum 미합의 |
| `/arm_status` | ArmStatus | 팔→파워트레인 | `PERCEIVING/PLANNING/EXECUTING/DONE/FAILED` | ⚠️ enum 미합의 |
| `/detected_objects` | DetectedObjectArray | 인식→파워트레인 | (팔 미사용) | ✅ Phase 2 |

**구간2용 status enum 제안 (파워트레인 팀 합의 필요):**
- `ArrivalStatus.status`: `ARRIVED_PICKUP`(박스 정렬 완료, 집어), `ARRIVED_DROP`(하역 지점 도착, 내려)
- `ArmStatus.status`: `PERCEIVING` → `PLANNING`(IK) → `EXECUTING`(하강/파지/리프트) → `CARRYING` → `DONE` / `FAILED`(재시도 한계 초과)
- `ChassisMode.mode`: `DRIVING`/`CORNERING`/`ROUGH_TERRAIN`(SAND·GRAVEL·WATER·ICE 통합)/`MISSION_STOP`/`FOLLOW_LEAD`

---

## 4. 로봇팔 FSM 상태표 (구간2 ARM_GRASP_BOX 중심)

drawio 구간2의 파지 서브루틴 + DROP 루프를 팔 노드 상태로 정리. 전류 임계 기반 파지/낙하 판정이 핵심.

| 상태 | 진입 조건 | 동작 | 전이 |
|------|-----------|------|------|
| `IDLE` | 초기 / DONE 후 | 홈 자세 대기, `/arm_status=IDLE` | `/arrival_status=ARRIVED_PICKUP` → `PERCEIVE` |
| `PERCEIVE` | 트리거 수신 | `/pick_target` 최신값 읽기(latched), pose 유효성 체크 | pose 유효 → `PLAN` / depth 무효 → 대기·재시도 |
| `PLAN` | 타겟 pose 확보 | MoveIt IK: 박스 상단 파지 포인트(454~754g 하중 고려) | 해 존재 → `DESCEND` / 해 없음 → `FAILED`(파워트레인에 재정렬 요청) |
| `DESCEND` | IK 성공 | 저속 하강 0.02 m/s, **그리퍼 전류 감시** | 전류 ≥ 임계 → `GRASP_CHECK` |
| `GRASP_CHECK` | 하강 정지 | CLOSE_GRIPPER, 전류 ≥ 임계 유지 확인 | 성공 → `LIFT` / 실패 → `DESCEND` 재시도(횟수 제한) |
| `LIFT` | 파지 성공 | 수직 리프트 → `CARRY_POSE` 고정 | 완료 → `CARRY` |
| `CARRY` | 운반 자세 | 자세 락 유지, **그리퍼 전류 주기 체크**, `/arm_status=CARRYING` | 전류 급감(DROP) → `REGRASP` / `ARRIVED_DROP` → `RELEASE` |
| `REGRASP` | DROP 감지(−2점) | `/arm_status=FAILED`(파워트레인 재정렬 유도) → `PERCEIVE` | 재시도 한계 초과 → `ABORT` |
| `RELEASE` | 하역 지점 도착 | 그리퍼 개방, 박스 내려놓기 | 완료 → `DONE` |
| `DONE` | 하역 완료 | `/arm_status=DONE`(파워트레인 재출발) → `IDLE` | → `IDLE` |
| `LOCKED` | `/chassis_mode∈{ROUGH_TERRAIN,CORNERING,FOLLOW_LEAD}` | 현재 관절각 홀드(진동 보호) | `DRIVING` 복귀 → 직전 상태 |

**자세 락(`LOCKED`)은 모든 상태에 우선하는 인터럽트.** CARRY 중 지형 모드면 CARRY_POSE 그대로 유지 + 전류 감시 계속. 구간1·3·4·5는 팔이 사실상 `IDLE`+`LOCKED` 조합으로만 동작.

---

## 5. 핸드셰이크 시퀀스 (구간2)

```
파워트레인                 인식노드(Phase2)          로봇팔 FSM
  │  DETECT_BOX(주행중)  ◀── /pick_target(latched) ──┐(상시 publish)
  │  ALIGN_BOX 정렬완료                               │
  ├── /arrival_status=ARRIVED_PICKUP ───────────────▶ IDLE→PERCEIVE→PLAN→DESCEND→GRASP→LIFT→CARRY
  │                                                   ├── /arm_status=EXECUTING/CARRYING ──▶ (대기)
  │  CARRY_MODE 주행 시작 ◀── (DONE 아님, 파지유지) ──┤
  │  지형 진입 → /chassis_mode=ROUGH_TERRAIN ────────▶ LOCKED(CARRY_POSE 유지)
  │  하역지점 → /arrival_status=ARRIVED_DROP ────────▶ RELEASE→DONE
  ├── /arm_status=DONE ◀──────────────────────────────┘
  │  재출발(레인 추종 재개)
```

DROP 발생 시: 팔이 `/arm_status=FAILED` → 파워트레인이 다시 `ALIGN_BOX`로 복귀 → `ARRIVED_PICKUP` 재발행 → 재파지 (drawio의 `DROP! → ALIGN_BOX` 루프와 일치).

---

## 6. 구현 방식 결정 (가) + 남은 선결 과제

### 결정: '가' — MoveIt 단일 경로 (2026-06-29)

> ⚠️ 처음엔 "MoveIt 실하드웨어 경로 부재 → position_node 직접 제어(A)"로 갔으나,
> **upstream/main에 이미 `moveit_dynamixel_bridge`(PR #9, commit 08ac318)가 있음**을
> PR 점검 중 발견. 이 브릿지가 `/arm_controller/follow_joint_trajectory` 액션을 구현해
> **MoveIt → 실제 다이나믹셀** 경로를 이미 뚫어놓음 → 전제가 바뀌어 **가로 전환**.

- **A. 모션 경로**: MoveIt 단일. FSM은 `move_action`(MoveGroup)에 목표 pose만 던지고
  IK·경로계획은 MoveIt이 수행 → `arm_controller` → `moveit_dynamixel_bridge` → 서보.
  IK 직접 구현(`_solve_ik`) 숙제 제거됨. position_node 직접 제어는 폐기(포트 경합 회피).

### 남은 선결 과제 (대부분 **브릿지 측** 작업으로 이관)

**B. 브릿지 effort(전류) 발행** ⚠️ 가장 시급
파지/DROP 판정은 그리퍼 current가 필요한데, 현재 `moveit_dynamixel_bridge`는
`/joint_states`에 **position만** 발행. → 브릿지가 GroupSyncRead로 PRESENT_CURRENT를 읽어
`/joint_states.effort`에 채우도록 확장해야 함. (FSM은 effort에서 읽도록 이미 구현)

**C. 그리퍼 실행 경로** — 그리퍼도 Dynamixel(결정 B). 브릿지/컨트롤러에 그리퍼 관절 +
`gripper_controller` FollowJointTrajectory 실행을 추가해야 함. 전류 임계값은 실측 캘리브.

**D. status enum 합의** (파워트레인 팀) — §3 잠정값(`ARRIVED_PICKUP`/`ARRIVED_DROP`/`DONE`...)
확정. drawio엔 핸드셰이크가 암묵적이라 명시 합의 필요.

**E. TF 연결** — `/pick_target` pose는 카메라 frame(`camera_color_optical_frame`) 기준.
MoveIt이 목표를 base_link로 변환하려면 카메라→base_link **TF**가 있어야 함.

**F. 구간4 제설 주체** — 팔로 치울지/밟고 갈지 미정(D 보류).

**G. 박스 파지 stroke** — 95mm 큐브, 454~754g. 그리퍼가 95mm를 잡는지 URDF 핑거 범위 확인(E 가능 확정).

---

## 7. 다음 단계

1. ✅ `arm_fsm_node.py` 스켈레톤(가 방향) — `dynamixel_control`, 빌드+mock 스모크테스트 통과.
2. **브릿지 확장**(§6-B effort + §6-C 그리퍼) — 가가 실제로 돌려면 최우선. upstream 머지 선행.
3. **TF**(§6-E) 카메라→base_link 연결 확인 (`ros2 run tf2_tools view_frames`).
4. `_carry_pose()` 구현(LIFT/CARRY 목표, base_link +Z 리프트) + 전류 임계 캘리브.
5. status enum 합의(§6-D) 후 `arm_fsm_node.py` 상단 상수 교체.
6. 구간2 단독 통합 테스트 (MoveIt+브릿지 기동 → `/arrival_status` mock → 파지 → `/arm_status=DONE`).
