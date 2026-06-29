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

## 6. 구현 갭 / 선결 과제 (코딩 전 해결 필요)

**A. MoveIt 실하드웨어 경로 부재** (가장 큰 갭)
현재 `robot_arm_moveit_config`는 `mock_components/GenericSystem`으로 **가짜 관절만** 구동. 실제 XL430을 MoveIt으로 움직이려면 **XL430용 `ros2_control` 하드웨어 인터페이스 작성 필요**(아직 없음). `dynamixel_control/position_node`는 토픽 기반이라 MoveIt FollowJointTrajectory와 직접 연결 안 됨.
→ 선택: (a) ros2_control HW 인터페이스 신규 작성, (b) FSM이 MoveIt 대신 position_node에 직접 goal 전송(IK는 별도 호출). 결정 필요.

**B. 그리퍼 전류 피드백 경로**
파지/DROP 판정의 핵심인 "전류 ≥ 임계"·"전류 급감"은 그리퍼 모터 current가 필요. 현재 `position_node`는 `DXL_IDS=[0..4]`(팔 5축) current를 `/dynamixel/state`로 읽지만, **그리퍼 핑거가 Dynamixel인지·어느 ID인지 불명확**(URDF는 prismatic finger, pick_test_pkg는 gripper_controller 사용). → 그리퍼 하드웨어·ID·전류 임계값 확정 필요.

**C. status enum 합의** (파워트레인 팀)
§3 제안값을 그대로 쓸지. 특히 `ARRIVED_PICKUP`/`ARRIVED_DROP`/`FAILED` 트리거 타이밍. drawio엔 팔↔파워트레인 핸드셰이크가 암묵적이라 명시 합의 필요.

**D. 구간4 제설 작업 주체** — 팔로 치울지/밟고 갈지 미정. 팔이면 두 번째 파지류 동작 추가 설계 필요.

**E. 박스 파지 포인트 IK** — 95mm 큐브 상단 파지, 454~754g 하중. 그리퍼 stroke가 95mm 박스를 잡는지 URDF 핑거 범위로 확인 필요.

---

## 7. 다음 단계 (이 문서 합의 후)

1. §6-A 결정 (MoveIt HW 인터페이스 vs position_node 직접) — **구현 방식 분기점**
2. §6-B 그리퍼 하드웨어/전류 확정
3. §3 status enum 파워트레인 팀 합의
4. `arm_fsm_node.py` 스켈레톤 작성 (§4 상태표 → transitions, 토픽 I/O만 먼저 mock으로)
5. 구간2 단독 통합 테스트 (`/arrival_status` mock 발행 → 파지 시퀀스 → `/arm_status=DONE`)
