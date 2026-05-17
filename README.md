# 2025 극한로봇 경진대회 — 개발 환경

ROS 2 Humble 기반 개발 환경을 Docker로 통일하여 협업합니다.
누구든 이 레포를 clone하고 `docker compose up`하면 동일한 환경에서 작업할 수 있습니다.

---

## 사전 요구사항

| 항목           | 버전       | 확인 명령어              |
| -------------- | ---------- | ------------------------ |
| Ubuntu         | 22.04 이상 | `lsb_release -a`         |
| Docker Engine  | 24.0 이상  | `docker --version`       |
| Docker Compose | v2 이상    | `docker compose version` |
| Git            | 아무 버전  | `git --version`          |

### Docker 설치 (없을 경우)

```bash
# Docker 공식 설치 스크립트
curl -fsSL https://get.docker.com | sudo sh

# sudo 없이 docker 사용 (로그아웃 후 재로그인 필요)
sudo usermod -aG docker $USER
```

---

## 처음 시작하기

### 1. 레포 클론

```bash
git clone https://github.com/ksp118/extreme-robot.git
cd extreme-robot
```

### 2. Docker 이미지 빌드

```bash
docker compose build
```

첫 빌드 시 ROS 2 베이스 이미지(약 3~4GB)를 받기 때문에 **10~20분** 걸립니다.
Dockerfile이 변경되지 않았다면 두 번째부터는 캐시로 수 초 내에 끝납니다.

### 3. 컨테이너 시작

환경에 따라 docker-compose 파일을 선택합니다:

```bash
# Ubuntu 네이티브
xhost +local:docker
docker compose up -d

# WSL2 (Windows)
xhost +local:
docker compose -f docker-compose.wsl.yml up -d
```

### 4. 컨테이너 진입

```bash
docker exec -it ros2_humble bash
```

진입하면 ROS 2 Humble 환경이 소싱된 상태입니다.
터미널을 여러 개 열어야 할 때는 새 터미널에서 같은 명령어를 반복하면 됩니다.

### 5. 동작 확인 (로봇팔 URDF)

```bash
# 컨테이너 안에서
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch robot_arm_description display.launch.py
```

RViz와 슬라이더 GUI가 뜨면 환경 구성 완료입니다.
RViz 초기 설정은 아래 [로봇팔 URDF 시각화](#로봇팔-urdf-시각화) 섹션을 참고하세요.

---

## 레포 구조

```
extreme-robot/
├── Dockerfile                  # 개발 환경 정의
├── docker-compose.yml          # Ubuntu 네이티브용 실행 설정
├── docker-compose.wsl.yml      # WSL2용 실행 설정
├── .gitignore                  # 빌드 산출물 제외
├── README.md
└── ros2_ws/                    # ROS 2 워크스페이스 (호스트와 공유)
    └── src/
        └── robot_arm_description/    # 로봇팔 URDF 패키지
            ├── urdf/                 # URDF 파일 (로봇 구조 정의)
            ├── launch/               # launch 파일
            ├── rviz/                 # RViz 설정
            ├── config/               # 기타 설정
            ├── CMakeLists.txt
            └── package.xml
```

`ros2_ws/` 디렉토리는 호스트와 Docker 컨테이너가 공유합니다.
호스트에서 `ros2_ws/src/` 아래 코드를 수정하면, 컨테이너 안에서 즉시 반영됩니다.

빌드 산출물(`build/`, `install/`, `log/`)은 `.gitignore`로 제외되어 있으므로
push되지 않습니다. 각자 컨테이너 안에서 `colcon build`로 빌드하세요.

---

## 로봇팔 URDF 시각화

### 빌드 및 실행

```bash
# 컨테이너 안에서
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch robot_arm_description display.launch.py
```

RViz, joint_state_publisher_gui 창이 함께 뜹니다.

### RViz 초기 설정 (처음 한 번만)

RViz가 처음 열리면 빈 화면에 로봇이 보이지 않습니다. 아래 순서대로 설정하세요:

1. 왼쪽 Displays 패널 → **Fixed Frame**이 `map`으로 되어 있음 → 클릭해서 `base_link`로 변경
2. 왼쪽 아래 **Add** 버튼 → **RobotModel** 선택 → OK
3. 추가된 RobotModel을 펼쳐서 **Description Topic** 항목을 찾기 → **Durability Policy**를 `Volatile`에서 `Transient Local`로 변경

설정이 완료되면 로봇팔 모델이 3D 뷰에 나타나며, joint_state_publisher_gui 슬라이더로 각 관절을 움직여볼 수 있습니다.

> **수정함** : Dockerfile에 `joint_state_publisher_gui` 추가. 이미지에 포함된 상태로 변경
> ~~**참고**: `joint_state_publisher_gui`가 없다는 에러가 나오면 컨테이너 안에서 설치하세요:~~
>~~
> ```bash
> apt update && apt install -y ros-humble-joint-state-publisher-gui
> ```
~~
---

## 일상적인 작업 흐름

### 아침에 시작할 때

```bash
cd extreme-robot
git pull                        # 팀원이 올린 변경사항 가져오기

# Ubuntu 네이티브
xhost +local:docker
docker compose up -d

# WSL2
xhost +local:
docker compose -f docker-compose.wsl.yml up -d

docker exec -it ros2_humble bash
```

### 코드 작업 후 push

```bash
# 호스트 터미널에서 (컨테이너 밖)
cd extreme-robot
git add .
git commit -m "feat: YOLO 객체인식 노드 추가"
git push
```

### 팀원이 올린 코드 가져오기

```bash
git pull
```

소스코드(`ros2_ws/src/`)만 git으로 관리되므로,
pull 후 컨테이너 안에서 다시 빌드하면 됩니다:

```bash
# 컨테이너 안에서
cd /root/ros2_ws
colcon build
source install/setup.bash
```

### 작업 끝날 때

```bash
docker compose down             # 컨테이너 종료
```

---

## Dockerfile이 변경된 경우

누군가 Dockerfile에 새 패키지를 추가하고 push했다면,
이미지를 다시 빌드해야 합니다:

```bash
git pull
docker compose down
docker compose build
docker compose up -d
```

---

## 새 ROS 2 패키지 추가하는 법

### apt 패키지 (SLAM, Nav2, MoveIt 등)

Dockerfile에 추가하고 push합니다:

```dockerfile
# Dockerfile의 apt-get install 목록에 추가
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-teleop-twist-keyboard \
    ros-humble-slam-toolbox \          # ← 새로 추가
    ros-humble-navigation2 \           # ← 새로 추가
    ...
```

```bash
git add Dockerfile
git commit -m "chore: SLAM, Nav2 패키지 추가"
git push
```

팀원들은 `git pull` 후 `docker compose build`로 동일 환경을 얻습니다.

### pip 패키지 (ultralytics 등)

마찬가지로 Dockerfile에 추가합니다:

```dockerfile
# Dockerfile 끝부분에 추가
RUN pip3 install ultralytics
```

### 직접 만든 ROS 2 패키지

`ros2_ws/src/` 아래에 패키지를 만들고 git으로 push합니다:

```bash
# 컨테이너 안에서
cd /root/ros2_ws/src
ros2 pkg create --build-type ament_python my_package

# 호스트에서 push
cd extreme-robot
git add ros2_ws/src/my_package
git commit -m "feat: my_package 패키지 생성"
git push
```

---

## 트러블슈팅

### GUI 창이 안 뜸

```bash
# Ubuntu 네이티브
xhost +local:docker

# WSL2
xhost +local:

# 그래도 안 되면 DISPLAY 변수 확인
echo $DISPLAY       # 보통 :0 또는 :1
```

### WSL2에서 GUI가 갑자기 안 될 때

WSLg가 죽은 경우입니다. PowerShell(관리자)에서 WSL을 재시작하세요:

```powershell
wsl --shutdown
```

이후 WSL 터미널을 다시 열고 컨테이너를 재시작합니다.

### 컨테이너 안에서 ros2 명령어 안 됨

```bash
source /opt/ros/humble/setup.bash
```

`.bashrc`에 이미 추가되어 있지만, 간혹 소싱이 안 될 때 수동으로 실행하세요.

### colcon build 에러

```bash
# 의존성 누락일 가능성 높음 — 먼저 rosdep 실행
cd /root/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### permission denied 에러 (ros2_ws 내 파일)

호스트와 컨테이너의 UID가 다를 때 발생합니다:

```bash
# 호스트에서
sudo chown -R $USER:$USER ros2_ws/
```

---

## 브랜치 전략 (권장)

```
main              ← 안정 버전만
├── feat/teleop   ← 원격 조종 개발
├── feat/vision   ← 비전/YOLO 개발
├── feat/slam     ← SLAM/자율주행 개발
└── feat/arm      ← 매니퓰레이터 개발
```

기능별로 브랜치를 나누고, 작동 확인 후 main에 merge합니다.

```bash
# 새 브랜치에서 작업 시작
git checkout -b feat/vision

# 작업 후 push
git add .
git commit -m "feat: 화재 타겟 인식 모델 학습"
git push -u origin feat/vision

# main에 merge (GitHub PR 또는 로컬)
git checkout main
git merge feat/vision
git push
```
