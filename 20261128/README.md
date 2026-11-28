# [3주차 - 11/28] LiDAR 기반 장애물 회피 주행과 ROS 2 시뮬레이션

## 학습 내용

- AMR(Autonomous Mobile Robot)과 자율주행 파이프라인
- 로봇의 위치 추정 및 환경 인식에 사용되는 센서
- 2D/3D LiDAR의 동작 원리와 데이터 구조
- Gazebo를 이용한 TurtleBot3 시뮬레이션
- RViz2를 이용한 로봇 및 센서 데이터 시각화
- LiDAR 기반 Simple Obstacle Avoidance
- Odometry를 이용한 로봇의 위치 및 방향 추정
- Euler Angle과 Quaternion
- State Machine 기반 장애물 회피
- Local Minima 문제와 Recovery Action
- TF(Transform)와 로봇 좌표계

## 1. High-level과 자율 이동 로봇

이전 실습에서는 ESP32를 이용하여 센서 데이터를 읽고 모터를 직접 제어했다. 이러한 방식으로도 기본적인 장애물 회피 주행은 가능하지만, 주변 환경을 분석하거나 목적지까지의 경로를 계산하는 작업은 더 많은 연산을 필요로 한다.

High-level에서는 다음과 같은 작업을 수행한다.

- 카메라 영상 처리 및 객체 인식
- LiDAR 데이터를 이용한 공간 정보 처리
- 로봇의 현재 위치 및 자세 추정
- 주변 환경의 지도 생성
- 목적지까지의 경로 계획
- 환경 변화에 따른 경로 재계획

High-level에서 계산한 제어 명령은 Low-level로 전달되며, MCU가 실제 모터와 액추에이터를 제어한다.

```text
              High-level
          Linux / ROS 2 / PC

        인지 → 계획 → 제어 명령
          ↑              │
          │              ↓
      Sensor Data    Motor Command
          ↑              │
          │              ↓
               Low-level
             MCU / Firmware
```

### 1-1. AMR(Autonomous Mobile Robot)

AMR은 주변 환경을 인지하고 스스로 이동 경로를 계획하여 목적지까지 이동하는 자율 이동 로봇이다.

대표적인 활용 사례로는 서빙 로봇, 배달 로봇, 물류 로봇, 로봇 청소기 등이 있다.

| 구분 | AGV | AMR |
|---|---|---|
| 명칭 | Automated Guided Vehicle | Autonomous Mobile Robot |
| 이동 방식 | 사전에 설정된 경로를 따라 이동 | 주변 환경을 인지하고 경로 계획 |
| 장애물 대응 | 정지 또는 사전에 정의된 동작 | 환경에 따라 경로 수정 |
| 이동 경로 | 비교적 고정적 | 환경에 따라 변경 가능 |

AMR은 단순히 장애물을 피하는 것에서 나아가 자신의 위치와 목적지를 파악하고, 이동 중 발생하는 환경 변화에 대응할 수 있어야 한다.

### 1-2. 자율주행 파이프라인

자율주행 시스템은 크게 Perception, Planning, Control 단계로 구성된다.

**Perception(인지)**

- 센서를 이용한 주변 환경 정보 수집
- 장애물의 위치 및 거리 파악
- 로봇 자신의 위치와 자세 추정

**Planning(계획)**

- 현재 위치에서 목적지까지의 경로 계산
- 장애물을 고려한 이동 방향 결정
- 환경 변화에 따른 경로 재계획

**Control(제어)**

- 계획된 경로를 따라가기 위한 속도 명령 생성
- 선속도와 각속도 결정
- Low-level에 제어 명령 전달

```text
Sensor Data
     ↓
Perception
     ↓
Planning
     ↓
Control
     ↓
Robot Movement
     │
     └──── Sensor Feedback
```

계획은 한 번만 수행하는 것이 아니라 로봇의 이동 중에도 계속 수정되어야 한다. 사람이 이동하거나 새로운 장애물이 나타나는 경우 기존 경로를 그대로 따라갈 수 없기 때문이다.

## 2. 로보틱스에서 사용되는 센서

### 2-1. 주요 센서

| Sensor | 측정 정보 | 활용 |
|---|---|---|
| Camera | 이미지 및 영상 | 객체 인식, 위치 추정 |
| LiDAR | 주변 물체까지의 거리 | 장애물 감지, SLAM |
| IMU | 가속도, 각속도 | 자세 및 움직임 추정 |
| GNSS | 위성 기반 위치 정보 | 실외 위치 추정 |
| Encoder | 모터 및 바퀴의 회전량 | 이동 거리 추정 |
| Ultrasonic | 초음파 기반 거리 | 근거리 장애물 감지 |
| Radar | 전파 기반 거리 및 속도 | 장애물 감지 |

이번 실습에서 사용하는 TurtleBot3 Burger는 LiDAR, IMU, 바퀴 Encoder를 활용한다.

### 2-2. IMU

IMU(Inertial Measurement Unit)는 로봇의 움직임과 자세를 추정하는 데 사용하는 센서이다.

- Accelerometer: 가속도 측정
- Gyroscope: 회전 각속도 측정
- Magnetometer: 자기장 측정 및 방향 추정(IMU 구성에 따라 포함)

IMU의 측정값을 적분하면 이동량이나 회전량을 추정할 수 있다. 하지만 센서 오차도 함께 누적되므로 시간이 지날수록 추정값과 실제 상태의 차이가 커질 수 있다.

이러한 현상을 Drift라고 한다.

Magnetometer는 주변 금속이나 전자기기의 영향을 받을 수 있어 실내 환경에서 사용하기 까다롭다.

### 2-3. GNSS

GNSS(Global Navigation Satellite System)는 여러 인공위성에서 전달되는 신호를 이용해 현재 위치를 추정하는 시스템이다.

GPS는 GNSS에 포함되는 위성항법 시스템 중 하나이다.

실외에서는 위치 추정에 활용할 수 있지만, 실내에서는 위성 신호가 차단되거나 약해지기 때문에 사용하기 어렵다.

이번 실습은 실내 이동 로봇을 대상으로 하므로 GNSS를 사용하지 않는다.

### 2-4. Encoder

Encoder는 모터나 바퀴의 회전량을 측정하는 센서이다.

바퀴의 반지름과 회전량을 알고 있다면 로봇이 이동한 거리를 계산할 수 있다.

바퀴의 반지름을 r, 회전 횟수를 n이라고 하면 이동 거리는 다음과 같다.

**d = 2πrn**

회전 각도를 Radian으로 나타내면 다음과 같다.

**d = rθ**

- d: 이동 거리
- r: 바퀴의 반지름
- n: 바퀴의 회전 횟수
- θ: 바퀴의 회전 각도(rad)

예를 들어 바퀴가 두 바퀴 회전했다면 바퀴 둘레의 두 배만큼 이동한 것으로 계산할 수 있다.

다만 바퀴가 미끄러지는 경우 실제 이동 거리와 Encoder로 계산한 이동 거리 사이에 오차가 발생한다.

Encoder를 이용한 이동량 추정은 이후 Odometry 계산에 활용된다.

## 3. LiDAR(Light Detection and Ranging)

### 3-1. LiDAR의 동작 원리

LiDAR는 레이저를 이용해 주변 물체까지의 거리를 측정하는 센서이다.

Time of Flight 방식에서는 레이저를 발사한 뒤 물체에 반사되어 돌아오기까지 걸린 시간을 측정한다.

```text
LiDAR ───── Laser ─────────> Obstacle
      <──── Reflection ─────
```

빛의 속도를 c, 왕복 시간을 t라고 하면 물체까지의 거리는 다음과 같다.

**d = ct / 2**

- d: 물체까지의 거리
- c: 빛의 속도
- t: 레이저의 왕복 시간

측정한 시간에는 레이저가 물체까지 이동하는 시간과 다시 센서로 돌아오는 시간이 모두 포함되어 있으므로 2로 나누어야 한다.

회전형 LiDAR는 여러 방향으로 레이저를 발사하여 주변 환경의 거리 정보를 수집한다.

### 3-2. LiDAR의 주요 사양

| Specification | 의미 |
|---|---|
| Field of View | 측정 가능한 각도 범위 |
| Minimum Range | 최소 측정 가능 거리 |
| Maximum Range | 최대 측정 가능 거리 |
| Angular Resolution | 각 측정 지점 사이의 각도 간격 |
| Scan Time | 스캔 사이의 시간 간격 |
| Sampling Interval | 각 측정값 사이의 시간 간격 |

LiDAR는 제품에 따라 측정 범위와 해상도가 다르다.

항상 측정 범위가 넓고 해상도가 높은 센서를 사용하는 것이 좋은 것은 아니다. 센서 데이터가 많아지면 처리해야 하는 연산량과 하드웨어 요구사항도 증가한다.

로봇의 사용 목적에 맞춰 센서의 성능, 연산량, 실시간성, 가격을 함께 고려해야 한다.

### 3-3. 2D LiDAR와 3D LiDAR

| 구분 | 2D LiDAR | 3D LiDAR |
|---|---|---|
| 측정 범위 | 하나의 평면 | 3차원 공간 |
| 좌표 표현 | X, Y | X, Y, Z |
| 데이터 표현 | Laser Scan | Point Cloud 등 |
| 연산량 | 비교적 적음 | 비교적 많음 |
| 활용 | 실내 이동 로봇, 2D SLAM | 자율주행, 3D SLAM, 정밀 측량 |

2D LiDAR는 센서가 위치한 높이의 평면을 스캔한다.

```text
2D LiDAR

Top View

    ● ● ● ● ●
  ●           ●
 ●    LiDAR    ●
  ●           ●
    ● ● ● ● ●
```

3D LiDAR는 여러 높이와 방향의 거리 정보를 수집하여 3차원 공간을 표현할 수 있다.

일반적인 다채널 3D LiDAR에서는 16, 32, 128채널 등 여러 측정 채널을 사용한다.

3차원 공간에서 측정한 점들의 집합을 Point Cloud라고 한다. 또한 3차원 공간을 일정한 크기의 셀로 나누어 표현할 때 Voxel이라는 단위를 사용하기도 한다.

이번 실습에서는 2D LiDAR를 사용한다.

### 3-4. LiDAR Scan과 Mapping의 차이

LiDAR에서 얻은 거리 데이터를 시각화하면 주변 벽과 장애물의 형태를 확인할 수 있다. 하지만 이것을 지도라고 할 수는 없다.

LiDAR Scan은 센서의 현재 위치를 기준으로 측정한 상대적인 거리 정보이다.

각 측정점의 거리와 각도를 이용하면 센서 좌표계에서의 위치를 계산할 수 있다.

**x = r cos(θ)**

**y = r sin(θ)**

- r: 센서에서 측정한 거리
- θ: 측정 방향
- x, y: 센서 좌표계에서의 측정점 위치

로봇이 이동하면 센서의 기준 위치도 달라지므로 새로운 Scan이 생성된다.

Mapping에서는 로봇의 위치와 자세를 고려하여 여러 위치에서 수집한 데이터를 공통 좌표계에 정렬하고 환경의 지도를 생성한다.

```text
LiDAR Scan

한 위치에서 측정한 상대적인 거리 정보
                 ↓
로봇의 위치 및 자세 고려
                 ↓
여러 Scan을 공통 좌표계에 정렬
                 ↓
Mapping
```

### 3-5. LiDAR의 한계

- 장애물 뒤쪽에 가려진 공간은 측정하기 어렵다.
- 유리와 거울처럼 빛을 투과하거나 반사하는 물체는 측정 오류를 발생시킬 수 있다.
- 비, 먼지 등의 외부 환경에 영향을 받을 수 있다.
- 2D LiDAR는 측정 평면보다 높거나 낮은 장애물을 감지하지 못할 수 있다.
- 실제 센서에는 측정 오차와 Noise가 존재한다.

이러한 한계 때문에 로봇에서는 하나의 센서에만 의존하지 않고 여러 센서의 정보를 함께 활용하기도 한다.

## 4. ROS 2 LiDAR 데이터 구조

### 4-1. LaserScan Message

ROS 2에서는 2D LiDAR의 거리 데이터를 주로 `sensor_msgs/msg/LaserScan` 형태로 전달한다.

Message 구조는 다음 명령어로 확인할 수 있다.

```bash
ros2 interface show sensor_msgs/msg/LaserScan
```

| Field | 의미 |
|---|---|
| `header` | 측정 시간 및 좌표계 정보 |
| `angle_min` | 스캔 시작 각도 |
| `angle_max` | 스캔 종료 각도 |
| `angle_increment` | 측정값 사이의 각도 간격 |
| `time_increment` | 측정값 사이의 시간 간격 |
| `scan_time` | 스캔 사이의 시간 간격 |
| `range_min` | 최소 측정 가능 거리 |
| `range_max` | 최대 측정 가능 거리 |
| `ranges` | 방향별 거리 측정값 배열 |
| `intensities` | 반사 신호의 강도 배열 |

각도는 Radian, 거리는 Meter 단위를 사용한다.

`intensities`는 반사 신호의 강도를 나타내며 물체의 재질, 반사 특성, 거리 등의 영향을 받는다.

이번 실습에서는 `intensities`를 사용하지 않고 `ranges`를 이용하여 장애물을 판단한다.

### 4-2. ranges와 측정 각도

`ranges`에는 각 방향에서 측정한 거리 정보가 순서대로 저장되어 있다.

```text
ranges = [1.2, 1.3, 1.5, 2.1, ...]
```

i번째 측정값의 각도는 다음과 같이 계산할 수 있다.

**θᵢ = θ_min + i × Δθ**

- θᵢ: i번째 측정점의 각도
- θ_min: 스캔 시작 각도
- Δθ: angle_increment
- i: ranges의 Index

360°를 약 1° 간격으로 측정한다면 약 360개의 거리 데이터를 얻을 수 있다.

**1° = π / 180 ≈ 0.01745 rad**

실습에서 확인한 LaserScan은 약 360°의 측정 범위와 약 1°의 각도 간격을 가진다.

단, 모든 LiDAR가 동일한 배열 길이와 시작 각도를 사용하는 것은 아니다.

따라서 `ranges[0]`이 항상 로봇의 정면이라고 가정해서는 안 되며, 센서의 장착 방향과 `angle_min`, `angle_increment`를 확인해야 한다.

### 4-3. LaserScan 데이터 확인

```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic info /scan -v
```

`/scan` Topic에서 전달되는 실제 거리 정보와 Message Type, Publisher 및 Subscriber 상태를 확인할 수 있다.

데이터를 처리할 때는 `range_min`, `range_max`의 측정 범위와 `inf`, `nan` 등의 값도 고려해야 한다.

## 5. Gazebo 시뮬레이션

### 5-1. Gazebo

Gazebo는 로봇의 물리적 움직임과 센서 동작을 가상 환경에서 시뮬레이션하는 도구이다.

- 로봇의 이동 및 회전
- 장애물과의 충돌
- 센서 데이터 생성
- 로봇에 작용하는 물리적 영향
- 가상 환경에서의 알고리즘 검증

실제 로봇에 알고리즘을 바로 적용하기 전에 시뮬레이션을 통해 기본적인 동작과 문제점을 확인할 수 있다.

### 5-2. TurtleBot3 모델 설정

이번 실습에서는 TurtleBot3 Burger 모델을 사용한다.

```bash
export TURTLEBOT3_MODEL=burger
```

터미널을 실행할 때마다 환경 변수를 설정하지 않도록 `.bashrc`에 추가할 수 있다.

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

설정값 확인:

```bash
echo $TURTLEBOT3_MODEL
```

### 5-3. Gazebo 실행

TurtleBot3 Gazebo Package의 DQN Stage 1과 Stage 2 환경을 이용하여 실습한다.

**DQN Stage 1**

```bash
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
```

**DQN Stage 2**

```bash
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage2.launch.py
```

Stage 1에서 기본적인 로봇 이동과 센서 데이터를 확인하고, Stage 2에서는 장애물이 존재하는 환경에서 회피 주행을 실습한다.

Launch 파일 이름은 설치된 TurtleBot3 Package에서 확인한 후 사용한다.

## 6. TurtleBot3 이동 제어

### 6-1. 로봇 좌표계

TurtleBot3 Burger는 좌우 바퀴의 회전 속도 차이를 이용해 이동하는 Differential Drive 방식의 로봇이다.

일반적인 ROS 로봇 좌표계는 다음과 같다.

```text
                 +X (Forward)
                      ↑
                      │
                      │
         +Y (Left) ← Robot

         +Z : 바닥에서 위쪽 방향
```

- X축: 로봇의 전후 방향
- Y축: 로봇의 좌우 방향
- Z축: 로봇의 위아래 방향

일반적인 Differential Drive 로봇은 옆으로 직접 이동할 수 없으므로 Y축 방향의 선속도를 사용하지 않는다.

| Field | 동작 |
|---|---|
| `linear.x` | 전진 및 후진 |
| `linear.y` | 직접 이동 불가 |
| `linear.z` | 직접 이동 불가 |
| `angular.x` | 일반적인 평면 주행에서 사용하지 않음 |
| `angular.y` | 일반적인 평면 주행에서 사용하지 않음 |
| `angular.z` | 좌회전 및 우회전 |

따라서 이번 실습에서는 `linear.x`와 `angular.z`를 이용하여 로봇의 움직임을 제어한다.

### 6-2. Twist와 TwistStamped

Twist는 로봇의 선속도와 각속도를 나타내는 Message이다.

TwistStamped는 Twist에 시간과 좌표계 정보를 추가한 형태이다.

```text
geometry_msgs/msg/TwistStamped
├── header
│   ├── stamp
│   └── frame_id
└── twist
    ├── linear
    │   ├── x
    │   ├── y
    │   └── z
    └── angular
        ├── x
        ├── y
        └── z
```

이번 시뮬레이션에서는 `/cmd_vel`의 Message Type으로 `TwistStamped`를 사용한다.

Python에서 각 속도값에 접근하는 방법은 다음과 같다.

```python
from geometry_msgs.msg import TwistStamped

msg = TwistStamped()

msg.twist.linear.x = 0.1
msg.twist.angular.z = 0.0
```

- `linear.x`: m/s
- `angular.z`: rad/s

실제 로봇이나 다른 ROS 2 환경에서는 `/cmd_vel`의 Message Type이 다를 수 있으므로 먼저 확인해야 한다.

```bash
ros2 topic info /cmd_vel
```

### 6-3. 터미널에서 속도 명령 발행

**전진**

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```

**제자리 회전**

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.5}}}"
```

**정지**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

전진할 때는 각속도를 0으로 설정하고, 제자리 회전할 때는 선속도를 0으로 설정한다.

정지할 때는 선속도와 각속도를 모두 0으로 설정한다. 반복 발행 중인 기존 속도 명령을 먼저 종료해야 한다.

### 6-4. Teleoperation

Teleoperation은 외부에서 로봇에 제어 명령을 전달하여 원격으로 조종하는 방식이다.

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

| Key | 동작 |
|---|---|
| W | 선속도 증가 |
| X | 선속도 감소 |
| A | 왼쪽 방향 각속도 증가 |
| D | 오른쪽 방향 각속도 증가 |
| S | 정지 |

키를 입력하면 Teleop Node에서 속도 명령을 발행하고 Gazebo의 로봇이 해당 명령을 수신하여 움직인다.

```text
Keyboard Input
      ↓
Teleop Node
      ↓
/cmd_vel
      ↓
TurtleBot3
```

## 7. RViz2를 이용한 데이터 시각화

### 7-1. Gazebo와 RViz2의 차이

| 구분 | Gazebo | RViz2 |
|---|---|---|
| 역할 | 물리 시뮬레이션 | ROS 2 데이터 시각화 |
| 환경 | 가상의 물리적 공간 | 좌표계 기반 시각화 공간 |
| 센서 | 가상 센서 데이터 생성 | 전달받은 센서 데이터 표시 |
| 실제 로봇 활용 | 주로 시뮬레이션 | 실제 로봇에서도 사용 |

Gazebo에서는 로봇이 존재하는 가상의 물리 환경을 확인할 수 있다.

RViz2에서는 로봇이 센서를 통해 어떤 정보를 얻고 있는지, 자신의 위치를 어떻게 추정하고 있는지 등을 확인할 수 있다.

### 7-2. RViz2 실행

Gazebo가 실행된 상태에서 새로운 터미널을 열고 다음 명령어를 실행한다.

```bash
rviz2
```

RViz2의 Add 기능을 이용하여 필요한 Display를 추가한다.

### 7-3. RobotModel 추가

RobotModel은 로봇의 몸체, 센서, 바퀴 등의 구조를 시각화한다.

설정 방법:

1. Add → RobotModel 선택
2. Description Source를 Topic으로 설정
3. Description Topic을 `/robot_description`으로 설정
4. Global Options의 Fixed Frame 확인

이번 실습에서는 아직 `map` Frame이 생성되지 않았으므로 Fixed Frame을 `base_footprint`로 설정한다.

존재하지 않는 Frame을 Fixed Frame으로 설정하면 Global Status에 오류가 표시될 수 있다.

### 7-4. LaserScan 추가

LiDAR의 거리 데이터를 시각화하기 위해 LaserScan을 추가한다.

1. Add → LaserScan 선택
2. Topic을 `/scan`으로 설정
3. 측정점의 크기 및 색상 설정
4. QoS 설정 확인

```text
ROS 2 Simulation
  ├── /robot_description ───→ RViz2 RobotModel
  ├── /scan ────────────────→ RViz2 LaserScan
  └── /tf ──────────────────→ 좌표계 변환
```

LaserScan의 점이 잘 보이지 않는 경우 Size를 조절할 수 있다. 실습에서는 0.01에서 0.03 정도로 크기를 변경하여 측정점을 확인했다.

### 7-5. Fixed Frame

Fixed Frame은 RViz2에서 데이터를 표시할 때 기준으로 사용하는 좌표계이다.

| Fixed Frame | 의미 |
|---|---|
| `map` | 지도 기준 시각화 |
| `odom` | Odometry 기준 시각화 |
| `base_footprint` | 로봇의 바닥 기준 시각화 |
| `base_link` | 로봇 몸체 기준 시각화 |

Fixed Frame에 따라 로봇과 주변 환경이 표시되는 방식이 달라진다.

예를 들어 `base_footprint`를 기준으로 설정하면 로봇은 기준 위치에 유지되고, 로봇의 이동에 따라 주변 환경의 상대 위치가 변하는 형태로 표시된다.

### 7-6. QoS 설정

LiDAR는 지속적으로 새로운 거리 데이터를 생성한다.

오래된 데이터가 누락되었을 때 이를 다시 받는 것보다 최신 데이터를 빠르게 받는 것이 중요한 경우가 많다.

따라서 LaserScan에는 Best Effort 정책을 사용할 수 있다.

| Reliability | 특징 |
|---|---|
| Reliable | 재전송 등을 통해 전달 신뢰성 확보 |
| Best Effort | 일부 데이터 유실을 허용 |

Publisher와 Subscriber의 QoS 설정이 호환되지 않으면 Topic이 존재하더라도 RViz2에서 데이터가 표시되지 않을 수 있다.

LaserScan이 표시되지 않는 경우 Topic 이름, Message Type, TF, QoS를 확인해야 한다.

## 8. Simple Obstacle Avoidance

LiDAR 데이터를 이용하여 로봇의 전진과 회전을 결정하는 기본적인 장애물 회피 알고리즘을 구현한다.

목적지나 지도 정보를 사용하는 것이 아니라 현재 측정된 장애물의 거리만을 이용하는 Reactive Obstacle Avoidance 방식이다.

### 8-1. 전방 한 지점을 이용한 장애물 감지

가장 간단한 방법은 로봇 정면의 거리 데이터 하나를 이용하는 것이다.

```text
          Obstacle
              ●
              │
              │ Distance
              │
            Robot
```

전방 거리를 Threshold와 비교하여 움직임을 결정한다.

- 전방 거리 > Threshold: 전진
- 전방 거리 ≤ Threshold: 정지 및 회전

하지만 로봇의 정면에 있는 한 지점만 확인하면 로봇 몸체의 너비를 고려하지 못한다.

정면은 비어 있더라도 로봇의 좌우 부분이 장애물에 충돌할 수 있다.

### 8-2. 전방 영역을 이용한 장애물 감지

한 지점만 사용하는 문제를 해결하기 위해 로봇의 정면을 기준으로 좌우 약 30°씩, 총 60° 범위의 거리 데이터를 사용한다.

```text
                Front
                  ↑
             \    │    /
              \   │   /
       Left    \  │  /    Right
        30°     \ │ /      30°
                 Robot
```

전방 영역의 최소 거리를 이용하여 장애물의 존재 여부를 판단한다.

**d_front = min(R_left ∪ R_right)**

- R_left: 왼쪽 영역의 거리 데이터
- R_right: 오른쪽 영역의 거리 데이터
- d_front: 전방 영역의 최소 거리

전방 영역 중 하나라도 Threshold 이하의 거리에 장애물이 존재하면 정지 및 회전하도록 구성한다.

최소 거리 대신 평균이나 가중 평균 등을 사용할 수도 있지만, 최소 거리는 가까운 장애물을 빠르게 감지할 수 있다는 장점이 있다.

다만 고정된 각도 범위만으로 로봇의 전체 충돌 가능 영역을 완전히 확인할 수 있는 것은 아니다.

### 8-3. 회전 방향 결정

왼쪽과 오른쪽 영역의 평균 거리를 비교하여 상대적으로 공간이 넓은 방향으로 회전한다.

**d_left_avg = (d_left,1 + ... + d_left,N) / N_L**

**d_right_avg = (d_right,1 + ... + d_right,N) / N_R**

- d_left_avg: 왼쪽 영역의 평균 거리
- d_right_avg: 오른쪽 영역의 평균 거리
- N_L: 왼쪽 영역의 측정점 개수
- N_R: 오른쪽 영역의 측정점 개수

| 조건 | 동작 |
|---|---|
| 왼쪽 평균 거리 > 오른쪽 평균 거리 | 좌회전 |
| 왼쪽 평균 거리 < 오른쪽 평균 거리 | 우회전 |

일반적인 ROS 좌표계에서는 `angular.z`가 양수이면 좌회전, 음수이면 우회전한다.

최소 거리는 장애물 감지에 사용하고, 좌우 평균 거리는 회전 방향을 결정하는 데 사용한다.

### 8-4. Node 통신 구조

장애물 회피 Node는 LiDAR 데이터를 구독하고 로봇의 속도 명령을 발행한다.

```text
LiDAR
  │
  │ /scan
  │ sensor_msgs/msg/LaserScan
  ↓
Obstacle Avoidance Node
  │
  ├── 전방 최소 거리 계산
  ├── 장애물 존재 여부 판단
  └── 회전 방향 결정
  │
  │ /cmd_vel
  │ geometry_msgs/msg/TwistStamped
  ↓
TurtleBot3
```

Node 내부에는 Subscriber와 Publisher가 모두 존재한다.

```python
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data

self.scan_sub = self.create_subscription(
    LaserScan,
    '/scan',
    self.scan_callback,
    qos_profile_sensor_data
)

self.cmd_pub = self.create_publisher(
    TwistStamped,
    '/cmd_vel',
    10
)
```

LiDAR 데이터가 수신되면 `scan_callback()`이 실행된다.

실제 Subscription의 QoS는 `/scan` Publisher의 설정에 맞게 구성해야 한다.

### 8-5. 기본 장애물 회피 로직

```text
              LiDAR Scan 수신
                     ↓
          전방 및 좌우 거리 계산
                     ↓
          최소 거리 > Threshold?
                /          \
              Yes           No
               ↓             ↓
             전진         선속도 0
                             ↓
                      좌우 평균 비교
                             ↓
                           회전
                \           /
                 속도 명령 발행
                       ↓
                   반복 수행
```

전방 최소 거리와 좌우 평균 거리를 계산한 이후의 제어 로직은 다음과 같이 구성할 수 있다.

```python
cmd = TwistStamped()
cmd.header.stamp = self.get_clock().now().to_msg()

if front_min > self.threshold:
    cmd.twist.linear.x = self.linear_speed
    cmd.twist.angular.z = 0.0

else:
    cmd.twist.linear.x = 0.0

    if left_avg > right_avg:
        cmd.twist.angular.z = self.angular_speed
    else:
        cmd.twist.angular.z = -self.angular_speed

self.cmd_pub.publish(cmd)
```

위 코드는 핵심 제어 로직을 나타낸 것으로, 거리 데이터 추출 및 초기화 부분은 생략했다.

### 8-6. Simple Obstacle Avoidance의 문제점

LiDAR 데이터가 수신될 때마다 전진과 회전을 다시 결정하기 때문에 장애물 주변에서 움직임이 불안정해질 수 있다.

- 전방 거리가 Threshold보다 조금만 커져도 다시 전진한다.
- 회전 중 새로운 센서 데이터가 들어오면 회전 방향이 변경될 수 있다.
- 벽이나 코너에서 전진과 회전을 반복할 수 있다.
- 좌우 평균 거리가 비슷한 환경에서 좌회전과 우회전을 반복할 수 있다.

목적지나 이전 이동 정보를 고려하지 않기 때문에 모든 환경에서 장애물을 벗어날 수 있는 것은 아니다.

이를 개선하기 위해 일정한 각도만큼 회전하는 방식을 적용한다.

## 9. Odometry

### 9-1. Odometry의 개념

Odometry는 로봇의 이동량을 이용하여 현재 위치와 방향을 추정하는 방법이다.

바퀴의 회전량이나 속도 정보를 이용하여 이전 위치에서 얼마나 이동했는지 계산하고, 이를 누적하여 현재 Pose를 추정한다.

```text
Initial Pose
     ↓
Movement 1
     ↓
Movement 2
     ↓
Movement 3
     ↓
Estimated Current Pose
```

예를 들어 초기 방향을 0°로 설정한 상태에서 다음과 같이 회전했다고 가정한다.

```text
초기 방향 :   0°
첫 회전   : +20°
두 번째   : -40°
현재 방향 : -20°
```

**현재 방향 = 0° + 20° - 40° = -20°**

Odometry는 일반적으로 초기 위치를 기준으로 상대적인 움직임을 추정한다.

센서 오차나 바퀴의 미끄러짐이 누적되기 때문에 지도상에서의 정확한 위치를 항상 제공하는 것은 아니다.

### 9-2. Odometry Message

ROS 2에서는 `nav_msgs/msg/Odometry`를 이용하여 로봇의 위치, 방향 및 속도 정보를 전달한다.

```bash
ros2 topic echo /odom
```

Message 구조 확인:

```bash
ros2 interface show nav_msgs/msg/Odometry
```

```text
nav_msgs/msg/Odometry
├── header
├── child_frame_id
├── pose
│   ├── pose
│   │   ├── position
│   │   │   ├── x
│   │   │   ├── y
│   │   │   └── z
│   │   └── orientation
│   │       ├── x
│   │       ├── y
│   │       ├── z
│   │       └── w
│   └── covariance
└── twist
    ├── twist
    │   ├── linear
    │   └── angular
    └── covariance
```

| Field | 의미 |
|---|---|
| `position` | 로봇의 추정 위치 |
| `orientation` | 로봇의 추정 방향 |
| `linear` | 선속도 |
| `angular` | 각속도 |
| `covariance` | 추정값의 불확실성을 표현하는 공분산 |
| `header` | 시간 및 기준 좌표계 |
| `child_frame_id` | 자식 좌표계 |

### 9-3. Odometry 데이터 접근

Odometry의 위치와 방향 데이터는 여러 Message가 포함된 계층 구조를 가진다.

```python
x = msg.pose.pose.position.x
y = msg.pose.pose.position.y

orientation = msg.pose.pose.orientation
```

현재 로봇의 속도는 다음과 같이 확인할 수 있다.

```python
linear_velocity = msg.twist.twist.linear.x
angular_velocity = msg.twist.twist.angular.z
```

방향 정보는 Quaternion으로 표현되므로 로봇의 회전 각도를 사용하려면 필요한 형태로 변환해야 한다.

## 10. Euler Angle과 Quaternion

### 10-1. Euler Angle

Euler Angle은 3차원 공간에서의 회전을 세 개의 각도를 이용하여 표현하는 방식이다.

로보틱스에서는 Roll, Pitch, Yaw를 많이 사용한다.

| Rotation | 회전축 | 의미 |
|---|---|---|
| Roll | X축 | 좌우 기울기 |
| Pitch | Y축 | 앞뒤 기울기 |
| Yaw | Z축 | 평면에서의 회전 방향 |

평면에서 이동하는 TurtleBot3의 장애물 회피에서는 주로 Yaw를 사용한다.

로봇이 30° 회전했다는 것은 평면상에서 Z축을 기준으로 방향이 30° 변했다는 의미이다.

### 10-2. Global Frame과 Local Frame

- Global Frame: 외부 환경을 기준으로 하는 좌표계
- Local Frame: 로봇 자체를 기준으로 하는 좌표계

로봇이 회전하면 Global Frame은 그대로 유지되지만 로봇의 Local Frame은 로봇과 함께 회전한다.

3차원 회전은 연산 순서에 따라 최종 자세가 달라질 수 있다.

예를 들어 X축 회전 이후 Y축 회전을 수행한 결과와, Y축 회전 이후 X축 회전을 수행한 결과는 일반적으로 동일하지 않다.

따라서 Euler Angle을 사용할 때는 회전 순서와 좌표계의 기준을 함께 고려해야 한다.

### 10-3. Gimbal Lock

Euler Angle의 문제점 중 하나는 Gimbal Lock이다.

3개의 회전축을 이용하여 자세를 표현하는 과정에서 특정 회전 상태에 도달하면 두 회전축이 겹치면서 독립적인 회전 자유도 하나를 잃게 되는 현상이다.

```text
Normal State

독립적인 회전축 3개
        ↓
회전 자유도 3개


Gimbal Lock

두 회전축이 일치
        ↓
독립적인 회전 자유도 2개
```

일반적인 Roll-Pitch-Yaw 표현에서는 Pitch가 ±90°에 도달할 때 이러한 특이점이 발생한다.

실제 로봇의 물리적 자유도가 사라지는 것이 아니라 Euler Angle을 이용한 자세 표현에서 문제가 발생하는 것이다.

ROS 2에서는 3차원 회전을 표현할 때 이러한 문제를 피할 수 있는 Quaternion을 사용한다.

### 10-4. Quaternion

Quaternion은 하나의 실수 성분과 세 개의 허수 성분을 이용하여 회전을 표현하는 방식이다.

**q = w + xi + yj + zk**

Euler Angle이 세 축에 대한 회전의 조합으로 자세를 표현한다면 Quaternion은 회전축과 해당 축에 대한 회전을 이용하여 자세를 나타낼 수 있다.

단위 회전축 벡터를 u = (uₓ, uᵧ, u_z), 회전 각도를 θ라고 하면 Quaternion은 다음과 같이 표현된다.

**w = cos(θ / 2)**

**x = uₓ × sin(θ / 2)**

**y = uᵧ × sin(θ / 2)**

**z = u_z × sin(θ / 2)**

따라서 x, y, z는 회전축과 관련된 성분이고 w는 회전 각도와 관련된 성분이다.

w 자체가 회전 각도를 의미하는 것은 아니다.

회전을 표현하는 Unit Quaternion은 다음 조건을 만족한다.

**x² + y² + z² + w² = 1**

회전이 없는 상태는 ROS 2의 Quaternion 필드 순서인 (x, y, z, w)를 기준으로 다음과 같이 나타낼 수 있다.

**q = (0, 0, 0, 1)**

### 10-5. Euler Angle과 Quaternion 비교

| 구분 | Euler Angle | Quaternion |
|---|---|---|
| 구성 | Roll, Pitch, Yaw | X, Y, Z, W |
| 표현 요소 | 3개 | 4개 |
| 직관성 | 비교적 높음 | 비교적 낮음 |
| Gimbal Lock | 발생 가능 | Euler Angle과 같은 특이점 없음 |
| ROS 2 활용 | 각도 해석 및 제어 | 자세 및 회전 정보 표현 |

Quaternion은 네 개의 성분으로 구성되지만 정규화 조건이 존재하므로 실제 회전 자유도는 3개이다.

### 10-6. Quaternion에서 Yaw 추출

장애물 회피 알고리즘에서 로봇을 일정 각도만큼 회전시키려면 현재 Yaw를 알아야 한다.

Odometry의 Quaternion을 Euler Angle로 변환한 뒤 Yaw를 사용한다.

변환 함수 사용 예시:

```python
from tf_transformations import euler_from_quaternion

orientation = msg.pose.pose.orientation

quaternion = [
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w
]

roll, pitch, yaw = euler_from_quaternion(quaternion)

self.current_yaw = yaw
```

`euler_from_quaternion()`은 Quaternion을 Roll, Pitch, Yaw로 변환한다.

평면 주행에서는 Roll과 Pitch보다 Yaw를 중심으로 회전 상태를 확인한다.

정규화된 Quaternion에서 Yaw만 필요한 경우 다음 수식을 이용할 수도 있다.

**Yaw = atan2(2(wz + xy), 1 - 2(y² + z²))**

## 11. State Machine 기반 장애물 회피

### 11-1. State Machine

State Machine은 시스템의 동작을 여러 상태(State)로 구분하고, 특정 조건에 따라 상태를 변경하는 방식이다.

기존 알고리즘에서는 LiDAR 데이터가 들어올 때마다 전진 또는 회전을 바로 결정했다.

따라서 회전이 충분히 이루어지기 전에 다시 전진하거나 반대 방향으로 회전할 수 있었다.

이를 개선하기 위해 로봇의 동작을 MOVE와 TURN 두 가지 상태로 나눈다.

| State | 동작 |
|---|---|
| MOVE | 전방 거리 확인 및 전진 |
| TURN | 지정된 목표 각도까지 회전 |

```text
              장애물 발견
     MOVE ──────────────────→ TURN
       ↑                        │
       │                        │
       └────────────────────────┘
             목표 각도 도달
```

TURN 상태에서는 전진 여부를 계속 판단하지 않고 목표 각도까지 회전하는 작업에 집중한다.

단, 실제 로봇에서는 회전 중에도 충돌 가능성을 확인해야 한다.

### 11-2. MOVE State

MOVE 상태에서는 기존과 동일하게 전방 장애물의 존재 여부를 확인한다.

장애물이 없으면 계속 전진한다.

장애물이 발견되면 다음 작업을 수행한다.

1. 로봇 정지
2. 왼쪽과 오른쪽의 평균 거리 비교
3. 회전 방향 결정
4. 현재 Yaw를 기준으로 목표 회전 각도 계산
5. TURN 상태로 변경

기존 알고리즘과 달리 장애물이 발견된 이후에는 회전 방향과 목표 각도를 결정하고 다음 상태로 넘어간다.

### 11-3. 목표 회전 각도 계산

이번 실습에서는 한 번의 회전 각도를 30°로 설정한다.

**θ_rotation = 30° = π / 6 rad**

회전 방향을 다음과 같이 정의한다.

| Direction | 값 |
|---|---:|
| Left | +1 |
| Right | -1 |

목표 회전 각도는 현재 각도에 회전 방향과 회전량을 적용하여 계산한다.

**θ_target = θ_current + s × θ_rotation**

- θ_target: 목표 각도
- θ_current: 현재 Yaw
- s: 회전 방향
- θ_rotation: 설정된 회전 각도

예를 들어 현재 방향이 19°일 때 왼쪽으로 30° 회전한다면 다음과 같다.

**θ_target = 19° + 30° = 49°**

반대로 오른쪽으로 30° 회전한다면 다음과 같다.

**θ_target = 19° - 30° = -11°**

### 11-4. Angle Normalization

각도는 한 바퀴를 회전하면 같은 방향을 나타내므로 범위를 일정하게 유지해야 한다.

예를 들어 370°와 10°는 같은 방향이다.

**370° ≡ 10°**

목표 각도를 0부터 2π 범위로 정규화하는 예시는 다음과 같다.

```python
import math

def normalize_angle(angle):
    return angle % (2.0 * math.pi)
```

현재 각도와 목표 각도 사이의 차이를 계산할 때도 각도의 순환성을 고려해야 한다.

```python
def angle_error(target, current):
    error = target - current

    return math.atan2(
        math.sin(error),
        math.cos(error)
    )
```

위 방식은 각도 차이를 -π부터 π 범위로 표현한다.

359°에서 1°로 회전하는 경우 단순히 각도를 빼면 -358°가 나오지만, 실제 가까운 방향의 각도 차이는 2°이다.

**1° - 359° = -358°**

**정규화된 각도 차이 = +2°**

따라서 회전 완료 여부를 판단할 때는 이러한 경계 조건도 고려해야 한다.

### 11-5. TURN State

TURN 상태에서는 Odometry에서 얻은 현재 Yaw와 목표 Yaw를 비교한다.

**e_θ = angle_error(θ_target, θ_current)**

- e_θ: 목표 방향까지의 각도 오차

목표 각도와 현재 각도가 완전히 일치할 때까지 기다리면 센서 오차나 회전량의 변화로 인해 회전이 종료되지 않을 수 있다.

따라서 Tolerance를 설정한다.

**|e_θ| ≤ ε**

- ε: 허용 가능한 각도 오차(Tolerance)

이번 실습에서는 Tolerance로 0.05 rad를 사용한다.

**ε = 0.05 rad ≈ 2.86°**

각도 오차가 허용 범위에 들어오면 회전을 완료한 것으로 판단하고 MOVE 상태로 변경한다.

### 11-6. 전체 알고리즘

```text
START
  ↓
State = MOVE
  ↓
현재 State 확인
  │
  ├── MOVE
  │     ↓
  │   전방 장애물 확인
  │     │
  │     ├── 없음 → 전진
  │     │
  │     └── 존재 → 정지
  │                  ↓
  │            회전 방향 결정
  │                  ↓
  │            목표 Yaw 계산
  │                  ↓
  │              State = TURN
  │
  └── TURN
        ↓
      현재 Yaw와 목표 Yaw 비교
        │
        ├── 오차 > Tolerance
        │          ↓
        │        회전 유지
        │
        └── 오차 ≤ Tolerance
                   ↓
                  정지
                   ↓
              State = MOVE
```

### 11-7. ROS 2 Node 구조

기존 장애물 회피 Node에 `/odom` Subscriber를 추가한다.

```text
/scan ────────┐
              ↓
       Obstacle Avoider
              ↑
/odom ────────┘
              │
              ↓
       State Machine
              │
              ↓
          /cmd_vel
```

| Topic | Message Type | 역할 |
|---|---|---|
| `/scan` | LaserScan | 장애물 거리 정보 수신 |
| `/odom` | Odometry | 현재 위치 및 방향 수신 |
| `/cmd_vel` | TwistStamped | 로봇 속도 명령 발행 |

`odom_callback()`에서는 Quaternion을 Euler Angle로 변환하여 현재 Yaw를 갱신한다.

`scan_callback()`에서는 현재 State와 장애물 정보를 바탕으로 속도 명령을 결정한다.

### 11-8. 핵심 코드 구조

```python
if self.state == 'MOVE':

    if front_min > self.threshold:
        cmd.twist.linear.x = self.linear_speed
        cmd.twist.angular.z = 0.0

    else:
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0

        self.turn_direction = (
            1 if left_avg > right_avg else -1
        )

        self.target_yaw = normalize_angle(
            self.current_yaw
            + self.turn_direction * self.rotation_angle
        )

        self.state = 'TURN'

elif self.state == 'TURN':

    error = angle_error(
        self.target_yaw,
        self.current_yaw
    )

    cmd.twist.linear.x = 0.0

    if abs(error) <= self.tolerance:
        cmd.twist.angular.z = 0.0
        self.state = 'MOVE'

    else:
        cmd.twist.angular.z = math.copysign(
            self.angular_speed,
            error
        )

self.cmd_pub.publish(cmd)
```

기존 Simple Obstacle Avoidance에서는 센서 데이터가 들어올 때마다 회전 방향을 다시 결정했다.

State Machine 방식에서는 회전이 시작되면 목표 방향에 도달하기 전까지 전진 상태로 돌아가지 않도록 구성한다.

위 코드는 강의에서 다룬 State Machine의 원리를 설명하기 위한 예시이며, 실제 사용 시에는 Odometry 수신 여부와 회전 중 안전 조건을 추가로 확인해야 한다.

## 12. Local Minima 문제

### 12-1. 코너에서 발생하는 문제

State Machine을 적용하여 일정한 각도만큼 회전하더라도 코너에 갇히는 문제가 발생할 수 있다.

예를 들어 90° 코너에서 장애물을 감지한 뒤 왼쪽으로 30° 회전했다고 가정한다.

회전한 방향에도 벽이 존재하면 다음 MOVE 상태에서 다시 장애물을 감지한다.

이때 오른쪽의 공간이 더 넓다고 판단하면 다시 오른쪽으로 30° 회전하게 된다.

```text
        Wall
  ─────────────────
                  │
       ↖ A        │
         ↕        │ Wall
       ↗ B        │
                  │
```

결과적으로 A 방향과 B 방향을 계속 반복하면서 코너를 벗어나지 못할 수 있다.

강의에서는 이처럼 주변 환경에 의해 로봇이 특정 구간에서 빠져나오지 못하는 문제를 Local Minima 문제로 설명했다.

### 12-2. 회전 각도를 증가시키는 방법의 한계

30° 회전으로 코너를 벗어나지 못한다면 회전 각도를 90° 또는 100° 이상으로 증가시키는 방법을 생각할 수 있다.

하지만 코너의 각도가 항상 90°인 것은 아니다.

- 좁은 코너
- 넓은 각도의 코너
- U자 형태의 장애물
- 서로 마주 보는 벽

환경마다 장애물의 형태가 다르기 때문에 회전 각도를 증가시키는 것만으로는 모든 상황을 해결하기 어렵다.

180° 회전을 사용하더라도 서로 마주 보는 장애물 사이에서는 앞뒤 이동을 반복할 수 있다.

따라서 회전 각도 변경 외에 별도의 탈출 동작이 필요하다.

## 13. Recovery Action

### 13-1. Recovery Action의 개념

Recovery Action은 로봇이 정상적인 동작을 수행하기 어려운 상황에서 문제를 해결하고 기존 동작으로 복귀하기 위한 행동이다.

장애물 회피 과정에서는 다음과 같은 상황에서 필요할 수 있다.

- 코너에서 좌우 회전을 반복하는 경우
- 장애물 사이에 갇힌 경우
- 반복적으로 같은 위치에서 장애물을 감지하는 경우
- 정상적인 전진이 이루어지지 않는 경우

기존 MOVE와 TURN 상태에 RECOVERY 또는 ESCAPE 상태를 추가하여 별도의 탈출 동작을 수행하도록 구성한다.

### 13-2. 후진을 이용한 탈출

코너에 갇힌 경우에는 제자리에서 회전만 반복하는 것보다 먼저 후진하여 공간을 확보하는 방법을 사용할 수 있다.

```text
1. 코너 진입

  ─────────────────
                  │
           Robot ↗│
                  │


2. 코너에 갇힌 상태

  ─────────────────
                  │
            Robot↔│
                  │


3. 후진하여 공간 확보

  ─────────────────
                  │
       Robot ↙    │
                  │


4. 회전 후 다른 방향으로 이동

  ─────────────────
                  │
    ← Robot       │
                  │
```

기본적인 Recovery Action은 다음과 같이 구성할 수 있다.

1. 코너에 갇혔는지 판단
2. 로봇 정지
3. 후진 방향의 장애물 확인
4. 후진하여 공간 확보
5. 새로운 이동 방향으로 회전
6. MOVE 상태로 복귀

후진 이후 회전하는 이유는 기존 위치에서는 회전이나 전진을 위한 공간이 부족할 수 있기 때문이다.

### 13-3. Recovery State Machine

기존 MOVE와 TURN 상태에 RECOVERY 상태를 추가한다.

```text
         ┌───────────────┐
         │               │
         ↓               │
       MOVE ──────────→ TURN
         ↑               │
         │               │ 반복 회전 등
         │               │ 탈출 실패 감지
         │               ↓
         └─────────── RECOVERY
                      │
                      ├── 정지
                      ├── 후진
                      └── 회전
```

RECOVERY 상태에서는 기존 장애물 회피 판단과 별도로 탈출 동작을 수행한다.

일반적인 장애물 감지 상황에서는 TURN 상태로 변경하고, 반복 회전 등으로 인해 갇혔다고 판단되는 경우에만 RECOVERY 상태를 사용한다.

장애물을 발견할 때마다 무조건 후진하는 방식과는 차이가 있다.

### 13-4. 코너에 갇힌 상태를 판단하는 방법

Recovery Action을 수행하기 위해서는 로봇이 현재 갇혀 있는 상태인지를 먼저 판단해야 한다.

강의에서 제시한 문제를 바탕으로 다음과 같은 판단 조건을 생각할 수 있다.

- 짧은 시간 동안 좌회전과 우회전이 반복되는 경우
- 일정 횟수 이상 장애물 회피 동작을 반복하는 경우
- 회전 이후에도 계속 장애물이 감지되는 경우
- Odometry상 실제 이동이 거의 이루어지지 않는 경우

구체적인 판단 기준과 Threshold는 로봇의 크기, 이동 속도 및 주행 환경에 맞게 설정해야 한다.

단순히 전방 거리가 짧다는 이유만으로 코너에 갇혔다고 판단해서는 안 된다.

### 13-5. Recovery Action Pseudocode

```text
Initialize state as MOVE

WHILE robot is running:

    Read LiDAR data
    Update current robot pose

    IF state is MOVE:

        IF front area is clear:
            Move forward

        ELSE:
            Stop
            Determine turn direction
            Calculate target yaw
            Change state to TURN

    ELSE IF state is TURN:

        IF robot is trapped:
            Stop
            Change state to RECOVERY

        ELSE IF target yaw is reached:
            Stop
            Change state to MOVE

        ELSE:
            Continue turning

    ELSE IF state is RECOVERY:

        Check rear clearance

        IF reversing is safe:
            Move backward
            Rotate toward a new direction
            Reset trapped-state information
            Change state to MOVE

        ELSE:
            Stop and reconsider escape direction

    Publish velocity command
```

실제 코드에서는 RECOVERY 상태를 BACKWARD와 ESCAPE_TURN 등으로 세분화하여 각 동작의 완료 여부를 관리할 수도 있다.

위 Pseudocode는 Recovery Action의 설계 예시이며, 후진 거리, 회전 완료 조건, 탈출 실패 시의 처리 방식 등은 별도로 구현해야 한다.

### 13-6. Wall Following

Wall Following은 벽과 일정한 거리를 유지하면서 이동하는 방식이다.

오른쪽 또는 왼쪽 벽을 기준으로 이동하도록 방향을 유지하면 반복적으로 좌우 회전을 수행하는 문제를 줄일 수 있다.

하지만 Wall Following에도 한계가 존재한다.

- 기둥과 같은 독립적인 장애물 주변을 계속 회전할 수 있다.
- 가까운 곳에 탈출 경로가 있어도 벽을 따라 불필요하게 이동할 수 있다.
- 목적지까지 비효율적인 경로로 이동할 수 있다.

따라서 Wall Following만 사용하는 것보다 장애물 회피, 후진, 회전, Recovery Action 등을 상황에 따라 조합하는 방식이 필요하다.

### 13-7. 장애물 회피 알고리즘 비교

| 구분 | Simple OA | State Machine OA | Recovery OA |
|---|---|---|---|
| 입력 | LiDAR | LiDAR, Odometry | LiDAR, 이전 동작 정보 등 |
| 주요 상태 | 전진 및 회전 판단 | MOVE, TURN | MOVE, TURN, RECOVERY |
| 회전 방식 | 매 Scan마다 판단 | 목표 각도까지 회전 | 회전 및 탈출 동작 |
| 이전 동작 고려 | X | 일부 고려 | O |
| 코너 대응 | 취약 | 반복 회전 가능 | 별도의 탈출 동작 수행 |
| 주요 한계 | 잦은 방향 변경 | Local Minima 문제 | 탈출 판단 및 조건 설정 필요 |

```text
Simple Obstacle Avoidance
             ↓
Problem
회전 중 방향 변경 및 불안정한 움직임
             ↓
State Machine
목표 각도까지 회전 유지
             ↓
Problem
코너에서 동일한 회전 동작 반복
             ↓
Recovery Action
후진 및 새로운 방향으로 회전
```

단순히 알고리즘의 복잡도를 높이는 것이 아니라 기존 알고리즘의 문제점을 확인하고 이를 해결하는 기능을 추가하는 방식으로 개선한다.

## 14. TF(Transform)

### 14-1. TF의 필요성

로봇에는 몸체, 바퀴, LiDAR, IMU 등 서로 다른 위치에 여러 장치가 존재한다.

각 장치에서 측정한 데이터는 해당 장치의 좌표계를 기준으로 표현될 수 있다.

예를 들어 LiDAR가 로봇 몸체보다 높은 위치에 장착되어 있다면 LiDAR의 측정점은 로봇 몸체의 중심 위치가 아닌 LiDAR의 위치를 기준으로 계산된다.

```text
       LiDAR Frame
            ●
            │
            │ 센서 장착 높이
            │
       base_link
            ●
            │
            │
     base_footprint
  ──────────●────────── Ground
```

센서에서 측정한 정보를 로봇의 위치나 지도상의 위치와 연결하려면 각 좌표계 사이의 위치 및 회전 관계를 알아야 한다.

TF는 이러한 좌표계 사이의 관계를 관리하고 좌표 변환을 수행할 수 있도록 하는 시스템이다.

### 14-2. Parent Frame과 Child Frame

TF는 좌표계 사이의 관계를 부모와 자식 구조로 표현한다.

```text
Parent Frame
      │
      │ Translation + Rotation
      ↓
Child Frame
```

- Parent Frame: 기준이 되는 좌표계
- Child Frame: Parent Frame을 기준으로 위치와 방향이 정의되는 좌표계

로봇 몸체를 기준으로 LiDAR가 어디에 위치하는지 정의했다면 로봇 몸체가 Parent Frame, LiDAR가 Child Frame이 된다.

### 14-3. TurtleBot3 TF Tree

일반적인 TurtleBot3의 좌표계 구조는 다음과 같이 표현할 수 있다.

```text
map
 └── odom
      └── base_footprint
           └── base_link
                ├── base_scan
                ├── imu_link
                ├── wheel_left_link
                └── wheel_right_link
```

실제 Frame 이름과 연결 구조는 로봇의 모델 및 설정에 따라 달라질 수 있다.

또한 SLAM이나 Localization을 실행하기 전에는 `map` Frame이 존재하지 않을 수 있다.

### 14-4. map Frame

`map`은 지도상의 위치를 표현하기 위한 기준 좌표계이다.

- 지도 기준 로봇의 위치 표현
- SLAM 및 Localization 결과 표현
- 목적지 및 장애물의 위치 표현

지도상에서 로봇이 어디에 위치하는지 나타낼 때 사용하는 좌표계이다.

### 14-5. odom Frame

`odom`은 Odometry를 이용하여 로봇의 이동을 표현하는 좌표계이다.

일반적으로 로봇이 Odometry 계산을 시작한 위치를 원점으로 사용한다.

로봇이 이동하면서 누적된 이동 정보를 바탕으로 로봇의 위치와 방향을 표현한다.

`odom`은 연속적인 위치 추정값을 제공하지만 시간이 지날수록 오차가 누적될 수 있다.

따라서 지도상의 위치와 Odometry상의 위치가 항상 일치하지는 않는다.

### 14-6. base_footprint

`base_footprint`는 로봇의 몸체 기준 위치를 바닥에 투영한 좌표계이다.

평면에서 이동하는 로봇의 위치와 방향을 표현하기에 적합하다.

- 2D 평면에서의 로봇 위치
- 평면 주행 및 내비게이션
- 로봇의 바닥 기준 좌표계

### 14-7. base_link

`base_link`는 로봇의 몸체에 고정된 기준 좌표계이다.

LiDAR, IMU, 바퀴 등 각 장치의 위치는 로봇 모델의 구조에 따라 `base_link`와 연결될 수 있다.

| Frame | 기준 |
|---|---|
| `map` | 지도 |
| `odom` | Odometry 좌표계 |
| `base_footprint` | 로봇의 바닥 투영 위치 |
| `base_link` | 로봇 몸체 |
| `base_scan` | LiDAR 센서 |

### 14-8. TF를 이용한 좌표 변환

LiDAR에서 장애물의 위치를 측정했다고 가정한다.

이 장애물의 위치는 처음에는 LiDAR 좌표계를 기준으로 표현된다.

TF를 이용하면 센서와 로봇 몸체, Odometry 및 지도 사이의 관계를 연결하여 지도 기준 장애물의 위치를 계산할 수 있다.

```text
장애물의 센서 기준 좌표
            ↓
LiDAR Frame → base_link
            ↓
base_link → base_footprint
            ↓
base_footprint → odom
            ↓
odom → map
            ↓
장애물의 지도 기준 좌표
```

좌표계 사이의 관계를 순서대로 연결하면 서로 다른 기준에서 표현된 데이터를 공통 좌표계로 변환할 수 있다.

TF는 센서 데이터 시각화뿐 아니라 위치 추정, 지도 생성, 장애물 회피, 내비게이션 등에 사용된다.

## 15. Static TF와 Dynamic TF

### 15-1. Static TF

Static TF는 시간에 따라 변하지 않는 두 좌표계 사이의 관계이다.

예를 들어 LiDAR와 IMU가 로봇 몸체에 고정되어 있다면 로봇이 이동하더라도 몸체와 센서 사이의 상대적인 위치는 변하지 않는다.

```text
base_link
 ├── LiDAR
 └── IMU
```

고정된 관계는 한 번 정의한 후 계속 사용할 수 있다.

ROS 2에서는 이러한 정보를 `/tf_static`을 통해 전달한다.

### 15-2. Dynamic TF

Dynamic TF는 시간에 따라 위치 또는 방향이 변하는 좌표계 사이의 관계이다.

로봇이 움직이면 Odometry 좌표계에서 로봇의 위치와 방향이 계속 달라진다.

```text
odom
  │
  │ 로봇 이동에 따라 계속 변경
  ↓
base_footprint
```

Dynamic TF는 로봇의 움직임에 맞춰 지속적으로 갱신해야 한다.

ROS 2에서는 주로 `/tf`를 통해 전달한다.

| 구분 | Static TF | Dynamic TF |
|---|---|---|
| 좌표 관계 | 고정 | 시간에 따라 변화 |
| 갱신 | 최초 정의 | 지속적으로 갱신 |
| Topic | `/tf_static` | `/tf` |
| 예시 | 몸체와 고정된 LiDAR | Odometry와 로봇 위치 |

### 15-3. Broadcaster와 Listener

TF에서는 Broadcaster와 Listener를 이용하여 좌표계 정보를 전달하고 사용한다.

- Broadcaster: 좌표계 사이의 Transform 정보를 전달
- Listener: Transform 정보를 수신하고 Buffer에 저장하여 필요한 좌표 변환에 사용

```text
TF Broadcaster
       │
       │ Transform Data
       ↓
TF Listener / Buffer
       ↓
Coordinate Transformation
```

Static TF Broadcaster는 고정된 좌표 관계를 전달하고, Dynamic TF Broadcaster는 시간에 따라 변화하는 관계를 지속적으로 전달한다.

## 16. TransformStamped

### 16-1. Message 구조

ROS 2에서 좌표계 사이의 위치와 회전 관계를 표현하기 위해 `geometry_msgs/msg/TransformStamped`를 사용한다.

```text
geometry_msgs/msg/TransformStamped
├── header
│   ├── stamp
│   └── frame_id
├── child_frame_id
└── transform
    ├── translation
    │   ├── x
    │   ├── y
    │   └── z
    └── rotation
        ├── x
        ├── y
        ├── z
        └── w
```

| Field | 의미 |
|---|---|
| `header.stamp` | Transform의 시간 정보 |
| `header.frame_id` | Parent Frame |
| `child_frame_id` | Child Frame |
| `translation` | 두 좌표계 사이의 위치 관계 |
| `rotation` | 두 좌표계 사이의 회전 관계 |

### 16-2. Translation과 Rotation

Translation은 Parent Frame을 기준으로 Child Frame이 얼마나 떨어져 있는지를 표현한다.

**t = (tₓ, tᵧ, t_z)**

Rotation은 Parent Frame을 기준으로 Child Frame이 얼마나 회전했는지를 표현하며 Quaternion을 사용한다.

**q = (qₓ, qᵧ, q_z, q_w)**

예를 들어 LiDAR가 `base_link`의 중심에서 Z축 방향으로 0.15m 위에 있고 상대적인 회전이 없다면 다음과 같이 표현할 수 있다.

```text
Parent Frame : base_link
Child Frame  : base_scan

Translation
x : 0.0
y : 0.0
z : 0.15

Rotation
x : 0.0
y : 0.0
z : 0.0
w : 1.0
```

위 수치는 좌표계 관계를 설명하기 위한 예시이다.

TF는 각 좌표계의 위치를 독립적으로 저장하는 것이 아니라 서로 다른 좌표계 사이의 상대적인 관계를 이용한다.

이번 강의에서는 TF의 개념과 데이터 구조까지 학습했으며, Broadcaster와 Listener를 직접 구현하는 실습은 다음 강의에서 진행할 예정이다.

## 17. ROS 2 디버깅

ROS 2에서는 Node 자체가 정상적으로 실행되더라도 Topic 이름, Message Type, QoS, 좌표계 등의 설정이 일치하지 않으면 로봇이 정상적으로 움직이지 않을 수 있다.

### 17-1. 주요 Topic

| Topic | 역할 |
|---|---|
| `/cmd_vel` | 로봇 속도 명령 |
| `/scan` | LiDAR 거리 데이터 |
| `/imu` | IMU 센서 데이터 |
| `/odom` | 위치, 방향 및 속도 추정 정보 |
| `/robot_description` | 로봇 모델 정보 |
| `/tf` | 동적으로 변화하는 좌표계 정보 |
| `/tf_static` | 고정된 좌표계 정보 |
| `/clock` | 시뮬레이션 시간 정보 |
| `/rosout` | ROS 2 로그 정보 |

### 17-2. Topic 확인 명령어

```bash
ros2 topic list
ros2 node list
```

실행 중인 Node와 존재하는 Topic을 확인한다.

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /scan
ros2 topic echo /odom
```

실제로 전달되는 데이터의 값을 확인한다.

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /scan -v
ros2 topic info /odom -v
```

Message Type, Publisher Count, Subscription Count, QoS 등의 정보를 확인한다.

### 17-3. 로봇이 움직이지 않는 경우

```text
Obstacle Avoidance Node 실행 확인
                ↓
/scan 데이터 수신 여부 확인
                ↓
scan_callback() 실행 여부 확인
                ↓
/cmd_vel Topic 발행 여부 확인
                ↓
실제 선속도 및 각속도 확인
                ↓
Message Type 및 QoS 확인
                ↓
Gazebo의 로봇 동작 확인
```

로봇이 움직이지 않는다고 해서 반드시 모터 제어 알고리즘에 문제가 있는 것은 아니다.

센서 데이터가 전달되지 않거나 Publisher와 Subscriber의 Message Type이 다를 수도 있으므로 통신 구조부터 확인해야 한다.

### 17-4. RViz2에서 데이터가 표시되지 않는 경우

- Fixed Frame이 실제로 존재하는지 확인
- RobotModel의 Description Topic 확인
- LaserScan의 Topic이 `/scan`인지 확인
- Publisher와 Subscriber의 QoS 호환 여부 확인
- TF를 통해 센서 좌표계를 기준 좌표계로 변환할 수 있는지 확인
- Global Status 및 각 Display의 Status 확인

### 17-5. Package 실행 오류

Python Node를 추가한 경우 `setup.py`의 `entry_points`에 실행 파일을 등록해야 한다.

```text
Workspace
└── src/
    └── LiDAR 관련 Package/
        ├── Python Package/
        │   ├── __init__.py
        │   └── 장애물 회피 코드
        ├── package.xml
        └── setup.py
```

빌드는 Workspace의 최상위 디렉토리에서 수행한다.

```bash
colcon build
source install/setup.bash
```

Node 실행:

```bash
ros2 run <package_name> <executable_name>
```

실행 파일을 찾지 못하는 경우 Package 이름과 Python Module 이름, `setup.py` 등록 여부를 확인한다.

### 17-6. 시뮬레이션 종료 및 정지

장애물 회피 Node를 종료하더라도 로봇에 마지막으로 전달된 속도 명령이 유지되는 경우가 있다.

따라서 Node를 종료할 때 선속도와 각속도가 모두 0인 정지 명령이 전달되도록 처리해야 한다.

Gazebo와 RViz2를 종료할 때는 실행한 터미널에서 `Ctrl+C`를 이용해 프로세스를 정상적으로 종료한다.

이전 시뮬레이션 프로세스가 남아 있는 경우 로봇이 중복으로 생성되거나 초기 상태가 정상적으로 설정되지 않을 수 있다.

## 18. 실습 정리

이번 실습에서는 LiDAR의 거리 데이터를 이용하여 장애물을 판단하고 ROS 2를 통해 로봇에 속도 명령을 전달하는 구조를 다뤘다.

첫 번째 알고리즘에서는 전방 최소 거리를 Threshold와 비교하여 장애물의 존재 여부를 판단하고, 좌우 평균 거리의 차이를 이용하여 회전 방향을 결정했다.

하지만 매 Scan마다 전진과 회전을 다시 판단하기 때문에 장애물 주변에서 불필요한 방향 변경이 발생할 수 있었다.

두 번째 알고리즘에서는 Odometry를 통해 현재 Yaw를 추정하고 State Machine을 적용하여 목표 각도에 도달할 때까지 회전하도록 구성했다.

그럼에도 코너와 같이 특정한 환경에서는 동일한 회전 동작이 반복될 수 있으므로, 후진과 회전을 포함하는 Recovery Action을 추가하는 방법을 학습했다.

```text
LiDAR + Odometry
         ↓
   Sensor Processing
         ↓
  Obstacle Detection
         ↓
     State Machine
    ├── MOVE
    ├── TURN
    └── RECOVERY
         ↓
      /cmd_vel
         ↓
   Robot Movement
```

마지막으로 TF를 통해 로봇 몸체와 각 센서의 좌표계 관계를 관리하는 방법을 학습했다.

현재 위치에서 측정한 LiDAR 데이터를 지도상의 정보로 활용하기 위해서는 로봇의 위치와 방향뿐 아니라 센서와 로봇 사이의 좌표 관계도 함께 고려해야 한다.
