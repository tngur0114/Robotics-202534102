# [2주차 - 11/21] 센서 통신 및 모터 제어, ROS 2 기초

## 학습 내용

- I2C, UART 통신 방식
- MPU6050 IMU 센서 데이터 처리
- LiDAR 데이터 수신 및 패킷 분석
- DC 모터 제어와 장애물 회피 주행
- ROS 2의 구조 및 개발 환경 설정
- DDS, Domain ID, QoS
- Node, Topic, Message, Service, Action
- Turtlesim을 이용한 로봇 제어
- Colcon Workspace 및 Package 생성
- Python 기반 Publisher와 Subscriber 구현

## 1. 임베디드 통신 프로토콜

지난 실습에서는 GPIO를 이용해 LED와 버튼을 제어했다. 이번 실습에서는 통신 프로토콜을 이용하여 센서 데이터를 읽고, 이를 바탕으로 로봇의 모터를 제어하는 방법을 다뤘다.

### 1-1. I2C(Inter-Integrated Circuit)

I2C는 두 개의 통신선을 이용하여 여러 장치와 데이터를 주고받는 직렬 통신 방식이다.

| 통신선 | 역할 |
|---|---|
| SDA(Serial Data) | 데이터 전송 |
| SCL(Serial Clock) | 클럭 신호 전송 |

클럭 신호에 맞춰 데이터를 주고받으며, 각 장치는 고유한 주소를 통해 구분한다.

실습에서 사용하는 ESP32의 I2C 핀은 다음과 같다.

| 통신선 | GPIO |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |

실제 연결 시에는 사용하는 보드의 Pinout을 확인해야 한다.

I2C 통신은 크게 다음 순서로 이루어진다.

```text
Start Condition
       ↓
Slave Address 전송
       ↓
데이터 읽기 또는 쓰기
       ↓
Stop Condition
```

Master가 통신을 시작하고 Slave의 주소를 지정하여 원하는 장치와 통신한다.

### 1-2. UART(Universal Asynchronous Receiver/Transmitter)

UART는 별도의 클럭 신호 없이 데이터를 주고받는 비동기 직렬 통신 방식이다.

| 통신선 | 역할 |
|---|---|
| TX(Transmit) | 데이터 송신 |
| RX(Receive) | 데이터 수신 |
| GND | 공통 기준 전위 |

송신 장치의 TX와 수신 장치의 RX를 연결한다.

```text
Device A          Device B

   TX ────────────── RX
   RX ────────────── TX
  GND ────────────── GND
```

별도의 클럭 신호가 없으므로 송신 측과 수신 측의 Baud Rate 및 통신 형식을 동일하게 설정해야 한다.

### 1-3. UART 데이터 형식

실습에서는 8N1 통신 형식을 사용한다.

| 구분 | 의미 |
|---|---|
| 8 | 8 Data Bits |
| N | No Parity |
| 1 | 1 Stop Bit |

```text
Start Bit | 8 Data Bits | Stop Bit
```

Parity Bit는 데이터의 오류를 간단하게 검출하기 위해 사용하는 비트이다. 데이터에 포함된 1의 개수가 짝수 또는 홀수가 되도록 추가 비트를 설정하는 방식이다.

8N1에서는 Parity Bit를 사용하지 않는다.

### 1-4. I2C와 UART 비교

| 구분 | I2C | UART |
|---|---|---|
| 통신 방식 | 동기식 | 비동기식 |
| 통신선 | SDA, SCL | TX, RX |
| 클럭 신호 | 사용 | 사용하지 않음 |
| 장치 구분 | 주소 사용 | 연결된 통신 포트 사용 |
| 실습 대상 | MPU6050 | LiDAR |

## 2. IMU 센서 데이터 처리

### 2-1. MPU6050

MPU6050은 3축 가속도계와 3축 자이로스코프를 포함한 IMU 센서이다.

| 센서 | 측정 데이터 |
|---|---|
| Accelerometer | X, Y, Z축 가속도 |
| Gyroscope | X, Y, Z축 회전 각속도 |
| Temperature Sensor | 내부 온도 |

ESP32와 MPU6050은 I2C를 통해 통신하며, 센서 내부 레지스터에 저장된 데이터를 읽어 사용한다.

### 2-2. 통신 초기화

MPU6050은 초기 상태에서 Sleep Mode로 동작하므로 데이터를 읽기 전에 Sleep Mode를 해제해야 한다.

`setup()`에서 수행하는 과정은 다음과 같다.

1. PC와 ESP32 사이의 시리얼 통신 초기화
2. SDA, SCL 핀을 지정하여 I2C 통신 초기화
3. MPU6050의 Slave Address 지정
4. 전원 관리 레지스터에 접근
5. Sleep Mode 해제

| 함수 | 역할 |
|---|---|
| `Serial.begin()` | PC와 ESP32 사이의 시리얼 통신 초기화 |
| `Wire.begin()` | I2C 통신 초기화 |
| `Wire.beginTransmission()` | 지정된 주소의 장치와 통신 준비 |
| `Wire.write()` | 레지스터 주소 또는 데이터 전송 |
| `Wire.endTransmission()` | 데이터 전송 완료 |
| `Wire.requestFrom()` | Slave 장치에 데이터 요청 |
| `Wire.read()` | 수신된 데이터 읽기 |

`Serial.begin()`과 `Wire.begin()`은 모두 통신을 초기화하는 함수지만 서로 다른 통신 인터페이스를 설정한다.

### 2-3. 센서 데이터 수신

MPU6050에서 한 번에 읽는 데이터는 총 14 Bytes이다.

| 데이터 | 크기 |
|---|---:|
| Acceleration X | 2 Bytes |
| Acceleration Y | 2 Bytes |
| Acceleration Z | 2 Bytes |
| Temperature | 2 Bytes |
| Gyroscope X | 2 Bytes |
| Gyroscope Y | 2 Bytes |
| Gyroscope Z | 2 Bytes |
| 합계 | 14 Bytes |

가속도와 각속도는 각 축마다 16비트 데이터로 저장된다.

온도 데이터를 사용하지 않더라도 이후 자이로스코프 데이터를 순서대로 읽기 위해 해당 바이트를 읽어 넘어가야 한다.

요청한 데이터가 모두 수신되지 않았다면 정상적인 센서 값으로 처리하지 않고 오류를 출력하거나 해당 측정을 건너뛴다.

### 2-4. Bit Shift와 Bitwise OR

센서에서 16비트 데이터를 읽을 때 8비트 데이터 두 개를 결합해야 한다.

```cpp
int16_t value = (Wire.read() << 8) | Wire.read();
```

위 식은 상위 바이트와 하위 바이트를 결합하는 원리를 나타낸다.

- `<< 8`: 첫 번째 데이터를 왼쪽으로 8비트 이동
- `|`: 두 번째 데이터와 Bitwise OR 연산 수행

```text
High Byte          00000001
Low Byte           10010011

High Byte << 8     00000001 00000000
Low Byte           00000000 10010011
                   -----------------
Bitwise OR         00000001 10010011
```

Bitwise OR 연산에서는 0과 특정 비트를 연산하면 해당 비트의 값이 유지되므로 두 개의 8비트 데이터를 하나의 16비트 데이터로 결합할 수 있다.

실제로는 데이터를 순서대로 읽어 각각의 변수에 저장한 후 결합하면 읽기 순서를 명확하게 관리할 수 있다.

### 2-5. 센서 데이터 단위 변환

레지스터에서 읽은 값은 Raw Data이므로 실제 물리량으로 사용하려면 단위 변환이 필요하다.

- 가속도: m/s²
- 각속도: deg/s 또는 rad/s

센서의 측정 범위와 Scale Factor에 따라 변환 값이 달라진다.

변환한 데이터는 `Serial.print()` 또는 `Serial.println()`을 이용해 Serial Monitor에서 확인할 수 있다.

## 3. LiDAR 센서 데이터 처리

### 3-1. LiDAR 설치 방향

LiDAR는 주변을 회전하면서 물체까지의 거리를 측정한다.

센서가 측정하는 각도와 실제 로봇의 앞, 뒤, 좌, 우 방향은 LiDAR의 장착 방향에 따라 달라진다.

따라서 장애물 회피 알고리즘을 구현하기 전에 다음 각도가 로봇의 어느 방향에 해당하는지 확인해야 한다.

```text
                  0°
                   │
                   │
       270° ──── LiDAR ──── 90°
                   │
                   │
                 180°
```

위 그림은 각도 구분을 위한 예시이며, 실제 로봇의 전방과 좌우 방향은 장착 상태와 센서의 각도 증가 방향을 기준으로 확인해야 한다.

### 3-2. LiDAR Packet

LiDAR는 측정한 거리 정보를 일정한 형식의 Packet으로 구성하여 ESP32에 전송한다.

실습에서 사용한 LiDAR의 Packet 크기는 47 Bytes이다.

| 구성 요소 | 역할 |
|---|---|
| Header | Packet 시작 위치 확인 |
| VerLen | 버전 및 데이터 구성 정보 |
| Speed | LiDAR 회전 속도 |
| Start Angle | 측정 시작 각도 |
| Distance | 물체까지의 거리 |
| Confidence | 측정 데이터의 신뢰도 정보 |
| End Angle | 측정 종료 각도 |
| Timestamp | 시간 정보 |
| CRC | 데이터 오류 검출 |

거리 데이터는 2 Bytes, Confidence는 1 Byte로 구성된다.

LiDAR 모델에 따라 Packet 구조가 다르기 때문에 실제 데이터를 분석할 때는 제조사의 통신 규격을 기준으로 해야 한다.

### 3-3. UART를 이용한 LiDAR 데이터 수신

LiDAR는 UART를 통해 Packet을 지속적으로 전송한다.

```text
LiDAR
  │ UART
  ↓
ESP32
  │ USB Serial
  ↓
PC Serial Monitor
```

ESP32는 LiDAR와 통신하는 동시에 PC로 데이터를 출력해야 하므로 서로 다른 시리얼 인터페이스를 사용한다.

- `Serial`: ESP32와 PC 사이의 통신
- `Serial2`: LiDAR와 ESP32 사이의 통신

LiDAR에서 데이터를 수신하는 과정은 다음과 같다.

1. 수신 버퍼에 데이터가 존재하는지 확인
2. Packet Header 확인
3. VerLen 확인
4. 나머지 데이터를 배열에 저장
5. 거리 및 각도 데이터 추출
6. 실제 단위로 변환

### 3-4. Little Endian 데이터 결합

LiDAR는 실습에서 사용한 데이터 형식을 기준으로 하위 바이트를 먼저 전송하고 상위 바이트를 나중에 전송한다.

따라서 IMU와는 바이트를 결합하는 순서에 차이가 있다.

```cpp
uint16_t value = low_byte | (high_byte << 8);
```

| 구분 | 데이터 전송 순서 |
|---|---|
| MPU6050 | High Byte → Low Byte |
| 실습용 LiDAR | Low Byte → High Byte |

데이터 크기가 동일하더라도 전송 순서를 잘못 해석하면 실제 측정값과 다른 값이 계산된다.

### 3-5. 각도 데이터 변환

LiDAR의 각도 데이터는 소수점을 포함한 실수가 아닌 정수 형태로 전송된다.

실습에서 사용하는 데이터는 실제 각도에 100을 곱한 형태로 저장되어 있으므로 수신한 값을 100으로 나누어 복원한다.

**\theta = \frac{\theta_{raw}}{100}**

예를 들어 수신한 각도 데이터가 9620이라면 실제 각도는 96.2°이다.

시작 각도와 종료 각도, 거리 데이터를 이용하여 각 방향의 측정값을 처리한다.

360° 방향의 거리 정보를 관리하기 위해 360개의 원소를 가지는 배열을 사용할 수 있다.

## 4. DC 모터 제어

### 4-1. Motor Driver

MCU의 GPIO만으로 모터 구동에 필요한 전류를 직접 공급하기는 어렵기 때문에 Motor Driver를 사용한다.

실습에서 사용하는 PCB에는 모터 구동에 필요한 회로가 구성되어 있어 ESP32에서는 제어 신호를 전달하는 방식으로 모터를 제어한다.

각 모터는 IN1과 IN2 두 개의 입력 신호를 사용한다.

### 4-2. 모터 동작 방향

실습용 PCB의 모터 제어 방식은 다음과 같다.

| IN1 | IN2 | 동작 |
|---|---|---|
| HIGH | LOW | 정방향 회전 |
| LOW | HIGH | 역방향 회전 |
| HIGH | HIGH | Brake |
| LOW | LOW | Coast(자유 회전) |

실제 전진과 후진 방향은 모터의 배선 및 장착 방향에 따라 달라질 수 있다.

4개의 모터를 각각 제어하므로 총 8개의 제어 핀을 사용한다.

```text
ESP32
  ├── IN1, IN2 → Motor 1
  ├── IN1, IN2 → Motor 2
  ├── IN1, IN2 → Motor 3
  └── IN1, IN2 → Motor 4
```

## 5. LiDAR 기반 장애물 회피 주행

IMU, LiDAR, Motor Driver 등의 동작을 확인한 후 센서 입력과 모터 제어를 연결하여 장애물 회피 알고리즘을 구성할 수 있다.

```text
LiDAR 거리 데이터 수신
          ↓
방향별 장애물 거리 확인
          ↓
장애물 존재 여부 판단
          ↓
이동 방향 결정
          ↓
Motor Driver 제어
          ↓
거리 데이터 재측정
```

기본적인 장애물 회피 방식은 다음과 같다.

- 전방에 장애물이 없으면 전진
- 전방 장애물이 일정 거리 이내에 존재하면 정지
- 주변 거리 정보를 비교하여 이동 가능한 방향 선택
- 회전 또는 후진한 뒤 다시 거리 측정

이번 장애물 회피 주행은 특정 목적지까지 최적 경로를 탐색하는 방식이 아니라, 현재 센서 정보를 이용하여 주변 장애물을 피하면서 이동하는 방식이다.

알고리즘의 동작 과정을 문서화할 때는 Pseudocode를 사용할 수 있으며, LaTeX와 Overleaf를 이용해 알고리즘을 정리할 수 있다.

## 6. High-level과 Low-level

지금까지 실습한 센서 데이터 수신과 모터 제어는 Low-level에 해당한다.

High-level에서는 센서로부터 전달받은 데이터를 처리하고 로봇이 어떤 행동을 수행할지 결정한다.

| 구분 | High-level | Low-level |
|---|---|---|
| 역할 | 인지, 계획, 판단 | 하드웨어 제어 |
| 주요 작업 | 위치 추정, 경로 계획 | 센서 데이터 수집, 모터 구동 |
| 개발 환경 | Linux, ROS 2 | MCU, Firmware |
| 주요 언어 | Python, C++ | C, C++ |
| 중요 요소 | 연산 성능 | 실시간성 |

```text
          High-level
       Linux / ROS 2
       인지 및 경로 계획
          ↑       │
   Sensor Data    │ Command
          │       ↓
          Low-level
        MCU / Firmware
       센서 및 모터 제어
```

실제 로봇은 충돌이나 파손 위험이 있고 장비 비용도 높기 때문에 High-level 알고리즘은 시뮬레이션 환경에서 먼저 검증하는 경우가 많다.

시뮬레이션에서 구현한 알고리즘을 실제 로봇 환경에 적용하는 과정을 Sim-to-Real이라고 한다.

## 7. ROS 2(Robot Operating System)

### 7-1. ROS의 정의

ROS는 이름에 Operating System이 포함되어 있지만 독립적인 운영체제는 아니다.

Linux와 같은 운영체제 위에서 로봇 소프트웨어 개발에 필요한 기능을 제공하는 Framework이자 Middleware이다.

### 7-2. ROS의 주요 기능

**Hardware Abstraction**

하드웨어마다 다른 데이터 형식이나 동작 방식을 추상화하여 개발자가 일관된 인터페이스를 사용할 수 있도록 한다.

**Inter-process Communication**

서로 독립적으로 실행되는 로봇 소프트웨어 구성 요소가 데이터를 주고받을 수 있도록 한다.

**Package Management**

기존에 개발된 로봇 소프트웨어 패키지를 설치하여 사용하거나 직접 개발한 기능을 패키지 형태로 관리하고 공유할 수 있다.

**Visualization and Simulation**

Gazebo, RViz 등의 도구를 활용하여 로봇의 상태와 센서 데이터를 확인하고 시뮬레이션 환경에서 알고리즘을 검증할 수 있다.

### 7-3. ROS 1과 ROS 2

| 구분 | ROS 1 | ROS 2 |
|---|---|---|
| 통신 구성 | ROS Master 기반 | 분산형 구조 |
| Discovery | Master를 통해 노드 등록 및 검색 | DDS 기반 자동 검색 |
| 통신 방식 | TCPROS 등 | DDS 기반 통신 |
| 보안 | 기본 보안 기능 제한적 | 인증, 암호화, 접근 제어 지원 가능 |
| QoS | 제한적 | 다양한 통신 정책 설정 가능 |
| 빌드 도구 | catkin | colcon |

ROS 1은 Master를 통해 노드의 등록과 검색을 관리한다. 실제 Topic 데이터는 연결된 노드 사이에서 전달된다.

ROS 2에서는 중앙 Master 없이 노드가 서로를 발견하고 통신할 수 있다.

## 8. ROS 2 개발 환경 설정

### 8-1. Ubuntu와 ROS 2 배포판

Ubuntu는 Linux 커널을 기반으로 하는 운영체제이다.

Ubuntu와 ROS 2는 배포판에 따라 지원하는 환경이 다르므로 호환되는 버전을 사용해야 한다.

| Ubuntu | 코드명 | ROS 2 |
|---|---|---|
| 20.04 LTS | Focal Fossa | Foxy |
| 22.04 LTS | Jammy Jellyfish | Humble |
| 24.04 LTS | Noble Numbat | Jazzy |

LTS(Long Term Support)는 장기간 유지보수를 지원하는 배포판을 의미한다.

실습에서는 Ubuntu 24.04와 ROS 2 Jazzy를 사용한다.

최신 버전이라고 해서 항상 개발에 적합한 것은 아니다. 사용하는 패키지, 라이브러리, 하드웨어와의 호환성도 고려해야 한다.

### 8-2. ROS 2 환경 변수 설정

ROS 2를 설치한 후 터미널에서 ROS 2 명령어를 사용하려면 환경 설정 파일을 불러와야 한다.

```bash
source /opt/ros/jazzy/setup.bash
```

매번 새로운 터미널에서 명령어를 입력하지 않도록 `.bashrc`에 추가할 수 있다.

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

- `.bashrc`: Bash 터미널을 실행할 때 읽는 설정 파일
- `echo`: 문자열 출력
- `>>`: 기존 파일의 마지막 부분에 내용 추가
- `source`: 지정된 파일의 내용을 현재 Shell 환경에 적용

설정된 ROS 2 배포판을 확인한다.

```bash
echo $ROS_DISTRO
```

출력 예시:

```text
jazzy
```

## 9. DDS와 Domain ID

### 9-1. DDS(Data Distribution Service)

DDS는 분산 시스템에서 데이터를 주고받기 위한 통신 표준이다.

ROS 2에서는 DDS를 기반으로 노드 사이의 Discovery와 데이터 통신을 수행할 수 있다.

- 중앙 Master 없이 노드 검색
- Publisher와 Subscriber 사이의 데이터 통신
- QoS를 이용한 통신 정책 설정
- 동일한 네트워크에 연결된 여러 장치 사이의 통신

대표적인 DDS 구현체에는 Fast DDS와 Cyclone DDS가 있다.

### 9-2. RMW 설정

RMW(ROS Middleware)는 ROS 2와 실제 Middleware 구현체 사이의 인터페이스이다.

이번 실습에서는 Cyclone DDS를 사용하도록 설정한다.

Cyclone DDS 패키지가 설치되어 있는 상태에서 다음 환경 변수를 설정한다.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

새로운 터미널에서도 적용되도록 설정하려면 `.bashrc`에 추가한다.

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

### 9-3. ROS_DOMAIN_ID

ROS_DOMAIN_ID는 ROS 2에서 통신 영역을 구분하기 위한 환경 변수이다.

무전기의 채널과 비슷한 개념으로, 동일한 Domain ID를 사용하는 노드끼리 서로를 발견하고 통신할 수 있다.

```text
Domain ID : 10

Node A ←────→ Node B


Domain ID : 20

Node C ←────→ Node D
```

서로 다른 Domain ID의 노드는 기본적으로 직접 통신하지 않는다.

여러 개발자가 같은 네트워크에서 실습하거나 서로 다른 로봇 시스템을 운용할 때 통신 영역을 구분하기 위해 사용한다.

현재 터미널에서 Domain ID를 설정한다.

```bash
export ROS_DOMAIN_ID=0
```

설정값 확인:

```bash
echo $ROS_DOMAIN_ID
```

`export`만 사용하면 현재 Shell과 그 하위 프로세스에 적용된다. 계속 사용하려면 `.bashrc`에 설정을 추가한다.

### 9-4. Talker와 Listener 통신 실습

Terminal 1:

```bash
ros2 run demo_nodes_cpp talker
```

Terminal 2:

```bash
ros2 run demo_nodes_cpp listener
```

Talker는 메시지를 지속적으로 발행하고 Listener는 해당 메시지를 수신한다.

```text
Talker
   │
   │ Hello World
   ↓
Listener
```

서로 다른 PC에서 실행하더라도 동일한 Domain ID와 통신 가능한 네트워크 환경을 사용하면 메시지를 주고받을 수 있다.

통신이 되지 않는다면 각 터미널의 Domain ID와 RMW 설정을 먼저 확인한다.

## 10. ROS 2 기본 구성 요소

### 10-1. Node

Node는 ROS 2에서 하나의 기능을 담당하는 논리적인 실행 단위이다.

하나의 로봇 안에서도 여러 개의 Node가 각자의 기능을 수행한다.

```text
Camera Node
     ↓
Perception Node
     ↓
Planning Node
     ↓
Control Node
```

각 Node는 독립적으로 구성할 수 있으며 다른 Node와 데이터를 주고받는다.

실습에서는 터미널마다 하나의 Node를 실행하여 확인했지만, 하나의 프로세스에서 여러 Node를 실행하는 것도 가능하다.

### 10-2. Publisher와 Subscriber

- **Publisher**: 특정 Topic에 데이터를 발행하는 역할
- **Subscriber**: 특정 Topic을 구독하여 데이터를 수신하는 역할

```text
Publisher ───── Topic ─────> Subscriber
                Message
```

예를 들어 LiDAR Node가 거리 데이터를 발행하면 장애물 감지 Node가 해당 데이터를 구독하여 장애물 유무를 판단할 수 있다.

### 10-3. Topic

Topic은 Publisher와 Subscriber 사이에서 메시지를 전달하는 이름이 지정된 통신 채널이다.

Publisher와 Subscriber가 통신하려면 Topic 이름과 Message Type이 일치해야 하며 QoS도 호환되어야 한다.

여러 로봇을 구분하거나 이름 충돌을 방지하기 위해 Namespace를 사용할 수 있다.

```text
/robot1/cmd_vel
/robot2/cmd_vel
```

두 Topic 모두 속도 명령을 의미하지만 Namespace를 통해 서로 다른 로봇의 명령으로 구분할 수 있다.

### 10-4. Message

Message는 Topic을 통해 실제로 전달되는 데이터이다.

ROS 2에서는 미리 정의된 다양한 Message Type을 사용할 수 있다.

| Message Package | 용도 |
|---|---|
| `std_msgs` | 기본 데이터 타입 |
| `geometry_msgs` | 위치, 자세, 속도 등의 기하학적 정보 |
| `sensor_msgs` | 센서 데이터 |
| `nav_msgs` | 지도, Odometry, 경로 등 내비게이션 정보 |

Message는 다른 Message를 내부에 포함하는 계층적인 구조를 가질 수 있다.

### 10-5. Service와 Action

Topic은 지속적인 데이터 전달에 사용하지만 요청과 응답이 필요한 경우에는 Service 또는 Action을 사용한다.

| 구분 | Topic | Service | Action |
|---|---|---|---|
| 구조 | Publish / Subscribe | Request / Response | Goal / Feedback / Result |
| 용도 | 지속적인 데이터 전송 | 특정 작업 요청 | 장시간 수행되는 작업 |
| 예시 | 센서 데이터, 속도 명령 | 로봇 초기화 | 목적지까지 이동 |

Action은 목표를 수행하는 동안 진행 상황을 Feedback으로 전달하고 작업이 완료되면 Result를 반환한다.

## 11. ROS 2 Message 구조

### 11-1. Pose

Pose는 로봇의 위치(Position)와 방향(Orientation)을 포함하는 정보이다.

```text
geometry_msgs/msg/Pose
├── position
│   ├── x
│   ├── y
│   └── z
└── orientation
    ├── x
    ├── y
    ├── z
    └── w
```

Position은 3차원 공간에서의 위치를 나타낸다.

Orientation은 Quaternion을 이용하여 회전 및 방향을 표현한다.

특정 데이터에 접근할 때는 내부 Message 구조를 따라 접근한다.

```python
pose.position.x
pose.orientation.w
```

### 11-2. Twist

Twist는 로봇의 선속도(Linear Velocity)와 각속도(Angular Velocity)를 표현하는 Message이다.

```text
geometry_msgs/msg/Twist
├── linear
│   ├── x
│   ├── y
│   └── z
└── angular
    ├── x
    ├── y
    └── z
```

평면에서 이동하는 일반적인 이동 로봇은 주로 `linear.x`와 `angular.z`를 사용한다.

- `linear.x`: 전후 방향 선속도
- `angular.z`: Z축 기준 회전 각속도

### 11-3. TwistStamped

Stamped가 포함된 Message는 시간 및 좌표계 정보를 함께 전달할 수 있다.

```text
geometry_msgs/msg/TwistStamped
├── header
│   ├── stamp
│   │   ├── sec
│   │   └── nanosec
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

- `stamp`: 데이터의 시간 정보
- `frame_id`: 데이터가 기준으로 하는 좌표계
- `twist`: 선속도 및 각속도

```python
msg.twist.linear.x = 1.0
msg.twist.angular.z = 0.5
```

Message를 계층적으로 구성하면 기존 Message Type을 재사용할 수 있고 복잡한 데이터도 구조적으로 관리할 수 있다.

## 12. ROS 2 CLI 명령어

### 12-1. 기본 명령어

ROS 2 명령어의 기본 구조는 다음과 같다.

```bash
ros2 run <package_name> <executable_name>
```

또는

```bash
ros2 <category> <verb> <arguments>
```

| 명령어 | 기능 |
|---|---|
| `ros2 node list` | 실행 중인 Node 확인 |
| `ros2 node info` | Node의 상세 정보 확인 |
| `ros2 topic list` | 현재 Topic 목록 확인 |
| `ros2 topic echo` | Topic의 Message 출력 |
| `ros2 topic info` | Topic의 정보 확인 |
| `ros2 topic pub` | Topic에 Message 발행 |
| `ros2 service list` | Service 목록 확인 |
| `ros2 service call` | Service 요청 |
| `ros2 interface show` | Message 및 Service 구조 확인 |

Tab 자동완성을 사용하면 실행 가능한 Package, Topic, Message Type 등을 확인할 수 있다.

### 12-2. Topic 확인

Talker와 Listener를 실행한 상태에서 다음 명령어를 사용한다.

```bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter
```

주요 Topic은 다음과 같다.

| Topic | 역할 |
|---|---|
| `/chatter` | Talker와 Listener 사이의 메시지 전달 |
| `/parameter_events` | Parameter 변경 관련 정보 |
| `/rosout` | ROS 2 로그 정보 |

### 12-3. rqt_graph

Node와 Topic 사이의 연결 관계를 시각적으로 확인할 수 있는 도구이다.

```bash
rqt_graph
```

Talker와 Listener의 연결 구조:

```text
/talker ───── /chatter ─────> /listener
```

복잡한 로봇 시스템에서 어떤 Node가 데이터를 발행하고 어떤 Node가 구독하는지 확인할 때 유용하다.

### 12-4. 터미널에서 Message 발행

별도의 코드를 작성하지 않고 CLI 명령어를 이용해 Topic에 데이터를 발행할 수 있다.

```bash
ros2 topic pub /test_topic std_msgs/msg/String "{data: 'Hello'}"
```

다른 터미널에서 해당 Topic을 구독한다.

```bash
ros2 topic echo /test_topic
```

Message Type에 정의된 필드 이름을 정확하게 지정해야 한다. String Message의 경우 실제 문자열은 `data` 필드에 저장된다.

발행 횟수와 주기도 설정할 수 있다.

```bash
ros2 topic pub --once /chatter std_msgs/msg/String "{data: 'Hello'}"
```

```bash
ros2 topic pub -r 10 /chatter std_msgs/msg/String "{data: 'Hello'}"
```

- `--once`: 메시지를 한 번 발행
- `-r 10`: 10Hz 주기로 발행(초당 10회)

### 12-5. 속도 명령 발행

`/cmd_vel`은 이동 로봇의 속도 명령에 일반적으로 사용하는 Topic 이름이다.

다음은 Twist 타입의 속도 명령을 발행하는 예시이다.

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

설정하지 않은 필드는 기본값인 0으로 처리된다.

정지 명령은 선속도와 각속도를 모두 0으로 설정하여 전달한다.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

실제 로봇에서는 구독하는 Topic 이름, Message Type, 제어기의 속도 제한 및 Timeout 설정을 확인해야 한다.

## 13. QoS(Quality of Service)

QoS는 ROS 2에서 데이터를 어떤 정책에 따라 전달하고 관리할지 설정하는 기능이다.

Publisher와 Subscriber의 QoS 설정이 호환되지 않으면 Topic 이름과 Message Type이 같더라도 데이터를 수신하지 못할 수 있다.

### 13-1. Reliability

데이터의 전달 신뢰성을 설정한다.

| 구분 | Reliable | Best Effort |
|---|---|---|
| 데이터 유실 | 재전송 등을 통해 전달 신뢰성 확보 | 일부 유실 허용 |
| 특징 | 데이터 전달 신뢰성 중심 | 최신 데이터 및 낮은 지연 중심 |
| 활용 예시 | 중요한 제어 명령 | LiDAR 등 고주기 센서 데이터 |

LiDAR와 같이 데이터를 빠르게 갱신하는 센서는 오래된 데이터보다 최신 데이터가 중요한 경우가 많으므로 Best Effort를 사용할 수 있다.

반대로 정지 명령과 같이 전달 여부가 중요한 데이터는 Reliable을 고려할 수 있다. 다만 실제 로봇의 안전을 위해서는 통신 설정 외에도 별도의 정지 및 오류 처리 기능이 필요하다.

### 13-2. Durability

이미 발행한 데이터를 이후에 구독을 시작한 Subscriber에게도 전달할지 결정하는 정책이다.

| 구분 | Volatile | Transient Local |
|---|---|---|
| 이전 데이터 보관 | 보관하지 않음 | Publisher에서 일정 범위의 데이터 보관 |
| 늦게 구독한 Node | 이전 데이터 수신 불가 | 호환 설정 시 이전 데이터 수신 가능 |
| 활용 예시 | 실시간 센서 데이터 | 지도 데이터 |

지도는 한 번 생성하거나 불러온 후 여러 Node에서 사용해야 하는 경우가 있다.

Volatile을 사용하면 지도가 발행된 이후에 구독을 시작한 Node가 기존 지도 데이터를 받지 못할 수 있다.

Transient Local을 이용하면 늦게 구독을 시작한 Node도 Publisher가 보관 중인 지도 데이터를 받을 수 있다.

### 13-3. QoS 확인

```bash
ros2 topic info /chatter --verbose
```

확인 가능한 주요 정보:

- Message Type
- Publisher Count
- Subscription Count
- Reliability
- Durability

센서 데이터를 수신하지 못하거나 시각화되지 않는 경우 QoS 설정도 확인해야 한다.

## 14. Turtlesim 실습

Turtlesim은 ROS 2의 Node, Topic, Message, Service 등의 기본 동작을 확인할 수 있는 2D 시뮬레이션 패키지이다.

### 14-1. Turtlesim 실행

Terminal 1:

```bash
ros2 run turtlesim turtlesim_node
```

Terminal 2:

```bash
ros2 run turtlesim turtle_teleop_key
```

Teleoperation(Teleop)은 외부에서 로봇을 원격으로 조종하는 것을 의미한다.

키보드 방향키를 입력하면 Teleop Node가 속도 명령을 발행하고 Turtlesim Node가 해당 메시지를 구독하여 움직임을 반영한다.

```text
Keyboard
    ↓
Teleop Node
    ↓
/turtle1/cmd_vel
    ↓
Turtlesim Node
    ↓
Turtle Movement
```

### 14-2. Node 및 Topic 확인

```bash
ros2 node list
ros2 topic list
rqt_graph
```

주요 Topic:

| Topic | 역할 |
|---|---|
| `/turtle1/cmd_vel` | 속도 명령 |
| `/turtle1/pose` | 현재 위치, 방향 및 속도 |
| `/turtle1/color_sensor` | 현재 위치의 색상 정보 |

현재 Turtle의 상태를 확인한다.

```bash
ros2 topic echo /turtle1/pose
```

Turtlesim은 평면에서 동작하므로 위치를 X, Y 좌표로 나타내며 방향은 theta로 표현한다.

### 14-3. 직접 속도 명령 발행

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
```

선속도와 각속도를 동시에 설정하면 Turtle은 전진하면서 회전하므로 원형 경로를 따라 이동한다.

평면 이동 로봇에서 주로 사용하는 값은 다음과 같다.

| 필드 | 의미 |
|---|---|
| `linear.x` | 전후 방향 이동 |
| `angular.z` | 제자리 회전 방향의 각속도 |

### 14-4. Service 실습

Turtlesim에서는 화면 초기화, 로봇 생성, 삭제, 위치 이동 등을 Service로 요청할 수 있다.

Service 목록 확인:

```bash
ros2 service list
```

화면에 그려진 이동 경로 제거:

```bash
ros2 service call /clear std_srvs/srv/Empty "{}"
```

Turtle을 지정된 위치로 이동:

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.0, y: 5.0, theta: 0.0}"
```

Service를 호출할 때는 Service 이름과 Service Type을 지정하고 필요한 Request 데이터를 전달한다.

## 15. Colcon Workspace

### 15-1. Workspace 구조

Workspace는 여러 ROS 2 Package를 개발하고 빌드하기 위한 작업 공간이다.

```text
colcon_ws/
├── src/
│   ├── package_1/
│   ├── package_2/
│   └── package_3/
├── build/
├── install/
└── log/
```

| 디렉토리 | 역할 |
|---|---|
| `src` | 개발 중인 Package 및 소스 코드 |
| `build` | 빌드 과정에서 생성되는 중간 파일 |
| `install` | 빌드 후 설치된 실행 파일 및 설정 파일 |
| `log` | 빌드 관련 로그 |

직접 작성하거나 수정하는 소스 코드는 `src` 디렉토리에서 관리한다.

### 15-2. Colcon

Colcon은 ROS 2 Package를 빌드할 때 사용하는 도구이다.

Workspace에 포함된 Package를 탐색하고 의존성을 고려하여 빌드한다.

전체 Package 빌드:

```bash
colcon build
```

특정 Package만 빌드:

```bash
colcon build --packages-select <package_name>
```

`colcon build`는 `src` 내부가 아닌 Workspace의 최상위 디렉토리에서 실행해야 한다.

### 15-3. Workspace 환경 설정

빌드가 완료되면 설치된 Package를 현재 터미널에서 사용할 수 있도록 환경 설정 파일을 불러온다.

```bash
source install/setup.bash
```

기본적인 작업 순서는 다음과 같다.

```text
소스 코드 작성 및 수정
          ↓
colcon build
          ↓
source install/setup.bash
          ↓
ros2 run
```

빌드 후에도 Package 또는 실행 파일을 찾지 못한다면 빌드 위치와 `source` 실행 여부를 확인해야 한다.

## 16. ROS 2 Package 생성

### 16-1. Package

Package는 특정 기능을 수행하는 소스 코드, 설정 파일, 의존성 정보 등을 하나로 묶은 ROS 2의 기본 관리 단위이다.

Package는 Workspace의 `src` 디렉토리 내부에 생성한다.

```bash
cd ~/robot_navigation/colcon_ws/src
```

Python 기반 Package 생성 명령어 예시:

```bash
ros2 pkg create pub_sub --build-type ament_python --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs
```

| 옵션 | 의미 |
|---|---|
| `--build-type ament_python` | Python 기반 Package 생성 |
| `--dependencies` | Package에서 사용할 의존성 지정 |
| `rclpy` | ROS 2 Python Client Library |
| `std_msgs` | 기본 Message Type |
| `geometry_msgs` | 위치 및 속도 관련 Message |
| `sensor_msgs` | 센서 관련 Message |
| `nav_msgs` | 내비게이션 관련 Message |

Package 이름은 소문자로 시작하고 소문자, 숫자, 밑줄 등을 사용하는 것이 일반적인 규칙이다.

### 16-2. Python Package 구조

```text
colcon_ws/
└── src/
    └── pub_sub/
        ├── pub_sub/
        │   ├── __init__.py
        │   ├── talker.py
        │   └── listener.py
        ├── resource/
        ├── test/
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

동일한 이름의 디렉토리가 두 번 등장하는 이유는 바깥쪽 디렉토리는 ROS 2 Package를 관리하고, 안쪽 디렉토리는 실제 Python Module을 관리하기 때문이다.

- `package.xml`: Package 정보 및 의존성 관리
- `setup.py`: Python Package 설치 및 실행 파일 등록
- `setup.cfg`: Python Package 관련 설정
- `talker.py`, `listener.py`: 실제 실행할 Python 코드

## 17. Python 기반 Publisher와 Subscriber

### 17-1. rclpy

ROS 2에서 Python을 이용해 Node를 작성할 때 사용하는 Client Library이다.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

ROS 2 Node를 생성할 때는 `Node` 클래스를 상속받아 사용할 수 있다.

### 17-2. Publisher

Publisher의 기본 동작 구조는 다음과 같다.

```text
Node 생성
    ↓
Publisher 생성
    ↓
Timer 생성
    ↓
Callback Function 실행
    ↓
Message 발행
```

Publisher의 주요 함수:

| 함수 | 역할 |
|---|---|
| `create_publisher()` | Publisher 생성 |
| `create_timer()` | 일정 주기로 Callback 실행 |
| `publish()` | Message 발행 |
| `get_logger().info()` | 로그 출력 |

Publisher 생성 예시:

```python
self.publisher = self.create_publisher(
    String,
    'chatter',
    10
)
```

- `String`: Message Type
- `'chatter'`: Topic 이름
- `10`: QoS Depth 설정

Timer를 생성하여 1초마다 Callback Function을 실행한다.

```python
self.timer = self.create_timer(1.0, self.publish_message)
```

Message 생성 및 발행:

```python
def publish_message(self):
    msg = String()
    msg.data = 'Hello World'

    self.publisher.publish(msg)
    self.get_logger().info(msg.data)
```

`String` Message는 내부의 `data` 필드에 실제 문자열을 저장한 후 발행한다.

### 17-3. Subscriber

Subscriber는 지정된 Topic에 Message가 발행되었을 때 Callback Function을 실행한다.

```text
Node 생성
    ↓
Subscriber 생성
    ↓
Message 수신
    ↓
Callback Function 실행
    ↓
데이터 처리 또는 출력
```

Subscriber 생성 예시:

```python
self.subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10
)
```

Callback Function:

```python
def listener_callback(self, msg):
    self.get_logger().info(msg.data)
```

Publisher는 Timer를 이용해 일정 주기로 Message를 발행하고, Subscriber는 Message가 수신될 때 Callback Function을 실행한다.

### 17-4. Node 실행 및 종료

```python
rclpy.init()

node = Talker()

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
```

| 함수 | 역할 |
|---|---|
| `rclpy.init()` | ROS 2 통신 환경 초기화 |
| `Talker()` | Node 객체 생성 |
| `rclpy.spin()` | Callback 처리 및 Node 실행 유지 |
| `destroy_node()` | Node 제거 및 자원 정리 |
| `rclpy.try_shutdown()` | ROS 2 통신 환경 종료 |

`spin()`은 Timer나 Message 수신 등의 이벤트가 발생했을 때 등록된 Callback Function을 처리할 수 있도록 Node의 실행을 유지한다.

## 18. setup.py 및 실행 파일 등록

### 18-1. Entry Point

Python 코드를 작성하는 것만으로는 `ros2 run` 명령어를 이용해 실행할 수 없다.

`setup.py`의 `entry_points`에 실행할 Python Module과 함수를 등록해야 한다.

```python
entry_points={
    'console_scripts': [
        'talker = pub_sub.talker:main',
        'listener = pub_sub.listener:main',
    ],
},
```

Entry Point의 기본 구조:

```text
실행 명령어 = 패키지명.파일명:함수명
```

예를 들어 다음과 같이 등록한 경우,

```python
'talker = pub_sub.talker:main'
```

`ros2 run pub_sub talker`를 실행하면 `pub_sub` Package의 `talker.py` 내부에 있는 `main()` 함수가 실행된다.

### 18-2. data_files

`setup.py`의 `data_files`는 빌드 과정에서 필요한 파일을 지정된 위치에 설치하도록 설정하는 항목이다.

이후 Launch 파일이나 설정 파일 등을 Package에 포함할 때 사용할 수 있다.

### 18-3. Package 빌드 및 실행

Workspace의 최상위 디렉토리로 이동한다.

```bash
cd ~/robot_navigation/colcon_ws
```

Package를 빌드하고 현재 터미널에 적용한다.

```bash
colcon build --packages-select pub_sub
source install/setup.bash
```

Terminal 1:

```bash
ros2 run pub_sub talker
```

Terminal 2:

```bash
ros2 run pub_sub listener
```

새로 연 터미널에서도 Workspace 환경이 적용되어 있어야 한다.

### 18-4. Package 실행 오류 확인

Package 또는 실행 파일을 찾지 못할 때 확인할 사항:

- Package가 `src` 디렉토리 내부에 존재하는지 확인
- Python 코드가 내부 Python Package 디렉토리에 존재하는지 확인
- `setup.py`의 `entry_points` 등록 내용 확인
- 수정한 파일이 저장되어 있는지 확인
- Workspace 최상위 디렉토리에서 빌드했는지 확인
- 빌드 후 `source install/setup.bash`를 실행했는지 확인

## 19. ROS 2 개발 과정 정리

```text
Workspace 생성
      ↓
src 내부에 Package 생성
      ↓
Python Node 작성
      ↓
Publisher / Subscriber 구성
      ↓
setup.py 실행 파일 등록
      ↓
colcon build
      ↓
source install/setup.bash
      ↓
ros2 run
      ↓
Topic 및 Message 확인
```

지난 실습에서는 MCU를 이용하여 센서와 액추에이터를 직접 제어했다면, 이번 ROS 2 실습에서는 각 기능을 Node 단위로 구성하고 Topic을 통해 데이터를 주고받는 구조를 학습했다.

센서 데이터 처리, 위치 추정, 경로 계획, 모터 제어 등을 각각의 Node로 구성하면 이후 로봇 내비게이션 시스템을 개발할 때 각 기능을 연결하고 관리할 수 있다.
