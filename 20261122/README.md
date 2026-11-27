# [2주차 - 11/22] ROS 2 Node 통신, Launch 및 Custom Interface

## 학습 내용

- Publisher, Processor, Subscriber를 이용한 Node 간 통신
- 키보드 입력을 이용한 2D 좌표 계산
- Timer와 Callback Function
- Launch 파일을 이용한 다중 Node 실행
- Service와 Action의 차이
- Custom Message 및 Custom Service 생성
- Service Server와 Client 구현
- Custom Service를 이용한 로봇 이동 좌표 계산
- CMakeLists.txt, package.xml, setup.py의 역할

## 1. Publisher와 Subscriber 복습

ROS 2에서는 각각의 기능을 Node 단위로 구성하고, Topic을 통해 데이터를 주고받는다.

Publisher는 Topic 이름과 Message Type을 지정하여 데이터를 발행하고, Subscriber는 동일한 Topic 이름과 Message Type을 사용하여 데이터를 수신한다.

```text
Publisher ────── Topic ──────> Subscriber
                 Message
```

### 1-1. Message 생성 및 발행

ROS 2에서 데이터를 발행하려면 단순히 Python 변수를 전달하는 것이 아니라 해당 Message Type의 객체를 생성해야 한다.

```python
from std_msgs.msg import String

msg = String()
msg.data = 'Hello World'

self.publisher.publish(msg)
```

- `String()`: ROS 2 String Message 객체 생성
- `msg.data`: Message 내부에 존재하는 실제 데이터 필드
- `publish(msg)`: 생성한 Message를 Topic에 발행

Python에서 사용하는 일반적인 문자열과 ROS 2의 String Message는 서로 다르다.

```python
text = 'Hello World'
msg = String()
msg.data = text
```

Message 내부에 다른 Message가 포함되어 있다면 계층 구조를 따라 접근해야 한다.

```python
msg.data
msg.position.x
msg.twist.linear.x
```

### 1-2. Node 이름과 Topic 이름

Node 이름, Python 클래스 이름, Topic 이름은 각각 다른 역할을 한다.

| 구분 | 역할 |
|---|---|
| Class Name | Python 내부의 클래스 이름 |
| Node Name | ROS 2에서 Node를 식별하는 이름 |
| Topic Name | 데이터를 주고받는 통신 채널 이름 |
| Message Type | 전달되는 데이터의 구조 |

```python
class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')
```

위 코드에서 `KeyboardPublisher`는 Python 클래스 이름이고, `keyboard_publisher`는 ROS 2에서 사용하는 Node 이름이다.

Publisher와 Subscriber 사이의 통신에서는 Topic 이름과 Message Type이 일치해야 한다.

## 2. Publisher, Processor, Subscriber 실습

기존 Talker와 Listener 실습에서는 두 개의 Node가 직접 통신했다.

이번 실습에서는 중간에 Processor Node를 추가하여 입력값을 처리하고, 처리된 결과를 다른 Node에 전달하는 구조를 구현한다.

### 2-1. 전체 구조

```text
Keyboard Input (W, A, S, D)
              ↓
       Publisher Node
              ↓
       Direction Topic
              ↓
       Processor Node
       현재 좌표 계산
              ↓
        Position Topic
              ↓
       Subscriber Node
       현재 좌표 출력
```

| Node | 역할 |
|---|---|
| Publisher | 키보드 입력을 받아 발행 |
| Processor | 입력 방향에 따라 현재 좌표 계산 |
| Subscriber | 계산된 좌표를 수신하고 출력 |

각 Node가 담당하는 기능을 구분하는 것이 중요하다.

Publisher에서는 좌표를 계산하지 않고 입력값만 전달한다. Processor는 전달받은 입력을 처리하며, Subscriber는 최종 결과를 출력한다.

### 2-2. 좌표계 설정

초기 로봇의 위치는 (0, 0)으로 설정한다.

키보드 입력에 따라 X축 또는 Y축 방향으로 한 단위씩 이동한다.

```text
                +Y
                 ↑
                 W
                 │
       A ───── (0,0) ───── D → +X
                 │
                 S
                 ↓
                -Y
```

| 입력 | 이동 방향 | 좌표 변화량 |
|---|---|---|
| W | 위쪽 | (0, +1) |
| A | 왼쪽 | (-1, 0) |
| S | 아래쪽 | (0, -1) |
| D | 오른쪽 | (+1, 0) |

현재 위치에 입력 방향에 따른 변화량을 더하여 새로운 위치를 계산한다.

**x_{new}=x_{current}+\Delta x**

**y_{new}=y_{current}+\Delta y**

초기 위치가 (0, 0)이고 W, W, D, S 순서로 입력한 경우 다음과 같이 좌표가 변한다.

| 입력 | X | Y |
|---|---:|---:|
| 초기 위치 | 0 | 0 |
| W | 0 | 1 |
| W | 0 | 2 |
| D | 1 | 2 |
| S | 1 | 1 |

## 3. Keyboard Publisher 구현

### 3-1. 키보드 입력 처리

Python의 `input()`을 이용하여 사용자의 키보드 입력을 받는다.

```python
direction = input('Enter direction (W/A/S/D): ').strip().lower()
```

- `input()`: 사용자가 입력하고 Enter를 누를 때까지 대기
- `strip()`: 입력값의 앞뒤 공백 제거
- `lower()`: 대문자를 소문자로 변환

W, A, S, D 이외의 값이 입력되는 경우도 고려해야 한다.

```python
if direction not in ['w', 'a', 's', 'd']:
    self.get_logger().warning('Invalid direction')
    return
```

잘못된 입력은 발행하지 않고 다시 입력받도록 처리한다.

### 3-2. Message 발행

키보드 입력값은 문자열이므로 `std_msgs/msg/String`을 사용한다.

```python
self.publisher = self.create_publisher(
    String,
    'direction',
    10
)
```

입력받은 방향을 Message에 저장하고 발행한다.

```python
msg = String()
msg.data = direction

self.publisher.publish(msg)
self.get_logger().info(f'Direction: {msg.data}')
```

Publisher에서는 방향 정보만 발행하며 현재 로봇의 위치는 관리하지 않는다.

### 3-3. Timer와 Callback Function

Timer는 일정 시간이 경과할 때마다 지정된 Callback Function을 실행하도록 설정하는 기능이다.

```python
self.timer = self.create_timer(
    0.1,
    self.publish_direction
)
```

`0.1`은 초 단위의 Timer 주기를 의미한다.

**f=\frac{1}{T}**

| Timer Period | Frequency |
|---|---|
| 1.0 sec | 1 Hz |
| 0.5 sec | 2 Hz |
| 0.1 sec | 10 Hz |

Timer의 주기가 0.1초라면 설정상 초당 10회 Callback 실행이 가능하다.

다만 `input()`은 사용자가 값을 입력할 때까지 실행을 기다리는 Blocking 함수이므로, 실제 키보드 입력 발행 주기가 항상 10Hz가 되는 것은 아니다.

### 3-4. spin()과 spin_once()

| 함수 | 역할 |
|---|---|
| `rclpy.spin()` | Node의 실행을 유지하면서 Callback을 계속 처리 |
| `rclpy.spin_once()` | 한 번의 실행에서 준비된 Callback을 처리 |
| `rclpy.ok()` | ROS 2 Context가 정상적으로 동작 중인지 확인 |

Timer와 `spin()`을 사용하는 방식 외에도 `while` 반복문과 `spin_once()`를 이용하여 입력과 발행을 처리할 수 있다.

```python
while rclpy.ok():
    node.publish_direction()
    rclpy.spin_once(node, timeout_sec=0)
```

## 4. Processor Node 구현

### 4-1. Publisher와 Subscriber를 동시에 사용하는 Node

Processor는 Publisher에서 발행한 방향 정보를 구독하고, 계산된 좌표를 다시 발행해야 한다.

따라서 하나의 Node 안에 Publisher와 Subscriber가 모두 존재한다.

```text
Direction Topic
       ↓
  Subscriber
       ↓
  Processor Node
  좌표 계산 및 갱신
       ↓
   Publisher
       ↓
Position Topic
```

하나의 Node가 반드시 Publisher 또는 Subscriber 중 하나의 기능만 수행해야 하는 것은 아니다.

```python
self.subscription = self.create_subscription(
    String,
    'direction',
    self.direction_callback,
    10
)

self.publisher = self.create_publisher(
    Int32MultiArray,
    'position',
    10
)
```

입력 Topic은 `String`을 사용하고 출력 Topic은 X, Y 좌표를 함께 전달하기 위해 `Int32MultiArray`를 사용한다.

### 4-2. 현재 위치 관리

현재 위치를 관리하는 방법으로 Python의 리스트, 개별 변수, NumPy 배열 등을 사용할 수 있다.

실습에서는 NumPy를 이용하여 현재 위치와 이동 방향을 배열로 관리하는 방법을 다뤘다.

```python
import numpy as np

self.location = np.array([0, 0])

self.direction_map = {
    'w': np.array([0, 1]),
    'a': np.array([-1, 0]),
    's': np.array([0, -1]),
    'd': np.array([1, 0])
}
```

Dictionary를 사용하면 입력값에 해당하는 좌표 변화량을 바로 가져올 수 있다.

### 4-3. 좌표 계산

방향 정보를 수신하면 Subscriber의 Callback Function이 실행된다.

```python
def direction_callback(self, msg):
    direction = msg.data.lower()

    if direction in self.direction_map:
        self.location += self.direction_map[direction]

        self.get_logger().info(
            f'Updated position: {self.location}'
        )
```

현재 위치에 이동 방향의 좌표 변화량을 더하여 새로운 위치를 계산한다.

```text
현재 위치: [1, 2]
입력 방향: d
변화량:   [1, 0]

[1, 2] + [1, 0] = [2, 2]
```

### 4-4. MultiArray Message

X, Y 좌표를 하나의 Message로 전달하기 위해 MultiArray를 사용한다.

```python
from std_msgs.msg import Int32MultiArray
```

| Message Type | 데이터 |
|---|---|
| `Int16MultiArray` | 16비트 부호 있는 정수 배열 |
| `Int32MultiArray` | 32비트 부호 있는 정수 배열 |
| `Int64MultiArray` | 64비트 부호 있는 정수 배열 |
| `Float64MultiArray` | 64비트 실수 배열 |

이번 실습에서는 음수 좌표도 발생할 수 있으므로 부호 있는 정수형을 사용하는 것이 적합하다.

### 4-5. 계산 결과 발행

```python
def publish_position(self):
    msg = Int32MultiArray()
    msg.data = self.location.tolist()

    self.publisher.publish(msg)
```

NumPy 배열을 리스트로 변환하여 Message의 `data` 필드에 저장한다.

Processor에서는 Timer를 이용하여 현재 위치를 일정 주기로 발행할 수 있다.

```python
self.timer = self.create_timer(
    0.5,
    self.publish_position
)
```

키보드 입력을 수신했을 때만 위치를 갱신하고, Timer를 통해 현재 위치를 반복적으로 발행하는 구조이다.

## 5. Subscriber Node 구현

Subscriber는 Processor에서 발행하는 좌표 정보를 수신하고 현재 위치를 출력한다.

```python
self.subscription = self.create_subscription(
    Int32MultiArray,
    'position',
    self.position_callback,
    10
)
```

수신한 Message에는 X, Y 좌표가 리스트 형태로 저장되어 있다.

```python
def position_callback(self, msg):
    x, y = msg.data

    self.get_logger().info(
        f'Current Position: ({x}, {y})'
    )
```

`x, y = msg.data`는 리스트에 포함된 두 개의 값을 각각의 변수에 저장하는 Unpacking 방식이다.

### 5-1. Logging

Node에서 어떤 작업이 수행되고 있는지 확인하기 위해 로그를 사용한다.

| 함수 | 용도 |
|---|---|
| `get_logger().info()` | 정상적인 동작 정보 |
| `get_logger().warning()` | 잘못된 입력 및 주의가 필요한 상태 |
| `get_logger().error()` | 실행 중 발생한 오류 |

단순히 동작 여부만 출력하는 것보다 입력값, 처리 결과, 오류 원인 등 필요한 정보를 함께 출력하는 것이 디버깅에 도움이 된다.

### 5-2. Package 구성

이전 실습에서 생성한 Publisher/Subscriber 관련 Package에 새로운 Executable을 추가한다.

```text
colcon_ws/
└── src/
    └── pub_sub/
        ├── pub_sub/
        │   ├── __init__.py
        │   ├── talker.py
        │   ├── listener.py
        │   ├── publisher.py
        │   ├── processor.py
        │   └── subscriber.py
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

서로 관련된 기능을 수행하는 Node는 동일한 Package 안에서 관리할 수 있다.

### 5-3. Entry Point 등록

새로운 Python 파일을 `ros2 run`으로 실행하려면 `setup.py`의 `console_scripts`에 등록해야 한다.

```python
entry_points={
    'console_scripts': [
        'talker = pub_sub.talker:main',
        'listener = pub_sub.listener:main',
        'publisher = pub_sub.publisher:main',
        'processor = pub_sub.processor:main',
        'subscriber = pub_sub.subscriber:main',
    ],
},
```

기존 Talker와 Listener는 유지하고 새롭게 작성한 Node를 추가한다.

Workspace의 최상위 디렉토리에서 빌드한다.

```bash
colcon build --packages-select pub_sub
source install/setup.bash
```

각각의 터미널에서 다음 명령어를 실행한다.

```bash
ros2 run pub_sub publisher
```

```bash
ros2 run pub_sub processor
```

```bash
ros2 run pub_sub subscriber
```

세 개의 Node가 실행된 상태에서 키보드 입력에 따라 위치 정보가 전달되는지 확인한다.

```bash
ros2 node list
ros2 topic list
ros2 topic echo /position
rqt_graph
```

## 6. Launch 파일

### 6-1. Launch 파일을 사용하는 이유

여러 개의 Node를 실행하려면 각각의 터미널에서 `ros2 run` 명령어를 입력해야 한다.

로봇 시스템의 규모가 커지면 실행해야 하는 Node가 많아지고, 각 Node의 실행 조건과 설정을 관리하기도 어려워진다.

Launch 파일을 사용하면 여러 Node와 프로세스를 하나의 명령어로 실행할 수 있다.

```text
기존 방식

Terminal 1 → Publisher
Terminal 2 → Processor
Terminal 3 → Subscriber

Launch 사용

ros2 launch
     ├── Publisher
     ├── Processor
     └── Subscriber
```

### 6-2. Launch 파일 형식

ROS 2에서는 Python, XML, YAML 등을 이용하여 Launch 파일을 작성할 수 있다.

이번 실습에서는 조건문, 반복문 등의 Python 기능을 활용할 수 있는 Python Launch 파일을 사용한다.

파일명은 일반적으로 다음과 같이 작성한다.

```text
my_location.launch.py
```

`.launch.py` 형식을 사용하면 일반적인 Python 소스 코드와 Launch 파일을 쉽게 구분할 수 있다.

### 6-3. Launch 파일 위치

Launch 파일은 Package의 최상위 디렉토리에 `launch` 폴더를 생성하여 관리한다.

```text
pub_sub/
├── pub_sub/
│   ├── publisher.py
│   ├── processor.py
│   └── subscriber.py
├── launch/
│   └── my_location.launch.py
├── package.xml
├── setup.cfg
└── setup.py
```

Python 소스 코드가 위치하는 내부 `pub_sub` 폴더가 아니라, `setup.py`와 동일한 위치에 `launch` 폴더를 생성한다.

### 6-4. LaunchDescription과 ExecuteProcess

Python Launch 파일에서는 `generate_launch_description()` 함수를 사용하여 실행할 작업을 정의한다.

실습에서 다룬 `ExecuteProcess` 기반 구조를 간단히 나타내면 다음과 같다.

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    publisher = ExecuteProcess(
        cmd=['ros2', 'run', 'pub_sub', 'publisher'],
        output='screen'
    )

    processor = ExecuteProcess(
        cmd=['ros2', 'run', 'pub_sub', 'processor'],
        output='screen'
    )

    subscriber = ExecuteProcess(
        cmd=['ros2', 'run', 'pub_sub', 'subscriber'],
        output='screen'
    )

    return LaunchDescription([
        publisher,
        processor,
        subscriber
    ])
```

| 구분 | 역할 |
|---|---|
| `generate_launch_description()` | Launch에서 실행할 작업 정의 |
| `LaunchDescription` | 실행할 Action들을 묶어서 관리 |
| `ExecuteProcess` | 지정한 명령어를 프로세스로 실행 |
| `cmd` | 실제 실행할 명령어 |
| `output='screen'` | 실행 결과를 화면에 출력 |

`generate_launch_description()`은 ROS 2 Launch 시스템에서 사용하는 함수 이름이므로 임의로 변경하지 않는다.

### 6-5. Event Handler를 이용한 실행 순서 설정

Node의 실행 순서가 중요한 경우 Event Handler를 사용할 수 있다.

`OnProcessStart`는 특정 프로세스가 시작되었을 때 다른 작업을 실행하는 이벤트이다.

```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
```

```python
return LaunchDescription([
    publisher,

    RegisterEventHandler(
        OnProcessStart(
            target_action=publisher,
            on_start=[processor]
        )
    ),

    RegisterEventHandler(
        OnProcessStart(
            target_action=processor,
            on_start=[subscriber]
        )
    )
])
```

실행 순서는 다음과 같다.

```text
Publisher 실행
      ↓
OnProcessStart
      ↓
Processor 실행
      ↓
OnProcessStart
      ↓
Subscriber 실행
```

단순히 여러 Node를 실행하는 경우에는 Event Handler를 사용하지 않아도 된다.

또한 프로세스가 시작되었다는 것과 해당 Node의 통신 준비가 모두 완료되었다는 것은 서로 다르므로 구분해야 한다.

## 7. Launch 파일 등록 및 실행

### 7-1. setup.py의 data_files

Launch 파일을 생성하는 것만으로는 `ros2 launch` 명령어를 통해 실행할 수 없다.

빌드 과정에서 Launch 파일이 설치 경로에 포함되도록 `setup.py`의 `data_files`에 추가해야 한다.

```python
data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name]
    ),
    (
        'share/' + package_name,
        ['package.xml']
    ),
    (
        'share/' + package_name + '/launch',
        ['launch/my_location.launch.py']
    ),
],
```

`data_files`는 여러 Tuple로 구성된 리스트이다.

각 Tuple은 다음 구조를 가진다.

```python
(설치할_위치, 설치할_파일_목록)
```

예를 들어 다음 코드는

```python
(
    'share/' + package_name + '/launch',
    ['launch/my_location.launch.py']
)
```

소스 디렉토리의 Launch 파일을 패키지 설치 경로의 `share/<package_name>/launch`에 설치하도록 설정한 것이다.

Launch 파일이 여러 개인 경우 파일 목록을 추가하거나 `glob()`을 사용할 수 있다.

```python
from glob import glob
import os
```

```python
(
    os.path.join('share', package_name, 'launch'),
    glob('launch/*.launch.py')
)
```

### 7-2. entry_points와 data_files의 차이

| 구분 | entry_points | data_files |
|---|---|---|
| 역할 | Python 실행 명령어 등록 | 추가 파일 설치 |
| 대상 | Node의 실행 함수 | Launch, 설정 파일 등 |
| 실행 방식 | `ros2 run` | `ros2 launch` 등 |

Launch 파일은 `console_scripts`가 아닌 `data_files`에 등록한다.

### 7-3. Launch 파일 실행

```bash
colcon build --packages-select pub_sub
source install/setup.bash
```

```bash
ros2 launch pub_sub my_location.launch.py
```

명령어의 차이를 구분해야 한다.

```bash
ros2 run <package_name> <executable_name>
```

```bash
ros2 launch <package_name> <launch_file_name>
```

- `ros2 run`: 등록된 Executable 실행
- `ros2 launch`: 설치된 Launch 파일 실행

키보드 `input()`을 사용하는 Node는 Launch로 실행했을 때 표준 입력 전달 방식에 따라 입력이 제대로 동작하지 않을 수 있다. 이 경우 Publisher를 별도의 터미널에서 실행하고 나머지 Node를 Launch로 실행하여 확인할 수 있다.

## 8. Service와 Action

### 8-1. Service

Service는 Client가 특정 작업을 요청하면 Server가 해당 요청을 처리한 후 결과를 반환하는 통신 방식이다.

```text
Client Node                     Server Node
     │                               │
     ├──────── Request ─────────────>│
     │                               │
     │                          요청 처리
     │                               │
     │<─────── Response ─────────────┤
     │                               │
```

- **Client**: Service를 요청하는 Node
- **Server**: Service 요청을 받아 처리하는 Node
- **Request**: Client가 전달하는 요청 데이터
- **Response**: Server가 처리 후 반환하는 데이터

Service는 센서 값 요청, 상태 조회, 로봇 초기화 등 요청에 대한 결과를 받을 때 사용한다.

이전에 사용한 Turtlesim의 `reset`, `spawn`, `kill`, `teleport` 등도 Service 방식으로 동작한다.

### 8-2. Action

Action은 비교적 긴 시간이 필요한 작업을 수행할 때 사용한다.

```text
Action Client                 Action Server
      │                             │
      ├──────── Goal ──────────────>│
      │                             │
      │<────── Feedback ────────────┤
      │<────── Feedback ────────────┤
      │<────── Feedback ────────────┤
      │                             │
      │<─────── Result ─────────────┤
```

Action은 작업을 수행하는 동안 진행 상황을 Feedback으로 전달할 수 있으며 작업 취소도 가능하다.

대표적인 예시는 다음과 같다.

- 로봇을 특정 목적지까지 이동
- 로봇 팔을 이용하여 물체 이동
- 일정 시간이 필요한 작업 수행

### 8-3. Topic, Service, Action 비교

| 구분 | Topic | Service | Action |
|---|---|---|---|
| 통신 구조 | Publish/Subscribe | Request/Response | Goal/Feedback/Result |
| 주요 용도 | 지속적인 데이터 전달 | 요청 및 응답 | 장시간 작업 수행 |
| 응답 | 별도의 응답 구조 없음 | Response 반환 | Feedback 및 Result 반환 |
| 작업 취소 | 별도 기능 없음 | 기본 취소 기능 없음 | 취소 가능 |
| 예시 | LiDAR, IMU 데이터 | 상태 조회, 초기화 | 목적지까지 이동 |
| 인터페이스 | `.msg` | `.srv` | `.action` |

이번 실습에서는 Service를 직접 구현하고 Action은 개념 위주로 다뤘다.

## 9. Custom Interface

### 9-1. Custom Interface를 사용하는 이유

ROS 2에는 `std_msgs`, `geometry_msgs`, `sensor_msgs` 등 다양한 Message Type이 정의되어 있다.

하지만 프로젝트에서 필요한 데이터 구조가 기존 Message Type과 맞지 않는 경우 직접 새로운 Interface를 정의할 수 있다.

| 파일 형식 | 역할 |
|---|---|
| `.msg` | Custom Message 정의 |
| `.srv` | Custom Service 정의 |
| `.action` | Custom Action 정의 |

Custom Interface를 별도의 Package로 관리하면 여러 Package에서 동일한 Message와 Service Type을 재사용할 수 있다.

### 9-2. Interface Package와 실행 Package

이번 실습에서는 Interface를 정의하는 Package와 실제 Node를 실행하는 Package를 구분한다.

| Package | Build Type | 역할 |
|---|---|---|
| `pub_sub` | ament_python | Publisher/Subscriber 실습 |
| `srv_msg` | ament_python | Service Server 및 Client 구현 |
| `custom_interfaces` | ament_cmake | Custom Message 및 Service 정의 |

`custom_interfaces`는 실제 로봇을 제어하는 소스 코드가 아니라 통신에 사용할 데이터 구조를 관리한다.

```text
custom_interfaces
       │
       │ Message / Service Type 제공
       ↓
    srv_msg
       ├── Server
       └── Client
```

### 9-3. Package 생성

Package를 생성할 때는 Workspace의 `src` 디렉토리로 이동한다.

```bash
cd ~/robot_navigation/colcon_ws/src
```

Python 기반의 Service 실습 Package 생성 예시:

```bash
ros2 pkg create srv_msg \
    --build-type ament_python \
    --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs
```

Custom Interface Package 생성:

```bash
ros2 pkg create custom_interfaces \
    --build-type ament_cmake
```

Custom Interface Package는 이번 실습에서 `ament_cmake` 기반으로 생성한다.

Package를 잘못 생성하여 `CMakeLists.txt`가 존재하지 않는 경우 Build Type을 확인해야 한다.

### 9-4. Interface 디렉토리 생성

```bash
cd custom_interfaces
mkdir -p msg srv
```

```text
custom_interfaces/
├── msg/
├── srv/
├── CMakeLists.txt
└── package.xml
```

- `msg`: Message Type 정의
- `srv`: Service Type 정의
- `CMakeLists.txt`: Interface 생성 및 빌드 설정
- `package.xml`: Package 정보 및 의존성 정의

Message와 Service 파일명은 `RobotPose.msg`, `MoveRobot.srv`처럼 PascalCase로 작성한다.

## 10. Custom Message 생성

### 10-1. Num.msg

정수 데이터를 전달하는 새로운 Message Type을 생성한다.

파일 위치:

```text
custom_interfaces/msg/Num.msg
```

파일 내용:

```text
int64 mynum
```

Message 정의는 다음 구조를 가진다.

```text
데이터_타입 변수명
```

- `int64`: 64비트 부호 있는 정수
- `mynum`: 실제 데이터가 저장되는 변수명

기존 `std_msgs/msg/Int64`는 `data`라는 필드를 사용하지만 직접 정의한 `Num.msg`에서는 `mynum`이라는 필드를 사용한다.

### 10-2. Python에서 Custom Message 사용

Interface를 생성하고 빌드한 후 Python 코드에서 Import할 수 있다.

```python
from custom_interfaces.msg import Num

msg = Num()
msg.mynum = 10
```

Custom Message 역시 기존 ROS 2 Message와 동일하게 객체를 생성한 후 내부 필드에 접근하여 값을 설정한다.

## 11. Custom Service 생성

### 11-1. AddTwoInts.srv

두 개의 정수를 입력받아 합을 반환하는 Service를 정의한다.

파일 위치:

```text
custom_interfaces/srv/AddTwoInts.srv
```

파일 내용:

```text
custom_interfaces/Num a
custom_interfaces/Num b
---
custom_interfaces/Num sum
```

`---`를 기준으로 Request와 Response를 구분한다.

```text
Request
├── a
└── b
────────────
Response
└── sum
```

`a`, `b`, `sum`은 단순한 정수 변수가 아니라 앞서 생성한 `Num` Message Type이다.

따라서 실제 정수값에 접근하려면 내부의 `mynum` 필드까지 접근해야 한다.

```python
request.a.mynum
request.b.mynum
response.sum.mynum
```

### 11-2. Message의 계층 구조

```text
AddTwoInts.srv
├── Request
│   ├── a
│   │   └── mynum
│   └── b
│       └── mynum
└── Response
    └── sum
        └── mynum
```

이미 정의한 Message Type을 다른 Message나 Service에서 재사용할 수 있다.

## 12. Custom Interface 빌드 설정

### 12-1. CMakeLists.txt

Custom Interface를 정의한 후에는 ROS 2에서 사용할 수 있도록 Interface 생성 과정을 빌드에 포함해야 한다.

`custom_interfaces/CMakeLists.txt`의 주요 구성은 다음과 같다.

```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Num.msg"
    "srv/AddTwoInts.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

| 구문 | 역할 |
|---|---|
| `project()` | 프로젝트 및 Package 이름 설정 |
| `find_package()` | 빌드에 필요한 Package 검색 |
| `rosidl_generate_interfaces()` | Message와 Service Interface 생성 |
| `ament_export_dependencies()` | Interface 사용에 필요한 의존성 정보 제공 |
| `ament_package()` | ament Package 설정 |

새로운 Message 또는 Service 파일을 생성했다면 `rosidl_generate_interfaces()`에 해당 파일을 추가해야 한다.

파일을 생성했더라도 CMakeLists.txt에 등록하지 않으면 Interface가 생성되지 않는다.

### 12-2. package.xml

`custom_interfaces/package.xml`에 Interface 생성 및 사용에 필요한 의존성을 추가한다.

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

- `rosidl_default_generators`: Interface를 생성하는 데 필요한 도구
- `rosidl_default_runtime`: 생성된 Interface를 실행 환경에서 사용하기 위한 의존성
- `rosidl_interface_packages`: Interface Package임을 나타내는 그룹

### 12-3. Interface를 사용하는 Package의 의존성 설정

`srv_msg/package.xml`에는 `custom_interfaces`를 의존성으로 추가한다.

```xml
<depend>custom_interfaces</depend>
```

이를 통해 Service 실습 Package가 Custom Interface Package에 의존한다는 것을 명시한다.

Python에서 Interface를 사용하려면 해당 Package가 정상적으로 빌드되고 현재 실행 환경에 적용되어 있어야 한다.

### 12-4. 빌드 및 확인

Workspace의 최상위 디렉토리에서 실행한다.

```bash
cd ~/robot_navigation/colcon_ws

colcon build
source install/setup.bash
```

Interface 생성 여부 확인:

```bash
ros2 interface show custom_interfaces/msg/Num
```

```bash
ros2 interface show custom_interfaces/srv/AddTwoInts
```

Interface가 정상적으로 생성되면 정의한 Message 및 Service 구조를 확인할 수 있다.

## 13. Service Server 구현

### 13-1. Server의 역할

Server는 Client의 요청을 기다리다가 요청이 들어오면 Callback Function을 실행한다.

두 개의 정수를 더하는 Service에서는 Server가 실제 덧셈 연산을 담당한다.

```text
Request
a = 2
b = 7
     ↓
Service Server
sum = a + b
     ↓
Response
sum = 9
```

### 13-2. Service 생성

```python
from custom_interfaces.srv import AddTwoInts
```

```python
self.service = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.add_callback
)
```

| Parameter | 역할 |
|---|---|
| `AddTwoInts` | Service Type |
| `add_two_ints` | Service 이름 |
| `self.add_callback` | 요청 수신 시 실행할 함수 |

Service Type과 Service 이름은 서로 다른 개념이다.

Client와 Server는 동일한 Service 이름과 Service Type을 사용해야 한다.

### 13-3. Server Callback Function

```python
def add_callback(self, request, response):

    response.sum.mynum = (
        request.a.mynum + request.b.mynum
    )

    self.get_logger().info(
        f'{request.a.mynum} + '
        f'{request.b.mynum} = '
        f'{response.sum.mynum}'
    )

    return response
```

Request에 포함된 `a`, `b`의 값을 읽어 덧셈을 수행한 후 Response에 결과를 저장한다.

Callback Function에서 반환한 Response가 Client에 전달된다.

## 14. Service Client 구현

### 14-1. Client 생성

```python
self.client = self.create_client(
    AddTwoInts,
    'add_two_ints'
)
```

Client를 생성할 때 Server에서 정의한 Service Type과 이름을 동일하게 사용한다.

### 14-2. Server 연결 확인

Server가 실행되지 않은 상태에서 Service를 요청하면 정상적으로 처리할 수 없다.

따라서 Service를 사용할 수 있는 상태인지 먼저 확인한다.

```python
while not self.client.wait_for_service(timeout_sec=1.0):
    self.get_logger().info(
        'Waiting for service...'
    )
```

`wait_for_service()`는 지정된 시간 동안 해당 Service를 사용할 수 있는지 확인한다.

### 14-3. Request 생성

```python
request = AddTwoInts.Request()

request.a.mynum = int(a)
request.b.mynum = int(b)
```

`.srv` 파일의 `---` 위쪽에 정의된 데이터가 Request에 해당한다.

Message 객체를 생성하고 내부 변수에 접근하는 방식은 Publisher 실습과 동일하다.

### 14-4. call_async()와 Future

Service 요청은 `call_async()`를 통해 비동기 방식으로 전달할 수 있다.

```python
future = self.client.call_async(request)
```

Future는 아직 완료되지 않은 작업의 결과를 나중에 확인하기 위한 객체이다.

```python
rclpy.spin_until_future_complete(self, future)
```

Server의 응답을 받을 때까지 Callback을 처리하며 기다린다.

```python
response = future.result()
```

응답을 정상적으로 받았다면 다음과 같이 결과에 접근할 수 있다.

```python
self.get_logger().info(
    f'Result: {response.sum.mynum}'
)
```

실제 구현에서는 Service 요청 실패나 예외 발생 여부도 확인해야 한다.

### 14-5. Command Line Argument

Client 실행 시 터미널에서 계산할 값을 함께 입력할 수 있다.

```bash
ros2 run srv_msg client 2 7
```

Python에서는 `sys.argv`를 사용하여 실행 시 입력한 Argument에 접근한다.

```python
import sys

a = int(sys.argv[1])
b = int(sys.argv[2])
```

| Index | 값 |
|---|---|
| `sys.argv[0]` | 실행 프로그램 이름 또는 경로 |
| `sys.argv[1]` | 2 |
| `sys.argv[2]` | 7 |

첫 번째 사용자 입력값은 `sys.argv[1]`부터 시작한다.

입력값이 부족하거나 정수로 변환할 수 없는 경우를 고려하여 예외 처리가 필요하다.

### 14-6. Service 실행

Server와 Client를 작성한 후 `setup.py`에 Executable을 등록한다.

```python
entry_points={
    'console_scripts': [
        'server = srv_msg.server:main',
        'client = srv_msg.client:main',
    ],
},
```

빌드 및 환경 설정:

```bash
colcon build
source install/setup.bash
```

Terminal 1 - Server:

```bash
ros2 run srv_msg server
```

Terminal 2 - Client:

```bash
ros2 run srv_msg client 2 7
```

강의에서 다룬 요청 및 응답 예시는 다음과 같다.

```text
Request:  2, 7
Response: 9
```

## 15. Custom Service를 이용한 로봇 이동 계산

두 정수의 덧셈을 수행하는 Service를 바탕으로 가상의 2D 평면에서 로봇의 이동 후 위치를 계산하는 Service를 구성한다.

### 15-1. 2D Robot Pose

2D 평면에서 로봇의 Pose는 X, Y 좌표와 방향 각도 theta로 나타낼 수 있다.

**Pose=(x,y,\theta)**

- x: X축 위치
- y: Y축 위치
- theta: 로봇이 바라보는 방향

```text
Y
↑
│                 Robot Heading
│                     ↗
│                   / θ
│               (x, y) ──────
│
└────────────────────────────→ X
```

앞선 키보드 이동 실습에서는 X, Y 위치만 관리했다면 이번 실습에서는 로봇의 방향까지 고려한다.

### 15-2. RobotPose.msg

2D 로봇의 Pose를 나타내는 Custom Message를 정의한다.

파일 위치:

```text
custom_interfaces/msg/RobotPose.msg
```

파일 내용:

```text
float64 x
float64 y
float64 theta
```

위치와 각도는 소수점을 포함할 수 있으므로 정수형이 아닌 `float64`를 사용한다.

### 15-3. MoveRobot.srv

현재 Pose와 이동 거리를 입력받아 이동 후 Pose를 반환하는 Service이다.

파일 위치:

```text
custom_interfaces/srv/MoveRobot.srv
```

인터페이스 구성 예시:

```text
custom_interfaces/RobotPose current_pose
float64 distance
---
custom_interfaces/RobotPose new_pose
bool success
```

| 구분 | 필드 | 의미 |
|---|---|---|
| Request | `current_pose` | 현재 로봇의 위치 및 방향 |
| Request | `distance` | 이동할 거리 |
| Response | `new_pose` | 이동 후 로봇의 위치 및 방향 |
| Response | `success` | 연산 또는 이동 성공 여부 |

이번 실습에서는 로봇이 기존 방향을 유지한 상태에서 지정된 거리만큼 직선 이동한다고 가정한다.

### 15-4. 이동 좌표 계산

현재 위치와 방향, 이동 거리를 이용하여 이동 후 좌표를 계산한다.

```text
Y
↑
│                       ● New Pose
│                     / │
│                   /   │ Δy
│                 / d   │
│               / θ     │
│ Current Pose ●────────┘
│                  Δx
└──────────────────────────────→ X
```

이동 거리를 d라고 하면 X축과 Y축 방향의 변화량은 다음과 같다.

**\Delta x=d\cos\theta**


**\Delta y=d\sin\theta**

현재 좌표에 각 방향의 변화량을 더하면 이동 후 좌표를 얻을 수 있다.

**x_{new}=x_{current}+d\cos\theta**


**y_{new}=y_{current}+d\sin\theta**


**\theta_{new}=\theta_{current}**

직선 이동만 고려하기 때문에 이동 전후의 방향은 동일하다.

### 15-5. Degree와 Radian

Python의 `math.sin()`과 `math.cos()`는 Radian을 입력값으로 사용한다.

각도를 Degree 단위로 입력받는 경우 Radian으로 변환해야 한다.


**\theta_{rad}=\theta_{deg}\frac{\pi}{180}**

```python
import math

theta_rad = math.radians(theta)
```

예를 들어 30° 방향으로 이동한다면 다음과 같이 변환된다.


**30^\circ=\frac{\pi}{6}\ rad**

### 15-6. 좌표 계산 예시

현재 로봇의 상태가 다음과 같다고 가정한다.

```text
Current X     : 1.2
Current Y     : 0.8
Current Theta : 30.0°
Distance      : 1.2
```

X축 방향 변화량:


**\Delta x=1.2\cos30^\circ**


**\Delta x\approx1.0392**

Y축 방향 변화량:


**\Delta y=1.2\sin30^\circ=0.6**

따라서 이동 후 좌표는 다음과 같다.


**x_{new}=1.2+1.0392=2.2392**


**y_{new}=0.8+0.6=1.4**

| 구분 | 이동 전 | 이동 후 |
|---|---:|---:|
| X | 1.2 | 2.2392 |
| Y | 0.8 | 1.4 |
| Theta | 30.0° | 30.0° |

### 15-7. Move Robot Server

Server에서는 Request에 포함된 현재 Pose와 이동 거리를 읽어 새로운 Pose를 계산한다.

좌표 계산을 구현하는 예시는 다음과 같다.

```python
def move_robot_callback(self, request, response):

    x = request.current_pose.x
    y = request.current_pose.y
    theta = request.current_pose.theta
    distance = request.distance

    theta_rad = math.radians(theta)

    response.new_pose.x = (
        x + distance * math.cos(theta_rad)
    )

    response.new_pose.y = (
        y + distance * math.sin(theta_rad)
    )

    response.new_pose.theta = theta
    response.success = True

    return response
```

이번 실습은 실제 모터를 움직이는 것이 아니라 입력값으로부터 이동 후 좌표를 계산하는 방식이다.

장애물이나 충돌 여부는 고려하지 않으며 정상적으로 좌표를 계산한 경우 `success`를 `True`로 설정한다.

### 15-8. Move Robot Client

Client는 X, Y, Theta, Distance 총 네 개의 값을 입력받아 Server에 전달한다.

```python
request = MoveRobot.Request()

request.current_pose.x = float(sys.argv[1])
request.current_pose.y = float(sys.argv[2])
request.current_pose.theta = float(sys.argv[3])
request.distance = float(sys.argv[4])
```

Server에서 반환한 데이터는 다음과 같이 접근할 수 있다.

```python
response.new_pose.x
response.new_pose.y
response.new_pose.theta
response.success
```

## 16. RobotPose 및 MoveRobot Interface 등록

새로운 Message와 Service를 생성했으므로 `CMakeLists.txt`에 추가한다.

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Num.msg"
    "msg/RobotPose.msg"
    "srv/AddTwoInts.srv"
    "srv/MoveRobot.srv"
)
```

Service 실습 Package에는 새로운 Server와 Client 파일을 추가한다.

```text
colcon_ws/
└── src/
    ├── pub_sub/
    │   ├── pub_sub/
    │   │   ├── talker.py
    │   │   ├── listener.py
    │   │   ├── publisher.py
    │   │   ├── processor.py
    │   │   └── subscriber.py
    │   ├── launch/
    │   │   └── my_location.launch.py
    │   ├── package.xml
    │   └── setup.py
    │
    ├── custom_interfaces/
    │   ├── msg/
    │   │   ├── Num.msg
    │   │   └── RobotPose.msg
    │   ├── srv/
    │   │   ├── AddTwoInts.srv
    │   │   └── MoveRobot.srv
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── srv_msg/
        ├── srv_msg/
        │   ├── server.py
        │   ├── client.py
        │   ├── move_robot_server.py
        │   └── move_robot_client.py
        ├── package.xml
        └── setup.py
```

### 16-1. Executable 등록

`srv_msg/setup.py`의 Entry Point에 추가한다.

```python
entry_points={
    'console_scripts': [
        'server = srv_msg.server:main',
        'client = srv_msg.client:main',
        'move_robot_server = srv_msg.move_robot_server:main',
        'move_robot_client = srv_msg.move_robot_client:main',
    ],
},
```

### 16-2. 빌드 및 실행

```bash
colcon build
source install/setup.bash
```

Terminal 1:

```bash
ros2 run srv_msg move_robot_server
```

Terminal 2:

```bash
ros2 run srv_msg move_robot_client 1.2 0.8 30.0 1.2
```

정상적으로 계산되는 경우 예상 결과는 다음과 같다.

```text
New Position X : 2.2392
New Position Y : 1.4000
New Theta      : 30.0
Success        : True
```

Service 구조 및 실행 상태는 다음 명령어를 통해 확인할 수 있다.

```bash
ros2 interface show custom_interfaces/srv/MoveRobot
```

```bash
ros2 service list
ros2 node list
```

## 17. ROS 2 Package 설정 파일 비교

이번 실습에서 사용한 설정 파일들은 각각 다른 역할을 수행한다.

| 파일 | 주요 역할 |
|---|---|
| `setup.py` | Python Package 설치, Executable 및 추가 파일 등록 |
| `setup.cfg` | Python 실행 스크립트의 설치 경로 설정 |
| `package.xml` | Package 정보와 의존성 정의 |
| `CMakeLists.txt` | CMake 기반 빌드 및 Interface 생성 설정 |
| `.msg` | Message 데이터 구조 정의 |
| `.srv` | Service의 Request와 Response 구조 정의 |
| `.launch.py` | 여러 Node 및 프로세스 실행 설정 |

### 17-1. ament_python과 ament_cmake

| 구분 | ament_python | ament_cmake |
|---|---|---|
| 빌드 기반 | Python setuptools | CMake |
| 주요 설정 파일 | setup.py | CMakeLists.txt |
| 실습 활용 | Python Node 구현 | Custom Interface 생성 |
| 공통 파일 | package.xml | package.xml |

Custom Interface는 `ament_cmake` Package에서 생성하고, 실제 Interface를 사용하는 Python Node는 `ament_python` Package에서 구현하는 구조를 사용했다.

### 17-2. Package 개발 순서

```text
src 디렉토리에서 Package 생성
              ↓
Python Node 또는 Interface 작성
              ↓
setup.py / CMakeLists.txt 설정
              ↓
package.xml 의존성 확인
              ↓
Workspace 최상위 디렉토리 이동
              ↓
         colcon build
              ↓
    source install/setup.bash
              ↓
      ros2 run / ros2 launch
```

Package 생성 위치와 빌드 위치를 혼동하지 않도록 주의해야 한다.

| 작업 | 실행 위치 |
|---|---|
| Package 생성 | `colcon_ws/src` |
| 소스 코드 작성 | 해당 Package 내부 |
| Package 빌드 | `colcon_ws` |
| 환경 설정 적용 | Workspace 기준 `source install/setup.bash` |

## 18. 실습 중 발생할 수 있는 오류

### 18-1. Package 또는 Executable을 찾지 못하는 경우

확인할 사항:

- Python 파일이 내부 Python Package 폴더에 있는지 확인
- `setup.py`의 `console_scripts` 등록 여부 확인
- Package 이름 및 파일 이름의 오타 확인
- Workspace 최상위 디렉토리에서 빌드했는지 확인
- `source install/setup.bash` 실행 여부 확인

### 18-2. Launch 파일을 찾지 못하는 경우

- Launch 파일이 `launch` 디렉토리에 존재하는지 확인
- `setup.py`의 `data_files` 등록 여부 확인
- 설치 대상 파일과 설치 경로 확인
- `ros2 launch` 명령어에 정확한 파일 이름 입력

### 18-3. Custom Interface 빌드 오류

- `ament_cmake` 기반으로 Package를 생성했는지 확인
- `CMakeLists.txt`에 모든 `.msg`, `.srv` 파일이 등록되어 있는지 확인
- Message 및 Service 파일 이름의 대소문자 확인
- `package.xml`에 Interface 생성 관련 의존성이 추가되어 있는지 확인
- Interface를 사용하는 Package에도 의존성이 등록되어 있는지 확인

### 18-4. Node 간 통신이 되지 않는 경우

- Topic 이름 또는 Service 이름 확인
- Message Type 또는 Service Type 확인
- 각 터미널의 ROS_DOMAIN_ID 확인
- Publisher, Subscriber 또는 Server의 실행 여부 확인
- Message 내부 필드에 정확하게 접근하는지 확인
- 로그를 이용하여 데이터 수신 및 처리 여부 확인

## 19. 실습 내용 정리

이번 실습에서는 단순히 두 개의 Node가 데이터를 주고받는 구조에서 입력, 연산, 출력 기능을 각각의 Node로 구분하는 구조로 확장했다.

```text
Input Node → Processing Node → Output Node
```

이러한 구조는 실제 로봇에서 센서 데이터를 수집하고, 필요한 연산을 수행한 뒤, 모터 제어 명령을 전달하는 과정에도 적용할 수 있다.

```text
Sensor Node
     ↓
Perception / Processing Node
     ↓
Planning Node
     ↓
Control Node
```

Launch 파일을 이용하면 여러 Node를 하나의 명령어로 실행할 수 있으며, Custom Interface를 이용하면 프로젝트에서 필요한 데이터 구조를 직접 정의할 수 있다.

Topic은 지속적인 센서 데이터 전달에 적합하고, Service는 특정 연산이나 상태 조회를 요청하는 데 사용할 수 있다. Action은 목적지까지 이동하는 것처럼 시간이 필요한 작업에 활용된다.

ROS 2를 사용하면서 Node 자체의 구현뿐 아니라 Package 구조, Interface 정의, 의존성 설정, 빌드 과정까지 함께 관리해야 한다는 점을 확인했다.
