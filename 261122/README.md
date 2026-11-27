# Robotics-202435002

Robotics AI 기술 문서 — 4일차 (2026-11-22)

---

## 1. 개요

4일차에는 3일차에 배운 토픽(Topic) 통신을 확장해 노드 3개가 이어지는 파이프라인을 만들고 여러 노드를 한 번에 실행하는 Launch 파일, 요청-응답 방식의 서비스(Service), 오래 걸리는 작업을 위한 액션(Action), 그리고 직접 메시지 형식을 정의하는 사용자 정의 인터페이스를 배웠다. 

| 항목 | 내용 |
| --- | --- |
| 작성자 | 신승민 (202435002) |
| 기간 | 4일차 (2026-11-22) |
| 개발 환경 | Ubuntu + ROS2 |

| 실습 | 패키지 / 파일 | 관련 이론 |
| --- | --- | --- |
| 키 입력 위치 추적 파이프라인 | `1_pubsub/` (`publisher.py`, `processor.py`, `subscriber.py`) | Topic |
| 노드 3개 한 번에 실행 | `1_pubsub/launch/my_location.launch.py` | Launch 파일 |
| 사용자 정의 메시지·서비스 | `custom_interfaces/` | 사용자 정의 인터페이스 |
| 두 수 더하기 / 로봇 이동 서비스 | `2_srvmsg/` | Service |

---

## 2. 이론

### 2.1 Launch 파일

노드가 많아지면 터미널을 여러 개 열고 `source`와 `ros2 run`을 하나씩 입력하기가 번거롭고 실수도 생기기 쉽다. Launch 파일은 여러 노드를 한 번에 실행하고 실행 순서나 파라미터, 토픽 이름 변경 등을 함께 설정할 수 있게 해준다. ROS2에서는 주로 파이썬(`.launch.py`)으로 작성하며 `generate_launch_description()` 함수가 실행할 항목을 `LaunchDescription`에 담아 반환한다.

```bash
ros2 launch <패키지 이름> <launch 파일 이름>
```

Launch 파일을 `ros2 launch`로 실행하려면 `setup.py`의 `data_files`에 등록해 빌드 시 `install/` 폴더로 복사되게 해야 한다.

### 2.2 Service

서비스는 노드 사이의 **요청(Request)과 응답(Response)** 을 기반으로 하는 통신 방식이다.

- 한 번 요청하고 한 번 응답받는 일회성 요청이나 짧은 작업에 적합하다.
- Client(요청하는 쪽) – Server(처리하고 응답하는 쪽) 구조로 동작한다.
- `.srv` 파일로 Request와 Response의 데이터 구조를 정의한다.
- 예시: 센서 데이터 요청, 로봇 상태 조회, 특정 값 초기화

### 2.3 Action

액션은 **오래 걸리는 작업**을 위한 비동기 통신 방식이다.

- Client – Server 구조로 동작한다.
- Goal(목표), Feedback(진행 상황), Result(최종 결과)의 세 가지 요소로 이루어진다.
- 작업 도중 Cancel(취소) 요청을 보낼 수 있다.
- 예시: 목표 지점까지 로봇 이동, 로봇 팔 동작 수행

**동작 순서**

1. Client가 Server에 Goal을 보낸다.
2. Server가 Goal을 수락하고 작업을 시작한다.
3. 작업하는 동안 Server가 Feedback을 주기적으로 보낸다 (예: 남은 거리).
4. 작업이 끝나면 Server가 Result를 보낸다. 중간에 Client가 Cancel을 보내면 작업을 멈춘다.

### 2.4 Topic / Service / Action 비교

| 구분 | Topic | Service | Action |
| --- | --- | --- | --- |
| 구조 | Publisher – Subscriber | Client – Server | Client – Server |
| 방향 | 단방향, 계속 전송 | 요청 1번 → 응답 1번 | 목표 → 피드백(여러 번) → 결과 |
| 응답 대기 | 없음 | 응답이 올 때까지 | 비동기, 도중 취소 가능 |
| 적합한 작업 | 계속 들어오는 데이터 | 짧고 한 번에 끝나는 작업 | 오래 걸리는 작업 |
| 예시 | LiDAR 스캔, 속도 명령 | 로봇 상태 조회, 값 초기화 | 목표 지점까지 이동, 로봇 팔 동작 |

### 2.5 사용자 정의 인터페이스

ROS2가 기본으로 제공하는 메시지 형식만으로 부족할 때는 `.msg`, `.srv`, `.action` 파일을 직접 만들어 사용할 수 있다.

- 인터페이스만 모아두는 별도 패키지를 `ament_cmake` 타입으로 만들고 그 안에 `msg/`, `srv/`, `action/` 폴더를 두어 정의한다.
- 직접 만든 메시지를 다른 메시지나 서비스 안에서 자료형처럼 다시 사용할 수 있다.

---

## 3. 실습

### 3.1 키 입력 위치 추적 파이프라인 (`1_pubsub`)

3일차의 talker / listener는 노드 2개가 토픽 하나로 연결되는 구조였다. 4일차에는 중간에 데이터를 가공하는 노드를 하나 더 두어, 키보드 입력(w, a, s, d)으로 좌표를 움직이고 현재 위치를 출력하는 노드 3개짜리 파이프라인을 만들었다.

```
publisher ──(key_input)──▶ processor ──(key_output)──▶ subscriber
 키 입력                     좌표 계산                     현재 위치 출력
```

| 노드 | 구독 토픽 | 발행 토픽 | 역할 |
| --- | --- | --- | --- |
| publisher | - | `key_input` | 사용자에게 w/a/s/d를 입력받아 발행 |
| processor | `key_input` | `key_output` | 입력에 따라 (x, y) 좌표를 갱신해 발행 |
| subscriber | `key_output` | - | 현재 위치를 로그로 출력 |

**publisher.py**

```python
self.publisher_ = self.create_publisher(String, 'key_input', 10)
self.timer = self.create_timer(0.1, self.publish_message)

def publish_message(self):
    msg = String()
    msg.data = input("input(w,a,s,d): ").lower()   # 소문자로 통일
    self.publisher_.publish(msg)
```

**processor.py (구독과 발행을 모두 하는 노드)**

```python
self.x, self.y = 0, 0
self.subscription_ = self.create_subscription(String, 'key_input', self.callback, 10)
self.publisher_ = self.create_publisher(String, 'key_output', 10)

def callback(self, msg):
    key = msg.data
    if key == 'w' : self.y += 1
    elif key == 'a' : self.x -= 1
    elif key == 's' : self.y -= 1
    elif key == 'd' : self.x += 1

    out = String()
    out.data = f"({self.x},{self.y})"
    self.publisher_.publish(out)
```

**subscriber.py**

```python
self.subscription_ = self.create_subscription(String, 'key_output', self.callback, 10)

def callback(self, msg):
    self.get_logger().info(f'now location: "{msg.data}"')
```

- **결과:** publisher 터미널에서 `w`, `d`, `d`를 차례로 입력하면 subscriber 터미널에 `now location: "(0,1)"`, `"(1,1)"`, `"(2,1)"`이 출력된다.
- **적용 이론:** processor처럼 한 노드가 Subscriber이면서 동시에 Publisher가 될 수 있다. 센서 데이터를 받아 가공한 뒤 다른 노드로 넘기는 구조는 실제 로봇에서도 흔히 쓰이며(예: LiDAR 원본 → 필터링 → 장애물 판단), 노드마다 역할을 하나씩 나누면 각각 따로 수정하고 테스트할 수 있다.

### 3.2 Launch 파일로 노드 3개 실행 (`my_location.launch.py`)

3.1의 노드 3개를 각각 다른 터미널에서 실행해야 하는데 publisher는 키 입력을 받아야 하기 때문에 자기 터미널이 따로 필요하다. 그래서 Launch 파일에서 `gnome-terminal`을 새로 열어 각 노드를 실행하고 노드가 시작된 순서대로 다음 노드를 실행하도록 했다.

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    publisher = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c',
            'source ~/robot_navigation/colcon_ws/install/setup.bash; '
            'ros2 run 1_pubsub publisher; '
            'exec bash'
        ],
        output='screen'
    )
    # processor, subscriber도 같은 형식

    return LaunchDescription([
        publisher,
        RegisterEventHandler(                       # publisher가 시작되면 processor 실행
            OnProcessStart(target_action=publisher, on_start=[processor])
        ),
        RegisterEventHandler(                       # processor가 시작되면 subscriber 실행
            OnProcessStart(target_action=processor, on_start=[subscriber])
        ),
    ])
```

| 구성 요소 | 역할 |
| --- | --- |
| `ExecuteProcess` | 터미널 명령을 실행한다. 여기서는 새 터미널을 열고 `source` 후 `ros2 run` 실행 |
| `exec bash` | 노드가 끝나도 터미널 창이 바로 닫히지 않게 유지 |
| `RegisterEventHandler` + `OnProcessStart` | 지정한 프로세스가 시작되는 이벤트에 맞춰 다음 동작을 실행 (실행 순서 제어) |

**setup.py에 launch 파일 등록**

```python
data_files=[
    ...
    ('share/' + package_name + '/launch',
        ['launch/my_location.launch.py']),
],
entry_points={
    'console_scripts': [
        'publisher = 1_pubsub.publisher:main',
        'processor = 1_pubsub.processor:main',
        'subscriber = 1_pubsub.subscriber:main',
    ],
},
```

```bash
colcon build --packages-select 1_pubsub
source install/setup.bash
ros2 launch 1_pubsub my_location.launch.py
```

- **결과:** 명령 한 줄로 터미널 3개가 열리고 publisher → processor → subscriber 순서로 노드가 실행된다.
- **적용 이론:** Launch 파일로 실행 구성을 코드로 남겨두면 매번 터미널을 열고 명령을 입력하지 않아도 같은 환경을 그대로 다시 실행할 수 있다. `data_files`에 등록하지 않으면 launch 파일이 `install/`에 복사되지 않아 `ros2 launch`가 파일을 찾지 못한다.

### 3.3 사용자 정의 인터페이스 (`custom_interfaces`)

서비스 실습에 사용할 메시지와 서비스 형식을 직접 정의한 인터페이스 패키지를 만들었다.

| 파일 | 내용 | 설명 |
| --- | --- | --- |
| `msg/Num.msg` | `int64 my_num` | 정수 하나를 담는 메시지 |
| `msg/RobotPose.msg` | `float64 x`, `float64 y`, `float64 theta` | 로봇의 위치(x, y)와 방향(theta, 라디안) |
| `srv/AddTwoInts.srv` | 요청 `Num a`, `Num b` / 응답 `Num sum` | 직접 만든 `Num` 메시지를 자료형으로 사용 |
| `srv/MoveRobot.srv` | 요청 `RobotPose current_pose`, `float64 distance` / 응답 `RobotPose result_pose`, `bool success` | 현재 자세에서 일정 거리 이동한 결과 |

```
# MoveRobot.srv
RobotPose current_pose
float64 distance
---
RobotPose result_pose
bool success
```

**CMakeLists.txt**

```cmake
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddTwoInts.srv"
  "msg/RobotPose.msg"
  "srv/MoveRobot.srv"
)
```

**package.xml**

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

```bash
colcon build --packages-select custom_interfaces
source install/setup.bash
ros2 interface show custom_interfaces/srv/MoveRobot   # 정의 확인
```

- **적용 이론:** 인터페이스 패키지는 파이썬 노드 패키지(`ament_python`)와 달리 `ament_cmake`로 만들어야 `rosidl`이 코드를 생성할 수 있다. 서비스 패키지(`2_srvmsg`)의 `package.xml`에 `<depend>custom_interfaces</depend>`를 추가해야 `from custom_interfaces.srv import MoveRobot`으로 가져다 쓸 수 있다.

### 3.4 서비스 서버 / 클라이언트 (`2_srvmsg`)

#### 3.4.1 두 수 더하기 (`add_ints_server.py`, `add_ints_client.py`)

**서버**

```python
self.srv = self.create_service(AddTwoInts, '/add_two_ints', self.srv_callback)

def srv_callback(self, request, response):
    response.sum.my_num = request.a.my_num + request.b.my_num
    return response
```

**클라이언트**

```python
self.cli = self.create_client(AddTwoInts, '/add_two_ints')
while not self.cli.wait_for_service(timeout_sec=1.0):      # 서버가 켜질 때까지 대기
    self.get_logger().info('Waiting for service...')

self.req.a.my_num = int(a)                                 # Num 메시지 안의 필드에 값 저장
self.req.b.my_num = int(b)
self.future = self.cli.call_async(self.req)                # 비동기로 요청
rclpy.spin_until_future_complete(self, self.future)        # 응답이 올 때까지 대기
```

`AddTwoInts.srv`의 `a`, `b`가 정수가 아니라 `Num` 메시지이기 때문에 `req.a`가 아니라 `req.a.my_num`처럼 한 단계 안으로 들어가 값을 넣어야 한다.

#### 3.4.2 로봇 이동 (`move_robot_server.py`, `move_robot_client.py`)

현재 위치(x, y), 방향(theta), 이동 거리를 요청으로 보내면 서버가 바라보는 방향으로 그 거리만큼 이동한 위치를 계산해 응답한다.

```latex
x' = x + d\cos\theta, \qquad y' = y + d\sin\theta, \qquad \theta' = \theta
```

**서버**

```python
def srv_callback(self, request, response):
    response.result_pose.x = request.current_pose.x + request.distance * math.cos(request.current_pose.theta)
    response.result_pose.y = request.current_pose.y + request.distance * math.sin(request.current_pose.theta)
    response.result_pose.theta = request.current_pose.theta
    response.success = True
    return response
```

**클라이언트 (명령줄 인자 4개를 받아 요청)**

```python
response = client.send_request(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])   # x y theta distance
```

```bash
ros2 run 2_srvmsg move_robot_server                 # 터미널 1
ros2 run 2_srvmsg move_robot_client 0 0 0 1         # 터미널 2: (0, 0)에서 0 rad 방향으로 1 이동
```

- **결과:** `(0, 0)`, `theta = 0`에서 거리 1을 요청하면 `New Pose X: 1.0`, `New Pose Y: 0.0`, `New Pose Theta: 0.0`, `Result: True`가 출력된다. `theta`는 라디안이므로 90° 방향은 `1.5708`처럼 입력해야 한다.
- **적용 이론:** 클라이언트는 서버가 준비될 때까지 `wait_for_service()`로 기다린 뒤 `call_async()`로 요청을 보내고 `spin_until_future_complete()`로 응답을 받는다. 요청 한 번에 응답 한 번으로 끝나는 짧은 계산이라 서비스가 적합하다. 실제로 로봇이 그 거리를 이동하면서 진행 상황을 알려줘야 한다면 액션이 더 알맞다.

---

## 4. 느낀 점

- **노드를 역할별로 나누기:** 3.1에서 입력, 계산, 출력을 노드 3개로 나눠 보니 좌표 계산 방식을 바꾸고 싶으면 processor만 고치면 됐다. 나중에 LiDAR 데이터를 받아 장애물을 판단하는 노드를 만들 때도 이렇게 나누는 것이 좋을 것 같다.