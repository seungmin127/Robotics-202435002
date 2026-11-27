# Robotics-202435002

Robotics AI 기술 문서 — 3일차

---

## 1. 개요

3일차에는 IMU 센서, MCU와 센서를 연결하는 임베디드 통신 프로토콜(UART, I2C 등), 그리고 고수준 제어를 위한 ROS2의 기초 개념과 개발 환경을 배웠다. 실습으로는 ESP32에서 IMU(I2C), LiDAR(UART), 모터(IN1/IN2)를 각각 테스트한 뒤 LiDAR와 모터를 합쳐 장애물 회피 로봇을 만들었고, ROS2에서는 토픽으로 메시지를 주고받는 Publisher / Subscriber 노드를 작성했다.

| 항목 | 내용 |
| --- | --- |
| 작성자 | 신승민 (202435002) |
| 기간 | 3일차 |
| 실습 보드 | ESP32 + MPU6050 IMU (I2C) + LiDAR (UART) + 4륜 DC 모터 (모터 드라이버) |
| 개발 환경 | Arduino IDE, Ubuntu + ROS2 |

| 실습 | 파일 | 관련 이론 |
| --- | --- | --- |
| IMU 값 읽기 | `imu_test.ino` | I2C, IMU |
| LiDAR 거리 읽기 | `lidar_test.ino` | UART |
| 모터 동작 확인 | `motor_test.ino` | IN1 / IN2 |
| 장애물 회피 로봇 | `project01.ino` | UART + IN1 / IN2 |
| ROS2 Pub / Sub | `1_pubsub/` (`talker.py`, `listener.py`) | Node, Topic, QoS, colcon |

---

## 2. 이론

### 2.1 IMU (Inertial Measurement Unit, 관성 측정 장치)

IMU는 물체의 움직임과 자세를 측정하는 센서로, 로봇이 얼마나 기울었는지, 얼마나 회전했는지를 알 수 있게 해준다.

- **자이로스코프(Gyroscope):** 각속도 측정 (얼마나 빠르게 회전하는지)
- **가속도계(Accelerometer):** 선가속도 측정 (어느 방향으로 얼마나 가속하는지, 중력 방향 포함)
- **지자기 센서(Magnetometer):** 지구 자기장 측정 (나침반처럼 방향을 알려줌)

자이로 + 가속도계를 합친 것을 6축 IMU, 지자기 센서까지 포함하면 9축 IMU라고 한다. 실습에서 사용한 MPU6050은 6축 IMU이며 I2C로 MCU와 연결하므로, 코드에서 SDA 핀(`SDA_PIN 21`), SCL 핀(`SCL_PIN 22`), 센서 주소(`MPU_ADDR 0x68`)를 지정해야 한다.

### 2.2 임베디드 통신 프로토콜

임베디드 통신 프로토콜은 MCU와 센서, 액추에이터, 외부 제어 장치 사이에서 데이터를 주고받기 위한 통신 규칙이다. 데이터의 전송 순서, 신호 타이밍, 전송 형식, 장치를 구분하는 방법 등을 정의한다. 임베디드에서는 신호선을 적게 쓰기 위해 주로 직렬 통신을 사용하며, 통신 방식에 따라 동기식과 비동기식으로 나뉜다.

| 구분 | 동기식 | 비동기식 |
| --- | --- | --- |
| 클럭 | 클럭 신호선을 함께 보내 일정한 주기를 만든다 | 클럭이 없다 |
| 타이밍 기준 | 클럭에 맞춰 데이터를 읽는다 | 미리 약속한 전송 속도(보레이트) |
| 특징 | 규칙이 정해져 있어 여러 장치 연결에 유리 | 프레임 형식(시작/정지 비트) 사용, 수신 버퍼가 중요 |
| 예 | I2C, SPI | UART |

### 2.3 UART

UART는 두 장치 사이에서 사용하는 간단한 **비동기 직렬 통신**이다.

- TX(Transmit, 송신)와 RX(Receive, 수신) 두 신호선을 사용하며, 한쪽의 TX를 다른 쪽의 RX에 교차로 연결하고 GND를 공통으로 묶는다.
- 클럭이 없기 때문에 양쪽이 같은 보레이트를 약속하고 통신한다.
- 데이터는 비트마다 HIGH/LOW 전압으로 전송되고, 받는 쪽은 이를 수신 버퍼에 쌓아두었다가 꺼내 쓴다. 아두이노에서는 `Serial.available()`로 버퍼에 쌓인 데이터가 있는지 확인하고 `Serial.read()`로 하나씩 읽는다.
- 대표적으로 LiDAR, GPS, PC와의 시리얼 디버깅에 사용한다.

**UART 프레임 구조**

```
대기(HIGH) → 시작 비트(LOW) → 데이터 비트(보통 8비트, LSB부터) → (패리티 비트) → 정지 비트(HIGH)
```

평소에는 HIGH 상태로 대기하다가 시작 비트(LOW)로 데이터가 온다는 것을 알리고, 정지 비트(HIGH)로 한 바이트가 끝났음을 알린다. 실습에서 쓴 `SERIAL_8N1`은 데이터 8비트, 패리티 없음(N), 정지 비트 1개라는 뜻이다.

### 2.4 I2C

I2C는 MCU와 여러 주변 장치를 적은 수의 신호선으로 연결하기 위한 **동기식 직렬 통신**으로, 주소를 기반으로 동작한다.

- SDA(데이터)와 SCL(클럭) 두 개의 신호선만 사용한다. 두 선에는 풀업 저항이 필요하다.
- Master(MCU)가 클럭을 만들고, 통신할 Slave(센서)를 선택한다.
- 장치마다 고유한 주소가 있어서 하나의 버스에 여러 장치를 연결해도 주소로 구분해 통신할 수 있다.
- **시작 조건:** SCL이 HIGH인 동안 SDA가 HIGH → LOW로 떨어지면(falling) 통신을 시작한다.
- **정지 조건:** SCL이 HIGH인 동안 SDA가 LOW → HIGH로 올라가면(rising) 통신을 끝낸다.
- 첫 번째 바이트는 Slave 주소 7비트 + 읽기/쓰기(R/W) 1비트로 구성된다.
- 바이트를 하나 보낼 때마다 받는 쪽이 ACK(잘 받음) 또는 NACK(받지 못함/끝)로 응답한다.

### 2.5 그 밖의 통신 프로토콜

| 프로토콜 | 방식 | 신호선 | 장치 구분 | 주 사용처 |
| --- | --- | --- | --- | --- |
| UART | 비동기 | TX, RX | 1:1 연결 | LiDAR, GPS, 디버깅 |
| I2C | 동기 | SDA, SCL | 주소 | IMU 등 여러 센서 |
| SPI | 동기 | SCLK, MOSI, MISO, CS | CS(칩 선택) 핀 | 빠른 속도가 필요한 센서, 디스플레이 |
| CAN | 비동기 (차동 신호) | CAN_H, CAN_L | 메시지 ID | 자동차 ECU 간 통신 |

I2C와 SPI는 동기식이라 센서를 여러 개 연결할 때 주로 사용하고, CAN은 노이즈에 강해 주로 자동차에서 사용한다.

### 2.6 모터 드라이버 IN1 / IN2

DC 모터는 MCU 핀의 전류로 직접 돌릴 수 없기 때문에 모터 드라이버를 거쳐 제어한다. 모터 하나당 IN1, IN2 두 개의 입력 핀을 사용하며, 두 핀의 조합으로 회전 방향을 정한다.

| IN1 | IN2 | 동작 |
| --- | --- | --- |
| HIGH | LOW | 정회전 |
| LOW | HIGH | 역회전 |
| LOW | LOW | 정지 |

IN 핀에 HIGH 대신 PWM 신호를 주면 2일차에 배운 듀티 사이클로 속도까지 조절할 수 있다.

### 2.7 ROS2 기초

#### 2.7.1 ROS란

ROS는 Robot Operating System의 약자지만, 실제로는 운영체제가 아니라 로봇 개발을 위한 **소프트웨어 프레임워크**이다. 그런데도 OS라는 이름이 붙은 이유는 OS가 하는 역할 일부를 로봇 소프트웨어에 제공하기 때문이며, 그래서 meta OS라고 부르기도 한다.

- **하드웨어 추상화:** 하드웨어마다 다른 동작 방식과 세부 구현을 감추고, 일관된 인터페이스(데이터 패키징)로 사용할 수 있게 한다.
- **프로세스 간 통신:** 각 프로세스는 분리된 메모리 공간에서 실행되며, ROS가 이들 사이의 데이터 교환을 맡는다.

ROS는 단순한 프로그램이 아니라 하나의 생태계이다.

- **표준 통신 프로토콜:** 노드끼리 정해진 방식으로 데이터를 주고받는다.
- **패키지:** 로봇에 필요한 거의 모든 기능(SLAM, 내비게이션, 센서 드라이버 등)이 패키지로 존재한다.
- **시각화·개발 도구:** RViz(센서 데이터와 로봇 상태 시각화), Gazebo(물리 시뮬레이터)

#### 2.7.2 ROS1과 ROS2

| 구분 | ROS1 | ROS2 |
| --- | --- | --- |
| 통신 구조 | 중앙 집중형 (ROS Master 필요) | P2P (DDS 기반, 노드끼리 직접 연결) |
| 보안 | 지원 X | 지원 O |
| 지원 OS | 리눅스만 지원 | 리눅스, 윈도우, macOS 등 (단, 개발 생태계가 리눅스 중심이라 리눅스 권장) |

#### 2.7.3 Ubuntu와 ROS2 버전

ROS2 배포판은 특정 Ubuntu LTS 버전에 맞춰 나온다(예: Humble ↔ Ubuntu 22.04, Jazzy ↔ Ubuntu 24.04). 그래서 Ubuntu 버전과 ROS2 버전을 반드시 짝을 맞춰 설치해야 한다. 보통 최신 버전보다는 2년 정도 지난 04월 LTS 버전을 사용하는 것이 암묵적인 규칙인데, 오래 쓰이면서 안정성이 검증되고 패키지 지원이 충분하기 때문이다.

#### 2.7.4 환경 설정: source와 echo

```bash
# 현재 터미널에 ROS2 환경 불러오기 (humble 자리에 설치한 배포판 이름)
source /opt/ros/humble/setup.bash

# 새 터미널을 열 때마다 자동으로 불러오도록 .bashrc에 추가
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

`source`는 ROS2 명령어와 패키지 경로 같은 환경 변수를 현재 터미널에 등록하는 명령이다. 터미널을 새로 열면 사라지기 때문에, `echo`로 이 명령을 `~/.bashrc`에 추가해 두면 터미널을 열 때마다 자동으로 실행된다. 이 과정을 빼먹으면 `ros2` 명령어 자체를 찾지 못하기 때문에 굉장히 중요하다.

#### 2.7.5 ROS2 통신 환경: DDS와 Domain ID

ROS2는 통신 계층으로 **DDS(Data Distribution Service)** 를 사용한다.

- 산업 표준의 고신뢰성 데이터 통신 규격으로, 검증된 통신 프로토콜을 사용한다.
- P2P 구조라 노드들끼리 서로를 자동으로 발견(discovery)한다.
- QoS(통신 품질) 설정이 가능하다.
- 구현체로 Fast DDS, Cyclone DDS 등이 있다.

**Domain ID**는 ROS 노드들을 도메인 단위로 묶는 번호이다. 같은 `ROS_DOMAIN_ID`를 가진 노드끼리만 같은 DDS 도메인에 연결되어 서로의 토픽을 자동으로 발견한다. 기본값은 0이며, 같은 네트워크에서 여러 사람이 실습할 때 번호를 다르게 주면 서로 간섭하지 않는다.

```bash
export ROS_DOMAIN_ID=1
```

#### 2.7.6 ROS2 기본 개념

| 개념 | 설명 |
| --- | --- |
| Node | 하나의 기능을 수행하는 실행 단위(프로세스). 보통 터미널 하나에서 노드 하나를 실행한다 |
| Message / msg type | 노드끼리 주고받는 데이터와 그 형식 (예: `std_msgs/msg/String`, `geometry_msgs/msg/Twist`, `sensor_msgs/msg/LaserScan`) |
| Topic | 메시지가 오가는 이름 붙은 통로. 한 방향으로 계속 흘러가는 데이터에 사용 |
| Publisher / Subscriber | 토픽에 메시지를 보내는 쪽 / 토픽을 구독해 메시지를 받는 쪽 (1:N 가능) |
| Service | 요청-응답 방식의 통신 (4일차) |
| Action | 오래 걸리는 작업을 위한 목표-피드백-결과 방식의 통신 (4일차) |

#### 2.7.7 QoS (Quality of Service)

QoS는 노드끼리 데이터를 주고받을 때 적용하는 통신 규칙이다.

| 항목 | 설정 | 의미 |
| --- | --- | --- |
| 신뢰성 (Reliability) | Reliable | 데이터가 유실되면 재전송해서 반드시 전달 |
| | Best Effort | 재전송 없이 빠르게 전달, 일부 유실 허용 (LiDAR처럼 데이터가 빠르게 계속 들어오는 센서에 적합) |
| 지속성 (Durability) | Volatile | 구독을 시작한 이후의 데이터만 받음 |
| | Transient Local | 늦게 들어온 구독자에게도 마지막으로 발행된 데이터를 전달 |

Publisher와 Subscriber의 QoS가 서로 맞지 않으면(예: Publisher는 Best Effort, Subscriber는 Reliable) 연결이 되지 않기 때문에 설정을 맞춰야 한다.

#### 2.7.8 colcon 워크스페이스

colcon 워크스페이스는 ROS2 패키지를 개발하고 관리하기 위한 작업 공간으로, 소스 코드와 빌드 결과물을 분리해서 관리한다.

```
ros2_ws/
├── src/       # 개발자가 작성한 ROS2 패키지의 소스 코드 (직접 수정하는 곳)
├── build/     # 컴파일 및 빌드 과정에서 생성되는 중간 파일
├── install/   # 빌드 후 설치된 실행 파일, 라이브러리, 설정 파일
└── log/       # 빌드 및 테스트 과정의 로그
```

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build                 # 워크스페이스 최상위에서 빌드
source install/setup.bash    # 빌드한 패키지를 현재 터미널에 등록
```

#### 2.7.9 로그

로그는 노드가 어떤 상태인지, 문제가 어디서 생겼는지 파악하는 가장 중요한 단서이다. ROS2 로그는 DEBUG, INFO, WARN, ERROR, FATAL 단계로 나뉘며, 의미 없는 로그를 남발하면 정말 중요한 로그를 놓치게 되므로 필요한 정보만 알맞은 단계로 남겨야 한다.

---

## 3. 실습

### 3.1 하드웨어 구성

| 구성 | 핀 / 설정 |
| --- | --- |
| MPU6050 IMU | SDA GPIO21, SCL GPIO22, 주소 `0x68` |
| LiDAR | RX GPIO16, `Serial2` 230400 bps, 8N1 (수신만 사용) |
| 앞 오른쪽 모터 (FR) | IN1 GPIO26, IN2 GPIO25 |
| 앞 왼쪽 모터 (FL) | IN1 GPIO19, IN2 GPIO27 |
| 뒤 오른쪽 모터 (RR) | IN1 GPIO23, IN2 GPIO12 |
| 뒤 왼쪽 모터 (RL) | IN1 GPIO0, IN2 GPIO15 |

LiDAR 각도 기준은 뒤쪽 0°, 왼쪽 90°, 앞쪽 180°, 오른쪽 270°이다.

### 3.2 IMU 값 읽기 (`imu_test.ino`)

MPU6050에서 가속도와 각속도를 읽어 실제 단위(m/s², deg/s)로 바꿔 시리얼 모니터에 출력했다.

**수행 과정**

1. `Wire.begin(SDA_PIN, SCL_PIN)`으로 I2C를 시작한다.
2. MPU6050은 전원이 들어오면 슬립 상태이므로, 전원 관리 레지스터(`0x6B`)에 0을 써서 깨운다.
3. 가속도 X축 데이터가 시작되는 `0x3B` 레지스터를 지정하고, 가속도 6바이트 + 온도 2바이트 + 자이로 6바이트 = 14바이트를 연속으로 읽는다.
4. 2바이트씩 합쳐 원시값(raw)을 만들고, 감도로 나눠 실제 단위로 변환한다.

```cpp
Wire.beginTransmission(MPU_ADDR);
Wire.write(0x3B);                 // 읽기 시작할 레지스터
Wire.endTransmission(false);      // 정지 조건 없이 바로 이어서 읽기
Wire.requestFrom(MPU_ADDR, 14);

int16_t ax_raw = (Wire.read() << 8) | Wire.read();   // 상위 바이트가 먼저 옴
...
float ax = (ax_raw / 16384.0) * 9.80665;   // ±2g 범위: 16384 = 1g → m/s²
float gx = gx_raw / 131.0;                 // ±250°/s 범위: 131 = 1°/s
```

**적용 이론**

- **I2C:** 주소 `0x68`로 센서를 선택하고 레지스터 번호를 보낸 뒤 데이터를 요청했다. `endTransmission(false)`는 정지 조건을 보내지 않고 통신을 이어가는 것으로, 레지스터 지정과 읽기를 한 번의 통신으로 처리하기 위해 사용했다.
- **IMU:** MPU6050의 기본 측정 범위(가속도 ±2g, 자이로 ±250°/s)에 맞는 감도 값으로 나눠 단위를 변환했다. 로봇이 가만히 있으면 중력 때문에 Z축 가속도가 약 9.8 m/s²로 나와야 한다.
- 온도 데이터는 사용하지 않지만, 레지스터가 연속으로 붙어 있어서 읽고 넘겨야 자이로 데이터를 읽을 수 있다.

### 3.3 LiDAR 거리 읽기 (`lidar_test.ino`)

UART로 들어오는 LiDAR 패킷을 해석해 0~359°의 거리값을 배열에 저장하고, 한 바퀴가 돌 때마다 전체 거리값을 출력했다.

**LiDAR 패킷 구조 (47바이트)**

| 바이트 | 내용 |
| --- | --- |
| 0 | 헤더 `0x54` |
| 1 | VerLen `0x2C` |
| 2~3 | 회전 속도 |
| 4~5 | 시작 각도 (0.01° 단위) |
| 6~41 | 측정점 12개 (각각 거리 2바이트 + 신뢰도 1바이트) |
| 42~43 | 끝 각도 (0.01° 단위) |
| 44~45 | 타임스탬프 |
| 46 | CRC |

**수행 과정**

1. `Serial2` 수신 버퍼에서 헤더 `0x54`, `0x2C`를 찾아 패킷의 시작을 확인한다.
2. 나머지 45바이트를 읽어 47바이트 패킷을 완성한다.
3. 시작 각도와 끝 각도를 꺼내 100으로 나누고, 그 사이를 11구간으로 나눠 측정점 12개의 각도를 계산한다.
4. 각도를 정수로 반올림해 `distances[360]`에 거리(mm)를 저장한다. 거리 0은 유효하지 않은 값으로 보고 버린다.
5. 각도가 360°에서 0°로 넘어가면 한 바퀴가 끝난 것으로 보고 0~359° 거리값을 모두 출력한다.

```cpp
Serial2.begin(230400, SERIAL_8N1, LIDAR_RX, -1);   // 수신 전용 UART

// LiDAR는 low byte, high byte 순서로 보내므로 두 바이트를 합쳐 16비트 값으로 만든다
uint16_t readUInt16(uint8_t low, uint8_t high) {
    return low | (high << 8);
}

uint16_t startRaw = readUInt16(packet[4], packet[5]);
float startAngle = startRaw / 100.0;               // 0.01° 단위 → 도
float angleStep = (endAngle - startAngle) / 11.0;  // 측정점 12개 → 11구간
```

**적용 이론**

- **UART:** LiDAR는 클럭 없이 230400 bps로 데이터를 계속 보내는 비동기 장치이다. 데이터가 수신 버퍼에 쌓이므로 `Serial2.available()`로 확인하고 읽었고, 헤더 바이트로 패킷의 시작을 찾는 프레임 방식을 직접 구현했다. LiDAR에서 받기만 하면 되므로 TX 핀은 `-1`로 사용하지 않았다.
- **바이트 순서:** LiDAR는 낮은 바이트가 먼저 오고(little endian), 3.2의 MPU6050은 높은 바이트가 먼저 온다(big endian). 센서마다 바이트 순서가 다르기 때문에 데이터시트를 보고 합치는 순서를 맞춰야 한다.

### 3.4 모터 동작 확인 (`motor_test.ino`)

네 바퀴를 하나씩 정회전 1초 → 정지 0.2초 → 역회전 1초 → 정지 0.2초 순서로 돌려, 각 모터의 배선과 회전 방향이 맞는지 확인했다.

```cpp
for (int i = 0; i < 8; i += 2) {
    digitalWrite(motorPins[i], HIGH); digitalWrite(motorPins[i + 1], LOW);   // 정회전
    delay(1000);
    digitalWrite(motorPins[i], LOW);  digitalWrite(motorPins[i + 1], LOW);   // 정지
    delay(200);
    digitalWrite(motorPins[i], LOW);  digitalWrite(motorPins[i + 1], HIGH);  // 역회전
    delay(1000);
    digitalWrite(motorPins[i], LOW);  digitalWrite(motorPins[i + 1], LOW);   // 정지
    delay(200);
}
```

**적용 이론:** IN1 / IN2의 조합(HIGH-LOW 정회전, LOW-HIGH 역회전, LOW-LOW 정지)으로 회전 방향을 제어했다. 핀 8개를 `motorPins` 배열에 IN1, IN2 순서로 넣어 두고 2칸씩 건너뛰며 반복하면, 모터 4개를 같은 코드로 차례대로 시험할 수 있다. 회전 방향을 바꾸기 전에 잠깐 정지 시간을 둔 것은 모터와 드라이버에 갑자기 큰 부하가 걸리지 않게 하기 위해서이다.

### 3.5 LiDAR 기반 장애물 회피 로봇 (`project01.ino`)

3.3의 LiDAR 파싱 코드와 3.4의 모터 제어 코드를 합쳐, 전방에 장애물이 있으면 멈춘 뒤 더 넓게 트인 쪽으로 회전하는 로봇을 만들었다.

**동작 흐름**

1. LiDAR 한 바퀴 측정이 끝날 때마다 장애물 판단을 실행한다.
2. 전방(150~210°), 좌측(60~120°), 우측(240~300°)에서 각각 가장 가까운 거리를 구한다.
3. 전방 거리가 300 mm보다 가까우면 정지 → 더 넓은 쪽으로 400 ms 회전 → 정지한다.
4. 앞이 트여 있으면 전진한다.

```cpp
uint16_t frontDist = getMinDistance(150, 210);   // 전방 ±30°
uint16_t leftDist  = getMinDistance(60, 120);    // 좌측 ±30°
uint16_t rightDist = getMinDistance(240, 300);   // 우측 ±30°

if (frontDist < SAFE_DISTANCE) {
    stopMotors();
    if (!wasTurning) {                            // 새 장애물일 때만 방향을 새로 정함
        turnDirection = (leftDist > rightDist) ? 1 : -1;
    }
    if (turnDirection == 1) turnLeft(); else turnRight();
    wasTurning = true;
} else {
    wasTurning = false;
    moveForward();
}
```

```cpp
// 좌회전: 오른쪽 바퀴 정회전, 왼쪽 바퀴 역회전 (제자리 회전)
digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
digitalWrite(FL_IN1, LOW);  digitalWrite(FL_IN2, HIGH);
```

**적용 이론:** 센서 입력(LiDAR, UART) → 판단(구역별 최소 거리 비교) → 출력(모터, IN1/IN2)이라는, 앞으로 자율주행 로봇이 갖게 될 기본 구조를 MCU 하나로 만들어 보았다. `wasTurning`은 회전하는 동안 방향을 유지해서, 좌우 거리가 비슷할 때 로봇이 좌우로 왔다 갔다 하는 것을 막는다.

### 3.6 ROS2 Publisher / Subscriber (`1_pubsub`)

ROS2 파이썬 패키지 `1_pubsub`을 만들고, 1초마다 메시지를 보내는 `talker` 노드와 그 메시지를 받아 출력하는 `listener` 노드를 작성했다.

**패키지 구조**

```
ros2_ws/src/1_pubsub/
├── 1_pubsub/
│   ├── __init__.py
│   ├── talker.py        # Publisher 노드
│   └── listener.py      # Subscriber 노드
├── resource/1_pubsub
├── test/                # 코드 스타일 검사용 기본 테스트 (copyright, flake8, pep257)
├── package.xml          # 패키지 정보와 의존성
├── setup.py             # 설치 방법과 실행 명령 등록
└── setup.cfg            # 실행 파일 설치 위치
```

**talker.py (Publisher)**

```python
class Talker(Node):
    def __init__(self):
        super().__init__('talker')                                              # 노드 이름
        self.publisher_ = self.create_publisher(String, '/demo_topic', 10)      # 메시지 타입, 토픽, 큐 크기
        self.start_time = time.time()
        self.timer = self.create_timer(1.0, self.publish_message)               # 1초마다 콜백 실행

    def publish_message(self):
        elapsed = int(time.time() - self.start_time)
        msg = String()
        msg.data = f"Hello ROS2: {elapsed}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
```

**listener.py (Subscriber)**

```python
class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription_ = self.create_subscription(String, '/demo_topic', self.callback, 10)

    def callback(self, msg):                      # 메시지가 들어올 때마다 실행
        self.get_logger().info(f'Received: "{msg.data}"')
```

두 노드 모두 `main()`에서 `rclpy.init()` → 노드 생성 → `rclpy.spin()`(콜백을 계속 처리하며 대기) → `destroy_node()` → `rclpy.shutdown()` 순서로 실행된다.

**setup.py: 실행 명령 등록**

```python
entry_points={
    'console_scripts': [
        'talker = 1_pubsub.talker:main',       # ros2 run 1_pubsub talker
        'listener = 1_pubsub.listener:main',   # ros2 run 1_pubsub listener
    ],
},
```

`실행 명령 이름 = 패키지.파일:함수` 형식으로 등록해야 `ros2 run`으로 노드를 실행할 수 있다. 나중에 launch 파일이나 설정 파일을 패키지에 포함하려면 `data_files`에 `('share/' + package_name + '/폴더', ['폴더/파일'])` 형식으로 추가한다.

**package.xml: 의존성**

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
```

이번 실습에는 `rclpy`와 `std_msgs`만 쓰지만, 앞으로 속도 명령(`geometry_msgs`), LiDAR·IMU 데이터(`sensor_msgs`), 지도·위치(`nav_msgs`)를 다룰 것을 대비해 함께 추가해 두었다.

**빌드 및 실행**

```bash
cd ~/ros2_ws
colcon build --packages-select 1_pubsub
source install/setup.bash

ros2 run 1_pubsub talker       # 터미널 1
ros2 run 1_pubsub listener     # 터미널 2 (이 터미널에서도 source 필요)

ros2 topic list                # /demo_topic 확인
ros2 topic echo /demo_topic    # 토픽 내용 직접 확인
```

**결과:** talker 터미널에는 `Publishing: "Hello ROS2: 0"`, `"Hello ROS2: 1"` … 이 1초마다 출력되고, listener 터미널에는 같은 내용이 `Received: ...`로 출력된다.

**적용 이론**

- **Node / Topic / Publisher / Subscriber:** talker와 listener는 서로를 직접 알지 못하고, `/demo_topic`이라는 토픽 이름과 `std_msgs/msg/String`이라는 메시지 타입만 맞춰서 통신한다. DDS가 같은 Domain ID 안에서 두 노드를 자동으로 찾아 연결해준다.
- **QoS:** `create_publisher`와 `create_subscription`의 마지막 인자 `10`은 큐 크기(History depth)로, 처리하지 못한 메시지를 최근 10개까지 보관한다는 뜻이다.
- **colcon / source:** 빌드 결과는 `install/`에 생기므로, 터미널마다 `source install/setup.bash`를 해야 `ros2 run`이 패키지를 찾을 수 있다.
- **로그:** `get_logger().info()`로 보내고 받은 내용을 INFO 단계 로그로 남겨, 통신이 되는지 바로 확인할 수 있게 했다.

---

## 4. 느낀 점

- **통신 방식에 따라 코드가 달라진다:** IMU는 I2C라 주소와 레지스터 번호로 데이터를 요청했고, LiDAR는 UART라 보레이트와 패킷 형식(헤더, 길이)을 정확히 알아야 데이터를 해석할 수 있었다. 같은 2바이트 값이라도 MPU6050은 높은 바이트가, LiDAR는 낮은 바이트가 먼저 와서, 센서마다 데이터시트를 꼭 확인해야 한다는 것을 느꼈다.
- **하나씩 테스트하고 합치기:** IMU, LiDAR, 모터를 각각 따로 테스트한 뒤 합치니, 장애물 회피 로봇이 이상하게 움직일 때 센서 문제인지 모터 배선 문제인지 나눠서 확인할 수 있었다.
- **블로킹 코드의 한계:** 장애물을 피하는 동안 `delay()`를 쓰면 그 시간 동안 LiDAR 데이터를 읽지 못해 수신 버퍼에 데이터가 밀린다. 비동기 통신에서 버퍼가 중요한 이유를 직접 느꼈고, 2일차의 디바운싱처럼 `millis()` 기반으로 바꾸거나 나중에 RTOS로 작업을 나누면 더 좋을 것 같다.
- **데이터 검증:** LiDAR 패킷 마지막 바이트에 CRC가 있지만 이번 코드에서는 확인하지 않았다. 노이즈로 깨진 패킷이 들어오면 잘못된 거리값이 저장될 수 있으므로, CRC 검사를 추가하면 더 안정적일 것이다.
- **ESP32 핀 선택:** GPIO0과 GPIO12는 부팅할 때 상태를 확인하는 핀(스트래핑 핀)이라, 모터 드라이버 연결 상태에 따라 부팅이 안 될 수 있다. 다음에는 이런 핀을 피해서 배선하는 것이 좋겠다.
- **고수준과의 연결:** ROS2 Pub/Sub 실습을 해보니, 지금 MCU가 직접 처리하는 LiDAR·IMU 데이터를 앞으로는 `sensor_msgs` 토픽으로 보내고, ROS2에서 판단한 속도 명령을 다시 MCU가 받아 모터를 돌리는 구조가 될 것이라는 그림이 그려졌다.
