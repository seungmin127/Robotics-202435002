// Front Right
#define FR_IN1 26
#define FR_IN2 25
// Front Left
#define FL_IN1 19
#define FL_IN2 27
// Rear Right
#define RR_IN1 23
#define RR_IN2 12
// Rear Left
#define RL_IN1 0
#define RL_IN2 15

int motorPins[8] = {
    FR_IN1, FR_IN2,
    FL_IN1, FL_IN2,
    RR_IN1, RR_IN2,
    RL_IN1, RL_IN2
};

// LiDAR 각도 기준
/*
 * Back  :    0°
 * Left  :   90°
 * Front :  180°
 * Right :  270°
*/

#define LIDAR_RX 16
#define SAFE_DISTANCE 300   // 장애물 감지 기준 거리 (mm)

uint8_t packet[47];
uint16_t distances[360] = {0};

float previousAngle = -1.0;

// 회전 방향 유지용 (1: 좌, -1: 우)
int turnDirection = 0;
// 직전 판단에서 회전했는지 (앞이 계속 막혀 있는 상태)
bool wasTurning = false;
// 회전 직후 한 바퀴는 회전 전/중 데이터가 섞여 있으므로 판단을 건너뜀
bool skipNextScan = false;

// 모터 정지
void stopMotors() {
    for (int i = 0; i < 8; i++) {
        digitalWrite(motorPins[i], LOW);
    }
}

// 모터 전진
void moveForward() {
    for (int i = 0; i < 8; i += 2) {
        digitalWrite(motorPins[i], HIGH);
        digitalWrite(motorPins[i + 1], LOW);
    }
}

// 오른쪽 바퀴 전진, 왼쪽 바퀴 후진
void turnLeft() {
    digitalWrite(FR_IN1, HIGH);
    digitalWrite(FR_IN2, LOW);
    digitalWrite(RR_IN1, HIGH);
    digitalWrite(RR_IN2, LOW);

    digitalWrite(FL_IN1, LOW);
    digitalWrite(FL_IN2, HIGH);
    digitalWrite(RL_IN1, LOW);
    digitalWrite(RL_IN2, HIGH);
}


// 왼쪽 바퀴 전진, 오른쪽 바퀴 후진
void turnRight() {
    digitalWrite(FR_IN1, LOW);
    digitalWrite(FR_IN2, HIGH);
    digitalWrite(RR_IN1, LOW);
    digitalWrite(RR_IN2, HIGH);

    digitalWrite(FL_IN1, HIGH);
    digitalWrite(FL_IN2, LOW);
    digitalWrite(RL_IN1, HIGH);
    digitalWrite(RL_IN2, LOW);
}

// LiDAR가 low byte, high byte 순서로 보낸 2바이트를 하나의 16비트 정수로 합침
uint16_t readUInt16(uint8_t low, uint8_t high) {
    return low | (high << 8);
}

// 지정한 각도 범위에서 가장 가까운 유효 거리값을 찾음
uint16_t getMinDistance(int startDeg, int endDeg) {
    uint16_t minDist = 9999;

    for (int degree = startDeg; degree <= endDeg; degree++) {
        // 0은 유효하지 않은 거리값으로 보고 제외한다
        if (distances[degree] > 0 && distances[degree] < minDist) {
            minDist = distances[degree];
        }
    }

    return minDist;
}

// 전방 장애물을 확인하고 빈 공간이 넓은 쪽으로 회전
void avoidObstacle() {
    uint16_t frontDist = getMinDistance(150, 210);   // 전방 ±30°
    uint16_t leftDist = getMinDistance(50, 130);     // 좌측 ±40°
    uint16_t rightDist = getMinDistance(230, 310);   // 우측 ±40°

    if (frontDist < SAFE_DISTANCE) {
        Serial.print("장애물 감지! 전방 거리: ");
        Serial.print(frontDist);
        Serial.println(" mm -> 정지 및 경로 탐색");

        stopMotors();
        delay(200);

        // 이미 돌던 중이면 방향을 바꾸지 않고 같은 쪽으로 계속 돌기
        // 새로 마주친 장애물이면 더 트인 쪽을 고름
        if (!wasTurning) {
            if (leftDist > rightDist) {
                turnDirection = 1;
            } else {
                turnDirection = -1;
            }
        }

        if (turnDirection == 1) {
            Serial.println("좌회전 실행");
            turnLeft();
        } else {
            Serial.println("우회전 실행");
            turnRight();
        }

        wasTurning = true;

        delay(400);

        stopMotors();
        delay(100);

        // 다음 한 바퀴는 회전 전 데이터가 섞여 있으므로 판단하지 않음
        skipNextScan = true;
    } else {
        // 앞이 트이면 다음 장애물은 새로 판단
        wasTurning = false;
        moveForward();
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(230400, SERIAL_8N1, LIDAR_RX, -1);

    for (int i = 0; i < 8; i++) {
        pinMode(motorPins[i], OUTPUT);
        digitalWrite(motorPins[i], LOW);
    }
}

void loop() {
    if (!Serial2.available()) {
        return;
    }

    // packet의 첫 byte는 항상 0x54
    if (Serial2.read() != 0x54) {
        return;
    }

    // 두 번째 byte가 들어올 때까지 기다림
    while (!Serial2.available()) {
    }

    // 두 번째 byte가 0x2C인지 확인
    if (Serial2.read() != 0x2C) {
        return;
    }

    packet[0] = 0x54;
    packet[1] = 0x2C;

    // header 2바이트를 제외한 나머지 packet 데이터를 읽음
    for (int i = 2; i < 47; i++) {
        while (!Serial2.available()) {
        }

        packet[i] = Serial2.read();
    }

    // 시작 각도와 끝 각도는 각각 2바이트
    uint16_t startRaw = readUInt16(packet[4], packet[5]);
    uint16_t endRaw = readUInt16(packet[42], packet[43]);

    // 각도값은 0.01도 단위로 들어오므로 100으로 나눔
    float startAngle = startRaw / 100.0;
    float endAngle = endRaw / 100.0;

    if (endAngle < startAngle) {
        endAngle += 360.0;
    }

    // 한 packet에는 12개 측정점이 있으므로 시작점과 끝점 사이를 11구간으로 나눔
    float angleStep = (endAngle - startAngle) / 11.0;

    for (int i = 0; i < 12; i++) {
        // 각 측정점은 distance 2바이트 + confidence 1바이트로 구성
        int index = 6 + i * 3;

        // 거리값 2바이트를 mm 단위 정수로 변환
        uint16_t distance = readUInt16(packet[index], packet[index + 1]);

        // 현재 측정점이 어느 각도에 해당하는지 계산
        float angle = startAngle + angleStep * i;

        if (angle >= 360.0) {
            angle -= 360.0;
        }

        // 360도에서 0도로 넘어오면 한 바퀴 완료로 보고 장애물 판단
        if (previousAngle > 300.0 && angle < 60.0) {
            // 회전 직후 첫 한 바퀴는 건너뛰고 새로 측정한 한 바퀴로 판단
            if (skipNextScan) {
                skipNextScan = false;
            } else {
                avoidObstacle();
            }
        }

        // float 각도를 가장 가까운 정수 각도로 반올림
        int degree = (int)(angle + 0.5);

        if (degree >= 360) {
            degree = 0;
        }

        // 0은 유효하지 않은 거리값으로 보고 저장하지 않음
        if (distance > 0) {
            distances[degree] = distance;
        }

        previousAngle = angle;
    }
}