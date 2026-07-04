// ═══════════════════════════════════════════════════════════════
//  UEL Rover - 아두이노 모터 제어
//
//  [통신]
//  Jetson Nano UART → 아두이노 Serial (115200 baud)
//  수신 프로토콜: "L<-255~255>,R<-255~255>\n"
//  예시: "L128,R-200\n"  →  왼쪽 128 / 오른쪽 -200
//
//  [모터 드라이버 핀 설정]
//  아래 핀 번호를 실제 배선에 맞게 수정하세요
//
//  L298N 기준:
//    ENA → 좌측 모터 PWM
//    IN1, IN2 → 좌측 모터 방향
//    ENB → 우측 모터 PWM
//    IN3, IN4 → 우측 모터 방향
//
//  DRV8833 기준:
//    AIN1, AIN2 → 좌측 모터 (PWM 겸용)
//    BIN1, BIN2 → 우측 모터 (PWM 겸용)
//    → 하단 setMotorDRV8833() 함수 참고
// ═══════════════════════════════════════════════════════════════

// ── 핀 설정 (실제 배선에 맞게 수정) ─────────────────────────────
// L298N / L293D 방식
#define LEFT_PWM   5    // ENA : 좌측 속도 (PWM 지원 핀)
#define LEFT_IN1   4    // IN1 : 좌측 방향A
#define LEFT_IN2   3    // IN2 : 좌측 방향B

#define RIGHT_PWM  10   // ENB : 우측 속도 (PWM 지원 핀)
#define RIGHT_IN1  9    // IN3 : 우측 방향A
#define RIGHT_IN2  8    // IN4 : 우측 방향B

// ── 안전 설정 ─────────────────────────────────────────────────
#define WATCHDOG_MS  500   // 이 시간(ms) 동안 명령 없으면 정지
#define BAUD_RATE    115200

// ──────────────────────────────────────────────────────────────

String inputBuffer = "";
unsigned long lastCmdTime = 0;

void setup() {
    Serial.begin(BAUD_RATE);

    pinMode(LEFT_PWM,  OUTPUT);
    pinMode(LEFT_IN1,  OUTPUT);
    pinMode(LEFT_IN2,  OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);

    stopAll();
    Serial.println("[UEL Rover] 모터 제어 준비 완료");
}

void loop() {
    // 시리얼 수신
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            parseCommand(inputBuffer);
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }

    // 워치독: 명령이 끊기면 안전 정지
    if (millis() - lastCmdTime > WATCHDOG_MS) {
        stopAll();
    }
}

// ── 명령 파싱: "L<value>,R<value>" ───────────────────────────
void parseCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    int l_idx = cmd.indexOf('L');
    int comma  = cmd.indexOf(',');
    int r_idx  = cmd.indexOf('R');

    if (l_idx < 0 || comma < 0 || r_idx < 0) {
        Serial.println("[오류] 잘못된 명령: " + cmd);
        return;
    }

    int left_val  = cmd.substring(l_idx + 1, comma).toInt();
    int right_val = cmd.substring(r_idx + 1).toInt();

    // 범위 제한
    left_val  = constrain(left_val,  -255, 255);
    right_val = constrain(right_val, -255, 255);

    setMotor(LEFT_PWM,  LEFT_IN1,  LEFT_IN2,  left_val);
    setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, right_val);

    lastCmdTime = millis();
}

// ── 모터 제어 (L298N / L293D 방식) ──────────────────────────
void setMotor(int pwm_pin, int in1, int in2, int value) {
    int spd = abs(value);

    if (value > 0) {
        // 전진
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (value < 0) {
        // 후진
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        // 정지 (브레이크)
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }

    analogWrite(pwm_pin, spd);
}

// ── 전체 정지 ────────────────────────────────────────────────
void stopAll() {
    setMotor(LEFT_PWM,  LEFT_IN1,  LEFT_IN2,  0);
    setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, 0);
}

// ════════════════════════════════════════════════════════════════
//  [DRV8833 사용 시 대체 함수]
//  setMotor() 대신 아래 함수를 사용
//  AIN1/AIN2, BIN1/BIN2 각각 PWM 신호를 넣는 방식
//
//  void setMotorDRV8833(int in1, int in2, int value) {
//      int spd = abs(value);
//      if (value > 0) {
//          analogWrite(in1, spd);
//          analogWrite(in2, 0);
//      } else if (value < 0) {
//          analogWrite(in1, 0);
//          analogWrite(in2, spd);
//      } else {
//          analogWrite(in1, 0);
//          analogWrite(in2, 0);
//      }
//  }
// ════════════════════════════════════════════════════════════════
