// ═══════════════════════════════════════════════════════════════
//  UEL Rover — OpenRB-150 micro-ROS 펌웨어
//  송신기: FlySky / 수신기: FS-iA10B (개별 PWM)
//
//  [필요 라이브러리] Arduino IDE Library Manager에서 설치
//    - DynamixelShield   (ROBOTIS)
//    - micro_ros_arduino (Jazzy zip 수동 설치)
//      https://github.com/micro-ROS/micro_ros_arduino/releases
//
//  [RC 배선] FS-iA10B 개별 채널 → OpenRB-150 디지털 핀
//    CH1 (조향)   → RC_CH1_PIN (핀 1)
//    CH2 (스로틀) → RC_CH2_PIN (핀 2)
//    CH5 (모드)   → RC_CH5_PIN (핀 5)
//    GND          → GND (공통)
//    ※ VCC는 수신기 별도 전원 또는 OpenRB-150 5V 핀
//
//  [PWM 신호 범위]
//    최소 : 1000 µs
//    중립 : 1500 µs
//    최대 : 2000 µs
//
//  [다이나믹셀]
//    OpenRB-150 TTL 포트에 직결
//    좌측 모터 ID : DXL_LEFT_ID
//    우측 모터 ID : DXL_RIGHT_ID
//
//  [micro-ROS 토픽]
//    subscribe: /cmd_vel_out (geometry_msgs/Twist) ← Jetson AUTO 명령
//    publish:   /wheel_velocity (Float32MultiArray [vL_rpm, vR_rpm])
//               /drive_mode    (String "RC"|"AUTO"|"STOP")
// ═══════════════════════════════════════════════════════════════

#include "DynamixelShield.h"
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

// ── 다이나믹셀 설정 ────────────────────────────────────────────
#define DXL_LEFT_ID   1        // 좌측 모터 ID (확인 후 수정)
#define DXL_RIGHT_ID  2        // 우측 모터 ID (확인 후 수정)
#define DXL_BAUDRATE  1000000
#define DXL_PROTOCOL  2.0f

// 우측 모터 장착 방향 반전 (로버 구조에 따라 조정)
#define RIGHT_MOTOR_INVERTED true

// ── RC 핀 설정 ────────────────────────────────────────────────
#define RC_CH1_PIN  6    // CH1: 조향   (FS-iA10B CH1 신호선)
#define RC_CH2_PIN  7    // CH2: 스로틀 (FS-iA10B CH2 신호선)
#define RC_CH5_PIN  8    // CH5: 모드   (FS-iA10B CH5 신호선)

// ── RC PWM 범위 및 모드 임계값 ────────────────────────────────
#define RC_MIN       1000
#define RC_MID       1500
#define RC_MAX       2000
#define RC_DEAD_ZONE   50   // 중립 데드존 ±50µs

#define RC_MODE_HIGH 1700   // ch5 >= 1700µs → RC 모드
#define RC_MODE_LOW  1300   // ch5 <= 1300µs → AUTO 모드

// ── 속도 설정 ─────────────────────────────────────────────────
#define MAX_RPM       50.0f   // 최대 RPM (다이나믹셀 모델에 따라 조정)
#define WHEEL_BASE    0.30f   // [m] 좌우 바퀴 간격
#define WHEEL_RADIUS  0.065f  // [m] 바퀴 반지름

// ── 안전 설정 ─────────────────────────────────────────────────
#define WATCHDOG_MS   500     // AUTO 명령 끊기면 정지
#define PUB_PERIOD_MS 20      // 발행 주기 50Hz

// ── 드라이브 모드 ─────────────────────────────────────────────
// 아두이노 IDE는 첫 함수 앞에 함수 프로토타입을 자동 삽입한다.
// getDriveMode()가 이 enum을 반환하므로, enum은 반드시 첫 함수보다
// 위에 정의되어야 프로토타입이 타입을 인식할 수 있다.
enum DriveMode { MODE_RC, MODE_AUTO, MODE_STOP };

// ──────────────────────────────────────────────────────────────
//  RC PWM 인터럽트 (CHANGE 방식 — 상승/하강 엣지 모두 감지)
// ──────────────────────────────────────────────────────────────
volatile uint16_t rc_ch1 = RC_MID;
volatile uint16_t rc_ch2 = RC_MID;
volatile uint16_t rc_ch5 = RC_MIN;

volatile uint32_t ch1_rise, ch2_rise, ch5_rise;

void ISR_CH1() {
    if (digitalRead(RC_CH1_PIN)) {
        ch1_rise = micros();
    } else {
        uint16_t pw = (uint16_t)(micros() - ch1_rise);
        if (pw >= 900 && pw <= 2100) rc_ch1 = pw;
    }
}
void ISR_CH2() {
    if (digitalRead(RC_CH2_PIN)) {
        ch2_rise = micros();
    } else {
        uint16_t pw = (uint16_t)(micros() - ch2_rise);
        if (pw >= 900 && pw <= 2100) rc_ch2 = pw;
    }
}
void ISR_CH5() {
    if (digitalRead(RC_CH5_PIN)) {
        ch5_rise = micros();
    } else {
        uint16_t pw = (uint16_t)(micros() - ch5_rise);
        if (pw >= 900 && pw <= 2100) rc_ch5 = pw;
    }
}

// ── RC 값 → 정규화 (-1.0 ~ +1.0), 데드존 적용 ───────────────
float rcToNorm(uint16_t us) {
    int centered = (int)us - RC_MID;
    if (abs(centered) < RC_DEAD_ZONE) return 0.0f;
    return constrain((float)centered / (float)(RC_MAX - RC_MID), -1.0f, 1.0f);
}

// ──────────────────────────────────────────────────────────────
//  micro-ROS
// ──────────────────────────────────────────────────────────────
rcl_node_t       node;
rcl_allocator_t  allocator;
rclc_support_t   support;
rclc_executor_t  executor;

rcl_subscription_t cmd_sub;
rcl_publisher_t    vel_pub;
rcl_publisher_t    mode_pub;

geometry_msgs__msg__Twist         cmd_msg;
std_msgs__msg__Float32MultiArray  vel_msg;
std_msgs__msg__String             mode_msg;

float vel_data[2] = {0.0f, 0.0f};
char  mode_buf[8];

bool          agent_connected  = false;
unsigned long last_cmd_ms      = 0;
unsigned long last_pub_ms      = 0;

float target_left_rpm  = 0.0f;
float target_right_rpm = 0.0f;

DynamixelShield dxl;

// ──────────────────────────────────────────────────────────────
//  /cmd_vel_out 콜백 (AUTO 모드)
// ──────────────────────────────────────────────────────────────
void cmdCallback(const void * msg_in) {
    const geometry_msgs__msg__Twist * twist =
        (const geometry_msgs__msg__Twist *)msg_in;

    float v = twist->linear.x;
    float w = twist->angular.z;

    float v_left  = v - w * WHEEL_BASE / 2.0f;
    float v_right = v + w * WHEEL_BASE / 2.0f;

    target_left_rpm  = constrain(
        v_left  / (2.0f * M_PI * WHEEL_RADIUS) * 60.0f, -MAX_RPM, MAX_RPM);
    target_right_rpm = constrain(
        v_right / (2.0f * M_PI * WHEEL_RADIUS) * 60.0f, -MAX_RPM, MAX_RPM);

    last_cmd_ms = millis();
}

// ──────────────────────────────────────────────────────────────
//  다이나믹셀 제어
// ──────────────────────────────────────────────────────────────
void setWheelVelocity(float left_rpm, float right_rpm) {
    if (RIGHT_MOTOR_INVERTED) right_rpm = -right_rpm;
    dxl.setGoalVelocity(DXL_LEFT_ID,  left_rpm,  UNIT_RPM);
    dxl.setGoalVelocity(DXL_RIGHT_ID, right_rpm, UNIT_RPM);
}

void stopMotors() {
    dxl.setGoalVelocity(DXL_LEFT_ID,  0.0f, UNIT_RPM);
    dxl.setGoalVelocity(DXL_RIGHT_ID, 0.0f, UNIT_RPM);
}

// ──────────────────────────────────────────────────────────────
//  드라이브 모드 판별 (enum 정의는 파일 상단 참고)
// ──────────────────────────────────────────────────────────────
DriveMode getDriveMode() {
    if (rc_ch5 >= RC_MODE_HIGH) return MODE_RC;
    if (rc_ch5 <= RC_MODE_LOW)  return MODE_AUTO;
    return MODE_STOP;
}

// ──────────────────────────────────────────────────────────────
//  micro-ROS 초기화 / 해제
// ──────────────────────────────────────────────────────────────
bool initMicroROS() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "openrb150", "", &support) != RCL_RET_OK) return false;

    rclc_subscription_init_default(
        &cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel_out");

    vel_msg.data.data     = vel_data;
    vel_msg.data.size     = 2;
    vel_msg.data.capacity = 2;
    rclc_publisher_init_default(
        &vel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/wheel_velocity");

    mode_msg.data.data     = mode_buf;
    mode_msg.data.capacity = sizeof(mode_buf);
    rclc_publisher_init_default(
        &mode_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/drive_mode");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(
        &executor, &cmd_sub, &cmd_msg, &cmdCallback, ON_NEW_DATA);

    return true;
}

void destroyMicroROS() {
    rcl_subscription_fini(&cmd_sub, &node);
    rcl_publisher_fini(&vel_pub,  &node);
    rcl_publisher_fini(&mode_pub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ──────────────────────────────────────────────────────────────
//  setup
// ──────────────────────────────────────────────────────────────
void setup() {
    // RC 핀 인터럽트
    pinMode(RC_CH1_PIN, INPUT);
    pinMode(RC_CH2_PIN, INPUT);
    pinMode(RC_CH5_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RC_CH1_PIN), ISR_CH1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RC_CH2_PIN), ISR_CH2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RC_CH5_PIN), ISR_CH5, CHANGE);

    // 다이나믹셀
    dxl.begin(DXL_BAUDRATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL);
    dxl.setOperatingMode(DXL_LEFT_ID,  OP_VELOCITY);
    dxl.setOperatingMode(DXL_RIGHT_ID, OP_VELOCITY);
    dxl.torqueOn(DXL_LEFT_ID);
    dxl.torqueOn(DXL_RIGHT_ID);

    // micro-ROS (USB 기본 시리얼 트랜스포트)
    set_microros_transports();
    delay(2000);
}

// ──────────────────────────────────────────────────────────────
//  loop
// ──────────────────────────────────────────────────────────────
void loop() {
    // micro-ROS Agent 연결 관리
    if (!agent_connected) {
        stopMotors();
        if (initMicroROS()) {
            agent_connected = true;
        } else {
            delay(500);
            return;
        }
    }

    if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)) != RCL_RET_OK) {
        agent_connected = false;
        destroyMicroROS();
        stopMotors();
        return;
    }

    // ── 모드별 모터 제어 ──────────────────────────────────────
    DriveMode mode = getDriveMode();

    if (mode == MODE_RC) {
        float throttle = rcToNorm(rc_ch2);  // CH2: 스로틀
        float steer    = rcToNorm(rc_ch1);  // CH1: 조향

        float left_rpm  = (throttle - steer) * MAX_RPM;
        float right_rpm = (throttle + steer) * MAX_RPM;
        setWheelVelocity(left_rpm, right_rpm);

    } else if (mode == MODE_AUTO) {
        if (millis() - last_cmd_ms > WATCHDOG_MS) {
            stopMotors();
        } else {
            setWheelVelocity(target_left_rpm, target_right_rpm);
        }

    } else {
        stopMotors();
    }

    // ── 50Hz 발행 ────────────────────────────────────────────
    if (millis() - last_pub_ms >= PUB_PERIOD_MS) {
        float vl = dxl.getPresentVelocity(DXL_LEFT_ID,  UNIT_RPM);
        float vr = dxl.getPresentVelocity(DXL_RIGHT_ID, UNIT_RPM);
        if (RIGHT_MOTOR_INVERTED) vr = -vr;

        vel_data[0] = vl;
        vel_data[1] = vr;
        rcl_publish(&vel_pub, &vel_msg, NULL);

        const char * mode_str = (mode == MODE_RC)  ? "RC"   :
                                (mode == MODE_AUTO) ? "AUTO" : "STOP";
        strncpy(mode_buf, mode_str, sizeof(mode_buf) - 1);
        mode_msg.data.size = strlen(mode_buf);
        rcl_publish(&mode_pub, &mode_msg, NULL);

        last_pub_ms = millis();
    }
}
