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
//
//  [Agent 자동 재연결]
//    WAITING → AVAILABLE → CONNECTED → DISCONNECTED → WAITING 상태 머신.
//    rmw_uros_ping_agent()로 Agent 생존을 주기적으로 확인하므로,
//    micro_ros_agent를 재시작해도 OpenRB 물리 리셋 없이 다시 붙는다.
//    RC/STOP 모터 제어는 이 상태 머신보다 먼저·무조건 실행된다.
// ═══════════════════════════════════════════════════════════════

#include "DynamixelShield.h"
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>   // rmw_uros_ping_agent 등 Agent 생존 확인 API
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
// 발행 전 present velocity 검증 임계값. dxl.getPresentVelocity()가 통신
// 실패/타임아웃 시 검증 없이 이상값을 반환하는 것을 걸러낸다. 물리적으로
// |RPM|은 MAX_RPM을 크게 넘을 수 없으므로 2배 여유를 둔다. 젯슨 쪽
// odom 가드(max_wheel_rpm=100)와 값을 맞춘다.
#define MAX_PLAUSIBLE_RPM (2.0f * MAX_RPM)
#define WHEEL_BASE    0.30f   // [m] 좌우 바퀴 간격
#define WHEEL_RADIUS  0.065f  // [m] 바퀴 반지름

// ── 안전 설정 ─────────────────────────────────────────────────
#define WATCHDOG_MS   500     // AUTO 명령 끊기면 정지
#define PUB_PERIOD_MS 20      // 발행 주기 50Hz

// ── Agent 재연결 설정 ─────────────────────────────────────────
// 모든 값은 millis() 기반 비동기 주기 제어에 쓰인다. loop 전체를
// 멈추는 delay()는 사용하지 않는다 (RC/STOP 응답성 보장).
#define PING_TIMEOUT_MS      50   // ping 1회 최대 대기 (loop 블로킹 상한)
#define PING_ATTEMPTS         1   // ping 1회당 시도 횟수 (블로킹 시간 = TIMEOUT×ATTEMPTS)
#define PING_WAIT_PERIOD_MS 500   // 미연결 상태에서 Agent 탐색 주기
#define PING_ALIVE_PERIOD_MS 200  // 연결 상태에서 생존 확인 주기
#define PING_FAIL_LIMIT       3   // 연속 실패 이 횟수부터 단절 판정 (순간 오류로 끊지 않음)

// entity destroy 시 세션 타임아웃을 0으로 만들어, Agent가 이미 죽은
// 상태에서 fini 호출이 오래 블로킹되는 것을 막는다.
// ※ rmw_uros_set_context_entity_destroy_session_timeout() 이 컴파일되지
//    않으면 이 값을 0으로 바꿀 것. 기능 저하는 "정리가 느려짐"뿐이다.
#define USE_DESTROY_SESSION_TIMEOUT 1

// ── 드라이브 모드 ─────────────────────────────────────────────
// 아두이노 IDE는 첫 함수 앞에 함수 프로토타입을 자동 삽입한다.
// getDriveMode()가 이 enum을 반환하므로, enum은 반드시 첫 함수보다
// 위에 정의되어야 프로토타입이 타입을 인식할 수 있다.
enum DriveMode { MODE_RC, MODE_AUTO, MODE_STOP };

// ── Agent 연결 상태 ───────────────────────────────────────────
// 위와 같은 이유로 첫 함수보다 위에 둔다.
//   WAITING      : Agent 없음. 주기적으로 ping만 던진다.
//   AVAILABLE    : ping 성공. 이번 loop에서 entity를 만든다.
//   CONNECTED    : entity 정상. spin/발행 수행 + 주기적 생존 확인.
//   DISCONNECTED : 단절 확정. 안전 정지 + entity 정리 후 WAITING 복귀.
enum AgentState {
    AGENT_WAITING,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

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

// ── Agent 연결 상태 (정적 변수만 사용, 동적 할당 없음) ────────
AgentState    agent_state      = AGENT_WAITING;
unsigned long last_ping_ms     = 0;
uint8_t       ping_fail_count  = 0;

// 생성된 entity 단계. 실패 시 여기까지만 되돌린다(부분 초기화 상태에서
// 잘못된 fini를 호출하지 않기 위함).
//   0:없음 1:support 2:node 3:sub 4:vel_pub 5:mode_pub 6:executor
uint8_t       entity_level     = 0;

// 재연결 이후 새 /cmd_vel_out을 실제로 받았는지. false면 AUTO는 정지.
// millis() 롤오버 구간에서도 확실히 판정되도록 시간 비교 대신 플래그를 쓴다.
bool          cmd_received     = false;

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

    last_cmd_ms  = millis();
    cmd_received = true;
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
//  micro-ROS entity 생성 / 해제
//
//  생성은 모든 단계의 반환값을 검사하고, 중간 실패 시 entity_level에
//  기록된 만큼만 되감는다. 예전 initMicroROS()는 support 생성 성공 후
//  node 생성이 실패하면 support를 해제하지 않고 빠져나가 재시도마다
//  자원이 새는 구조였다.
// ──────────────────────────────────────────────────────────────
bool createEntities() {
    entity_level = 0;
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    entity_level = 1;

    if (rclc_node_init_default(&node, "openrb150", "", &support) != RCL_RET_OK) return false;
    entity_level = 2;

    if (rclc_subscription_init_default(
            &cmd_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel_out") != RCL_RET_OK) return false;
    entity_level = 3;

    vel_msg.data.data     = vel_data;
    vel_msg.data.size     = 2;
    vel_msg.data.capacity = 2;
    if (rclc_publisher_init_default(
            &vel_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/wheel_velocity") != RCL_RET_OK) return false;
    entity_level = 4;

    mode_msg.data.data     = mode_buf;
    mode_msg.data.size     = 0;
    mode_msg.data.capacity = sizeof(mode_buf);
    if (rclc_publisher_init_default(
            &mode_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/drive_mode") != RCL_RET_OK) return false;
    entity_level = 5;

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
    entity_level = 6;

    if (rclc_executor_add_subscription(
            &executor, &cmd_sub, &cmd_msg, &cmdCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

    return true;
}

void destroyEntities() {
    if (entity_level == 0) return;   // 만든 것이 없으면 fini도 없다

#if USE_DESTROY_SESSION_TIMEOUT
    // Agent가 이미 사라진 상태에서 fini가 응답을 기다리며 오래 멈추는 것을
    // 막는다. support가 만들어진 뒤에만 context 접근이 유효하다.
    if (entity_level >= 1) {
        rmw_context_t * rmw_ctx = rcl_context_get_rmw_context(&support.context);
        if (rmw_ctx != NULL) {
            (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
        }
    }
#endif

    // 생성 역순으로, 만들어진 단계까지만 해제
    if (entity_level >= 6) rclc_executor_fini(&executor);
    if (entity_level >= 5) rcl_publisher_fini(&mode_pub, &node);
    if (entity_level >= 4) rcl_publisher_fini(&vel_pub,  &node);
    if (entity_level >= 3) rcl_subscription_fini(&cmd_sub, &node);
    if (entity_level >= 2) rcl_node_fini(&node);
    if (entity_level >= 1) rclc_support_fini(&support);

    entity_level = 0;
}

// AUTO 명령 상태 초기화. 재연결 직후 예전 명령이 되살아나지 않게 한다.
void resetAutoCommand() {
    target_left_rpm  = 0.0f;
    target_right_rpm = 0.0f;
    last_cmd_ms      = 0;
    cmd_received     = false;
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

    // 목표속도를 먼저 0으로 쓰고 나서 토크를 건다. Goal Velocity는 RAM
    // 레지스터라 MCU만 리셋되고 서보 전원이 유지된 경우 직전 값이 남아
    // 있다. 순서를 반대로 하면 토크가 걸리는 순간 이전 속도로 돌아간다.
    stopMotors();
    dxl.torqueOn(DXL_LEFT_ID);
    dxl.torqueOn(DXL_RIGHT_ID);

    // micro-ROS (USB 기본 시리얼 트랜스포트)
    // 여기서는 세션을 맺지 않는다. 연결은 loop()의 상태 머신이 담당하며,
    // Agent가 나중에 떠도 물리 리셋 없이 자동으로 붙는다.
    set_microros_transports();
    delay(2000);

    agent_state     = AGENT_WAITING;
    last_ping_ms    = millis();
    ping_fail_count = 0;
    entity_level    = 0;
    resetAutoCommand();
}

// ──────────────────────────────────────────────────────────────
//  loop
// ──────────────────────────────────────────────────────────────
void loop() {
    // ── 모드별 모터 제어 (micro-ROS 연결 상태와 무관하게 매 루프 실행) ──
    //    RC/STOP은 데드맨 스위치라 ROS/에이전트 연결 여부와 상관없이
    //    항상 동작해야 한다. 아래 micro-ROS 연결 관리 블록에서 초기화
    //    실패 시 return하던 예전 구조는 이 블록 자체를 건너뛰게 만들어
    //    에이전트가 안 붙어 있으면 RC 조종까지 막히는 버그가 있었다.
    DriveMode mode = getDriveMode();

    if (mode == MODE_RC) {
        float throttle = rcToNorm(rc_ch2);  // CH2: 스로틀
        float steer    = rcToNorm(rc_ch1);  // CH1: 조향

        float left_rpm  = (throttle - steer) * MAX_RPM;
        float right_rpm = (throttle + steer) * MAX_RPM;
        setWheelVelocity(left_rpm, right_rpm);

    } else if (mode == MODE_AUTO) {
        // 세 조건이 모두 참일 때만 명령 적용:
        //   ① Agent 연결됨  ② 재연결 후 새 명령을 받음  ③ 워치독 이내
        if (agent_state != AGENT_CONNECTED ||
            !cmd_received ||
            millis() - last_cmd_ms > WATCHDOG_MS) {
            stopMotors();
        } else {
            setWheelVelocity(target_left_rpm, target_right_rpm);
        }

    } else {
        stopMotors();
    }

    // ── micro-ROS Agent 연결 상태 머신 ────────────────────────
    //  위 모터 제어는 이미 끝났다. 아래 어디에도 loop 전체를 멈추는
    //  delay()나 조기 return이 없으므로, Agent 유무와 무관하게 RC/STOP
    //  제어 주기가 유지된다 (커밋 2c2b358의 의도 유지·강화).
    switch (agent_state) {

    case AGENT_WAITING:
        // 매 loop가 아니라 정해진 주기에만 ping. 블로킹은 최대
        // PING_TIMEOUT_MS × PING_ATTEMPTS 로 제한된다.
        if (millis() - last_ping_ms >= PING_WAIT_PERIOD_MS) {
            last_ping_ms = millis();
            if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) == RMW_RET_OK) {
                agent_state = AGENT_AVAILABLE;
            }
        }
        break;

    case AGENT_AVAILABLE:
        // ping이 성공한 경우에만 entity 생성을 시도한다.
        if (createEntities()) {
            resetAutoCommand();      // 예전 AUTO 명령 폐기
            ping_fail_count = 0;
            last_ping_ms    = millis();
            last_pub_ms     = millis();
            agent_state     = AGENT_CONNECTED;
        } else {
            destroyEntities();       // 부분 생성분만 정확히 되감기
            last_ping_ms    = millis();   // 즉시 재시도 방지(busy loop 차단)
            agent_state     = AGENT_WAITING;
        }
        break;

    case AGENT_CONNECTED:
        // 주기적 생존 확인. 순간적 1회 실패로는 끊지 않는다.
        if (millis() - last_ping_ms >= PING_ALIVE_PERIOD_MS) {
            last_ping_ms = millis();
            if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) == RMW_RET_OK) {
                ping_fail_count = 0;
            } else if (++ping_fail_count >= PING_FAIL_LIMIT) {
                agent_state = AGENT_DISCONNECTED;
            }
        }

        if (agent_state == AGENT_CONNECTED) {
            // spin_some의 반환값은 단절 판정에 쓰지 않는다. 처리할 데이터가
            // 없을 때도 RCL_RET_OK가 아닌 값이 나올 수 있어, 예전 코드처럼
            // 이를 단절로 보면 멀쩡한 세션을 끊고 entity를 재생성하게 된다.
            // 생존 판정은 위의 ping이 전담한다.
            (void) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

            // ── 50Hz 발행 (entity가 완전히 생성된 뒤에만) ────
            if (entity_level >= 6 && millis() - last_pub_ms >= PUB_PERIOD_MS) {
                float vl = dxl.getPresentVelocity(DXL_LEFT_ID,  UNIT_RPM);
                float vr = dxl.getPresentVelocity(DXL_RIGHT_ID, UNIT_RPM);
                if (RIGHT_MOTOR_INVERTED) vr = -vr;

                // 발생원 방어(근본): 통신 오류로 getPresentVelocity가 NaN이나
                // 물리적으로 불가능한 값을 반환하면 그 표본을 /wheel_velocity로
                // 내보내지 않는다. 그대로 발행하면 젯슨 odom이 수십 km로 발산해
                // slam_toolbox가 거대한 OccupancyGrid를 잡다 OOM으로 죽는다.
                // 이상 표본은 그냥 건너뛴다(직전 odom 유지). 여기서 Serial로
                // 로그를 찍으면 micro-ROS USB 시리얼 트랜스포트를 오염시키므로
                // 절대 출력하지 않는다.
                bool vel_ok = !isnan(vl) && !isnan(vr) &&
                              fabsf(vl) <= MAX_PLAUSIBLE_RPM &&
                              fabsf(vr) <= MAX_PLAUSIBLE_RPM;
                if (vel_ok) {
                    vel_data[0] = vl;
                    vel_data[1] = vr;
                    rcl_publish(&vel_pub, &vel_msg, NULL);
                }

                const char * mode_str = (mode == MODE_RC)  ? "RC"   :
                                        (mode == MODE_AUTO) ? "AUTO" : "STOP";
                strncpy(mode_buf, mode_str, sizeof(mode_buf) - 1);
                mode_msg.data.size = strlen(mode_buf);
                rcl_publish(&mode_pub, &mode_msg, NULL);

                last_pub_ms = millis();
            }
        }
        break;

    case AGENT_DISCONNECTED:
        // AUTO 명령원이 사라졌으므로 즉시 안전 정지.
        // RC 모드였다면 위에서 이미 RC 속도를 실었으므로 덮어쓰지 않는다
        // (덮어쓰면 데드맨 스위치인 RC 조종이 끊긴다).
        resetAutoCommand();
        if (mode == MODE_AUTO) stopMotors();

        destroyEntities();
        ping_fail_count = 0;
        last_ping_ms    = millis();
        agent_state     = AGENT_WAITING;
        break;
    }
}
