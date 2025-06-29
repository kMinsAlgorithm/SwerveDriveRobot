#include <PID_v1.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h> // 문자열 초기화를 위해 추가

// PID 제어를 위한 변수 선언
double input1, output1, setpoint1, normalized_speed1;
double input2, output2, setpoint2, normalized_speed2;
double input3, output3, setpoint3, normalized_speed3;

// PID 객체 생성 (Kp, Ki, Kd 값을 원하는 대로 조정)
PID motorPID1(&input1, &output1, &setpoint1, 2.0, 5.0, 1.0, DIRECT);
PID motorPID2(&input2, &output2, &setpoint2, 2.0, 5.0, 1.0, DIRECT);
PID motorPID3(&input3, &output3, &setpoint3, 2.0, 5.0, 1.0, DIRECT);

unsigned long pwm_value1 = 0;
unsigned long pwm_value2 = 0;
unsigned long pwm_value3 = 0;
unsigned long prev_time1 = 0;
unsigned long prev_time2 = 0;
unsigned long prev_time3 = 0;

// 엔코더 값 저장 시간 업데이트용
long lastTime = 0;

// wheel encoder (en25)

// ----------- 첫번째 엔코더 -----------------
int encoderPin1_A = 41; // 첫 번째 엔코더 A 핀
int encoderPin2_A = 40; // 첫 번째 엔코더 B 핀

long distance_A = 0;
float speed_A = 0; // 속도 (mm/s)
float angularVelocity_A = 0; // 각속도 (rad/s)

long lastEncoderValue_A = 0;
volatile int lastEncoded_A = 0;
long encoderValue_A = 0;

// ----------- 두번째 엔코더 -----------------
int encoderPin1_B = 42; // 두 번째 엔코더 A 핀
int encoderPin2_B = 43; // 두 번째 엔코더 B 핀

long distance_B = 0;
float speed_B = 0; // 속도 (mm/s)
float angularVelocity_B = 0; // 각속도 (rad/s)

long lastEncoderValue_B= 0;
volatile int lastEncoded_B = 0;
long encoderValue_B = 0;

// ----------- 세번째 엔코더 -----------------
int encoderPin1_C = 45; // 세 번째 엔코더 A 핀
int encoderPin2_C = 44; // 세 번째 엔코더 B 핀

long distance_C = 0;
float speed_C = 0; // 속도 (mm/s)
float angularVelocity_C = 0; // 각속도 (rad/s)


long lastEncoderValue_C = 0;
volatile int lastEncoded_C = 0;
long encoderValue_C = 0;

// angle encoder (an25)
int pinPWMINPUT1 = 10;
int pinPWMINPUT2 = 11;
int pinPWMINPUT3 = 12;

long angle1 = 0;
long angle2 = 0;
long angle3 = 0;

long targetAngle1 = 0;
long targetAngle2 = 0;
long targetAngle3 = 0;

int wheel1Speed = 0,wheel2Speed = 0, wheel3Speed = 0; 

long pinPWMINPUTmin = 96;
long pinPWMINPUTmax = 876;

int motorDirPin1 = 22;
int motorPwmPin1 = 7;

int motorDirPin2 = 23;
int motorPwmPin2 = 8;

int motorDirPin3 = 24;
int motorPwmPin3 = 9;

// 휠 모터 핀 설정
const int motor1VspPin = 4, motor1DirPin = 25, motor1EnPin = 26;
const int motor2VspPin = 5, motor2DirPin = 27, motor2EnPin = 28;
const int motor3VspPin = 6, motor3DirPin = 29, motor3EnPin = 30;

// 휠 모터 최대 최솟값 설정
int minWheelMotorSpeed = 100;
int maxWheelMotorSpeed = 220;

// 조인트 모터 최대 최솟값 설정
int minMotorSpeed = 80;
int maxMotorSpeed = 120;

bool motor1Reached = false;
bool motor2Reached = false;
bool motor3Reached = false;

// 로봇의 위치와 방향 (yaw)
float posX = 0.0;
float posY = 0.0;
float yaw = 0.0;

float wheelbase = 0.3;
// for cmd_Vel
float wheel_radius = 0.07;
float wheelRadius = wheel_radius / 2.0; // 바퀴 반지름(mm)
float wheelCircumference = wheel_radius * 3.1416; // 바퀴 둘레(mm)

// for angular calc
float wheelDiameter_ang = 70.2; // 바퀴 지름(mm)
float wheelRadius_ang = wheelDiameter_ang / 2.0; // 바퀴 반지름(mm)
float wheelCircumference_ang = wheelDiameter_ang * 3.1416; // 바퀴 둘레(mm)

bool motorDirState1 = HIGH; // HIGH: 정방향, LOW: 역방향
bool motorDirState2 = HIGH;
bool motorDirState3 = HIGH;

// micro-ros 변수
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
// 조인트 상태 발행을 위한 메시지 생성
rcl_publisher_t joint_state_publisher;
sensor_msgs__msg__JointState joint_state_msg;
// 각 조인트 이름 정의
unsigned long publish_interval = 100;
unsigned long last_publish_time = 0;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

typedef struct {
    float angle;
    float normalized_speed;
} AngleSpeed;

typedef struct {
    long angle1;
    long angle2;
    long angle3;
} Angles;

typedef struct {
    long angularVelocity_A;
    long angularVelocity_B;
    long angularVelocity_C;
} Velocities;


void error_loop() {
  // 모든 모터 정지
  // stopAllMotors();

  // LED 깜빡거림
  while (1) {
      // PID 제어 중지
    motorPID1.SetMode(MANUAL);
    motorPID2.SetMode(MANUAL);
    motorPID3.SetMode(MANUAL);

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1000);
    // Serial.println("error");
    stopAllMotors();

  }
}

// 모든 모터를 정지시키는 함수
void stopAllWheelMotors() {
  // 휠 모터 핀에도 0 값을 전달하여 정지
  analogWrite(motor1VspPin, 0);
  analogWrite(motor2VspPin, 0);
  analogWrite(motor3VspPin, 0);
  
  // 휠 모터 방향 핀도 LOW로 설정
  digitalWrite(motor1DirPin, LOW);
  digitalWrite(motor2DirPin, LOW);
  digitalWrite(motor3DirPin, LOW);
}

void stopAllMotors() {
  // 각 모터 핀에 0 값을 전달하여 모터 정지
  analogWrite(motorPwmPin1, 0);
  analogWrite(motorPwmPin2, 0);
  analogWrite(motorPwmPin3, 0);

  // 휠 모터 핀에도 0 값을 전달하여 정지
  analogWrite(motor1VspPin, 0);
  analogWrite(motor2VspPin, 0);
  analogWrite(motor3VspPin, 0);

  // 모터 방향 핀도 안전을 위해 LOW로 설정
  digitalWrite(motorDirPin1, LOW);
  digitalWrite(motorDirPin2, LOW);
  digitalWrite(motorDirPin3, LOW);
  
  // 휠 모터 방향 핀도 LOW로 설정
  digitalWrite(motor1DirPin, LOW);
  digitalWrite(motor2DirPin, LOW);
  digitalWrite(motor3DirPin, LOW);
}
// Twist message callback
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // cmd_vel 메시지에서 linear_x, linear_y, angular_z 값을 추출하여 모터 각도 계산
  // float linear_x = - msg->linear.y;
  // float linear_y = msg->linear.x;
  float linear_x =  msg->linear.x;
  float linear_y = - msg->linear.y;
  float angular_z = msg->angular.z;

  calculateAndStoreAngles(linear_x, linear_y, angular_z);
}

void init_joint_state_msg() {
    joint_state_msg.name.data = (rosidl_runtime_c__String *)malloc(6 * sizeof(rosidl_runtime_c__String));
    joint_state_msg.name.capacity = 6;
    joint_state_msg.name.size = 6;

    // Joint names for steering positions
    joint_state_msg.name.data[0].data = (char *)"wheel_1_joint";
    joint_state_msg.name.data[0].size = strlen(joint_state_msg.name.data[0].data);
    joint_state_msg.name.data[0].capacity = strlen(joint_state_msg.name.data[0].data) + 1;

    joint_state_msg.name.data[1].data = (char *)"wheel_2_joint";
    joint_state_msg.name.data[1].size = strlen(joint_state_msg.name.data[1].data);
    joint_state_msg.name.data[1].capacity = strlen(joint_state_msg.name.data[1].data) + 1;

    joint_state_msg.name.data[2].data = (char *)"wheel_3_joint";
    joint_state_msg.name.data[2].size = strlen(joint_state_msg.name.data[2].data);
    joint_state_msg.name.data[2].capacity = strlen(joint_state_msg.name.data[2].data) + 1;

    // Joint names for wheel drive positions
    joint_state_msg.name.data[3].data = (char *)"wheel_1_drive_joint";
    joint_state_msg.name.data[3].size = strlen(joint_state_msg.name.data[3].data);
    joint_state_msg.name.data[3].capacity = strlen(joint_state_msg.name.data[3].data) + 1;

    joint_state_msg.name.data[4].data = (char *)"wheel_2_drive_joint";
    joint_state_msg.name.data[4].size = strlen(joint_state_msg.name.data[4].data);
    joint_state_msg.name.data[4].capacity = strlen(joint_state_msg.name.data[4].data) + 1;

    joint_state_msg.name.data[5].data = (char *)"wheel_3_drive_joint";
    joint_state_msg.name.data[5].size = strlen(joint_state_msg.name.data[5].data);
    joint_state_msg.name.data[5].capacity = strlen(joint_state_msg.name.data[5].data) + 1;

    // Initialize positions and velocities for six joints
    joint_state_msg.position.data = (double *)malloc(6 * sizeof(double));
    joint_state_msg.position.size = 6;
    joint_state_msg.position.capacity = 6;

    joint_state_msg.velocity.data = (double *)malloc(6 * sizeof(double));
    joint_state_msg.velocity.size = 6;
    joint_state_msg.velocity.capacity = 6;
}


void setup() {
  // micro-ros 초기화
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // QoS 프로필 생성
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_profile.depth = 10;  // 큐 크기를 10으로 설정

  // cmd_vel 토픽에 대한 subscriber 생성 (QoS 설정 적용)
  RCCHECK(rclc_subscription_init(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel",
      &qos_profile));

  // JointState 발행 초기화
  RCCHECK(rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states"));
  init_joint_state_msg();
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // PID 제어기 모드 설정
  motorPID1.SetMode(AUTOMATIC);
  motorPID2.SetMode(AUTOMATIC);
  motorPID3.SetMode(AUTOMATIC);

  Serial.begin(230400);

  // Serial.begin(115200);
  // wheel encoder (en25)
  pinMode(encoderPin1_A, INPUT); 
  pinMode(encoderPin2_A, INPUT);
  pinMode(encoderPin1_B, INPUT); 
  pinMode(encoderPin2_B, INPUT);
  pinMode(encoderPin1_C, INPUT); 
  pinMode(encoderPin2_C, INPUT);
  
  // angle encoder (an25)
  pinMode(pinPWMINPUT1, INPUT);
  pinMode(pinPWMINPUT2, INPUT);
  pinMode(pinPWMINPUT3, INPUT);

  // joint motor
  pinMode(motorDirPin1, OUTPUT);
  pinMode(motorPwmPin1, OUTPUT);

  pinMode(motorDirPin2, OUTPUT);
  pinMode(motorPwmPin2, OUTPUT);

  pinMode(motorDirPin3, OUTPUT);
  pinMode(motorPwmPin3, OUTPUT);

  // wheel motor
  pinMode(motor1VspPin, OUTPUT);
  pinMode(motor1DirPin, OUTPUT);
  pinMode(motor1EnPin, OUTPUT);

  pinMode(motor2VspPin, OUTPUT);
  pinMode(motor2DirPin, OUTPUT);
  pinMode(motor2EnPin, OUTPUT);

  pinMode(motor3VspPin, OUTPUT);
  pinMode(motor3DirPin, OUTPUT);
  pinMode(motor3EnPin, OUTPUT);

  // bldc 제어를 위해서 EN 핀이 LOW상태여야 함.
  digitalWrite(motor1EnPin, LOW);
  digitalWrite(motor2EnPin, LOW);
  digitalWrite(motor3EnPin, LOW);

  // --------------- angle encoder --------------------
  attachInterrupt(digitalPinToInterrupt(pinPWMINPUT1), rising1, RISING);
  attachInterrupt(digitalPinToInterrupt(pinPWMINPUT2), rising2, RISING);
  attachInterrupt(digitalPinToInterrupt(pinPWMINPUT3), rising3, RISING);

  // --------------- wheel encoder --------------------
  // 첫 번째 엔코더 인터럽트
  attachInterrupt(digitalPinToInterrupt(encoderPin1_A), updateEncoder_A, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderPin2_A), updateEncoder_A, CHANGE);

  // 두 번째 엔코더 인터럽트
  attachInterrupt(digitalPinToInterrupt(encoderPin1_B), updateEncoder_B, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderPin2_B), updateEncoder_B, CHANGE);

  // 세 번째 엔코더 인터럽트
  attachInterrupt(digitalPinToInterrupt(encoderPin1_C), updateEncoder_C, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderPin2_C), updateEncoder_C, CHANGE);

}

void loop() {
  // micro-ros Executor 실행
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  long currentTime = millis();
  long deltaTime = currentTime - lastTime; // 시간 변화(ms)
  
  // 각도 및 속도 계산
  Angles angles = getAngles();
  Velocities velocities = getVelocities(deltaTime);
  lastTime = currentTime;
  // 조인트 상태 발행
  // publishJointStates(angles, velocities);
  // 퍼블리시 주기가 지났는지 확인 후 오도메트리 퍼블리시
  if (currentTime - last_publish_time >= publish_interval) {
    // 오도메트리 계산 (각 바퀴의 속도 및 각도를 기반으로) 및 퍼블리시
    // calculateOdometry(velocities, angles, deltaTime / 1000.0);
    last_publish_time = currentTime;
    publishJointStates(angles, velocities, deltaTime);
   
  }

  // 각도 제어 로직 실행
  controlMotorDirections();
  // controlWheelMotor();
}
void publishJointStates(Angles angles, Velocities velocities, long deltaTime) {
    // 각도를 라디안으로 변환하여 조인트 상태에 저장
    joint_state_msg.position.data[0] = angles.angle1 * (M_PI / 180.0);  // Steering position of wheel 1 (in radians)
    joint_state_msg.position.data[1] = angles.angle2 * (M_PI / 180.0);  // Steering position of wheel 2 (in radians)
    joint_state_msg.position.data[2] = angles.angle3 * (M_PI / 180.0);  // Steering position of wheel 3 (in radians)

    // 각속도(rad/s)를 시간에 따라 적분하여 각도(라디안) 계산
    static double wheel1_position = 0.0;  // 초기 위치 값
    static double wheel2_position = 0.0;  // 초기 위치 값
    static double wheel3_position = 0.0;  // 초기 위치 값

    double delta_time_sec = deltaTime / 1000.0;  // ms를 초 단위로 변환

    // 각속도를 적분하여 각도를 구하고, 이를 position에 누적
    wheel1_position += velocities.angularVelocity_A * delta_time_sec;
    wheel2_position += velocities.angularVelocity_B * delta_time_sec;
    wheel3_position += velocities.angularVelocity_C * delta_time_sec;

    // 각도를 조인트 상태에 저장 (적분된 값)
    joint_state_msg.position.data[3] = wheel1_position;  // Wheel 1 position
    joint_state_msg.position.data[4] = wheel2_position;  // Wheel 2 position
    joint_state_msg.position.data[5] = wheel3_position;  // Wheel 3 position

    // 각속도(rad/s)를 조인트 상태에 저장
    joint_state_msg.velocity.data[3] = velocities.angularVelocity_A / 6;  // Wheel 1 velocity
    joint_state_msg.velocity.data[4] = velocities.angularVelocity_B / 6;  // Wheel 2 velocity
    joint_state_msg.velocity.data[5] = velocities.angularVelocity_C / 6;  // Wheel 3 velocity

    // 타임스탬프 설정
    joint_state_msg.header.stamp.sec = millis() / 1000;
    joint_state_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

    // JointState 메시지 발행
    rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
}

Angles getAngles() { // 만약 각도 정보를 얻어서 루프 문에서 뭔갈 하게 된다면 아래 주석을 해제하고 본 함수의 리턴을 Angles로 변경하면 됨.
  angle1 = (pinPWMINPUTmax - pwm_value1) * 360 / (pinPWMINPUTmax - pinPWMINPUTmin);
  angle2 = (pinPWMINPUTmax - pwm_value2) * 360 / (pinPWMINPUTmax - pinPWMINPUTmin);
  angle3 = (pinPWMINPUTmax - pwm_value3) * 360 / (pinPWMINPUTmax - pinPWMINPUTmin);

  if (angle1 > 360) angle1 = 0;
  if (angle2 > 360) angle2 = 0;
  if (angle3 > 360) angle3 = 0;

  Angles result = {angle1, angle2, angle3};
  return result;
}

Velocities getVelocities(long deltaTime) {
  float deltaTimeSeconds = deltaTime / 1000.0;

  // 첫 번째 엔코더 각속도 계산
  float deltaAngle_A = (encoderValue_A - lastEncoderValue_A) * 2 * 3.1416 / 40.0; // 각도 변화량을 라디안으로 변환
  angularVelocity_A = deltaAngle_A / deltaTimeSeconds; // 각속도 (rad/s)

  // 두 번째 엔코더 각속도 계산
  float deltaAngle_B = (encoderValue_B - lastEncoderValue_B) * 2 * 3.1416 / 40.0; // 각도 변화량을 라디안으로 변환
  angularVelocity_B = deltaAngle_B / deltaTimeSeconds; // 각속도 (rad/s)

  // 세 번째 엔코더 각속도 계산
  float deltaAngle_C = (encoderValue_C - lastEncoderValue_C) * 2 * 3.1416 / 40.0; // 각도 변화량을 라디안으로 변환
  angularVelocity_C = deltaAngle_C / deltaTimeSeconds; // 각속도 (rad/s)

  // 엔코더 값 갱신
  lastEncoderValue_A = encoderValue_A;
  lastEncoderValue_B = encoderValue_B;
  lastEncoderValue_C = encoderValue_C;

  Velocities result = {angularVelocity_A, angularVelocity_B, angularVelocity_C};
  return result;
}

void calculateAndStoreAngles(float linear_y, float linear_x, float angular_z) {
  float wx2 = cos(radians(30));
  float wy2 = sin(radians(30));
  float wx3 = cos(radians(270));
  float wy3 = sin(radians(270));
  float wx1 = cos(radians(150));
  float wy1 = sin(radians(150));

  // 각 바퀴의 속도 성분 계산
  float Vx_wheel1 = linear_x - angular_z * wy1 * wheelbase;
  float Vy_wheel1 = linear_y + angular_z * wx1 * wheelbase;

  float Vx_wheel2 = linear_x - angular_z * wy2 * wheelbase;
  float Vy_wheel2 = linear_y + angular_z * wx2 * wheelbase;

  float Vx_wheel3 = linear_x - angular_z * wy3 * wheelbase;
  float Vy_wheel3 = linear_y + angular_z * wx3 * wheelbase;
  
  // 바퀴 속도 계산 (속도 벡터의 크기)
  float wheel_speed1 = sqrt(Vx_wheel1 * Vx_wheel1 + Vy_wheel1 * Vy_wheel1);
  float wheel_speed2 = sqrt(Vx_wheel2 * Vx_wheel2 + Vy_wheel2 * Vy_wheel2);
  float wheel_speed3 = sqrt(Vx_wheel3 * Vx_wheel3 + Vy_wheel3 * Vy_wheel3);

  normalized_speed1 = wheel_speed1 / wheel_radius;  // 속도를 각속도로 변환
  normalized_speed2 = wheel_speed2 / wheel_radius;
  normalized_speed3 = wheel_speed3 / wheel_radius;
    
  targetAngle1 = atan2(Vy_wheel1, Vx_wheel1) * 180 / PI;
  targetAngle2 = atan2(Vy_wheel2, Vx_wheel2) * 180 / PI;
  targetAngle3 = atan2(Vy_wheel3, Vx_wheel3) * 180 / PI;

  // Serial.print(" | before target Angles: ");
  // Serial.print(targetAngle1);
  // Serial.print(", ");
  // Serial.print(targetAngle2);
  // Serial.print(", ");
  // Serial.println(targetAngle3);
  
  AngleSpeed joint1 = mapAngleToRange(targetAngle1, normalized_speed1);
  targetAngle1 = joint1.angle;
  normalized_speed1 = joint1.normalized_speed * -1;

  AngleSpeed joint2 = mapAngleToRange(targetAngle2, normalized_speed2);
  targetAngle2 = joint2.angle;
  normalized_speed2 = joint2.normalized_speed;

  AngleSpeed joint3 = mapAngleToRange(targetAngle3, normalized_speed3);
  targetAngle3 = joint3.angle;
  normalized_speed3 = joint3.normalized_speed;

  // Serial.print(" | Stored target Angles: ");
  // Serial.print(targetAngle1);
  // Serial.print(", ");
  // Serial.print(targetAngle2);
  // Serial.print(", ");
  // Serial.println(targetAngle3);
}

AngleSpeed mapAngleToRange(float angle, float normalized_speed) {
    angle = fmod(angle, 360);
    if (angle < 0) angle += 360;

    if (angle == 360) {
        angle = 0;
    } else if (angle > 179) {
        angle = angle - 180;
        normalized_speed *= -1 ;
    }

    AngleSpeed result = {angle, normalized_speed};
    return result;
}
void controlWheelMotor() {
    // 각 바퀴의 속도에 대해 개별적으로 처리
    if (abs(angularVelocity_A) > 360) {
        analogWrite(motor1VspPin, 0);  // 모터 1 정지
    } else {
        wheel1Speed = applyWheelMotorControl(normalized_speed1, motor1DirPin, motor1VspPin, wheel1Speed);
        analogWrite(motor1VspPin, wheel1Speed);  // 모터 1 PWM 적용
    }

    if (abs(angularVelocity_B) > 360) {
        analogWrite(motor2VspPin, 0);  // 모터 2 정지
    } else {
        wheel2Speed = applyWheelMotorControl(normalized_speed2, motor2DirPin, motor2VspPin, wheel2Speed);
        analogWrite(motor2VspPin, wheel2Speed);  // 모터 2 PWM 적용
    }

    if (abs(angularVelocity_C) > 360) {
        analogWrite(motor3VspPin, 0);  // 모터 3 정지
    } else {
        wheel3Speed = applyWheelMotorControl(normalized_speed3, motor3DirPin, motor3VspPin, wheel3Speed);
        analogWrite(motor3VspPin, wheel3Speed);  // 모터 3 PWM 적용
    }
}


int applyWheelMotorControl(long normalized_speed, int wheelMotorDirPin, int wheelMotorVspPin, int &wheelSpeed){
  // 모터 속도 설정
  if (normalized_speed == 0) {
    // 속도가 0이면 모터를 정지
    // analogWrite(wheelMotorVspPin, 0);
    // wheelSpeed = 0;
    return 0; // 함수 종료
  }

  if (normalized_speed > 0) {
    digitalWrite(wheelMotorDirPin, LOW);
  } else {
    digitalWrite(wheelMotorDirPin, HIGH);
  }
    
  // PWM 범위 내 속도로 normalize
  int pwm_speed = map(abs(normalized_speed), 1.0, 20.0, minWheelMotorSpeed, maxWheelMotorSpeed); // 1-20.0 범위에서 속도를 매핑
  pwm_speed = constrain(pwm_speed, minWheelMotorSpeed, maxWheelMotorSpeed); // 속도를 제한
  
  return pwm_speed;
  //wheelSpeed = pwm_speed;
//   Serial.print("PWM Speed: ");
//   Serial.println(wheelSpeed);
}

void controlMotorDirections() {
    // 각도와 목표각도를 PID 입력 값과 목표 값으로 설정
    input1 = angle1;
    setpoint1 = targetAngle1;
    input2 = angle2;
    setpoint2 = targetAngle2;
    input3 = angle3;
    setpoint3 = targetAngle3;

    unsigned long currentTime = millis();
    if (currentTime - prev_time1 > 100) {  // 100ms마다 PID 제어 수행
        // PID 계산 수행
        motorPID1.Compute();
        motorPID2.Compute();
        motorPID3.Compute();
        prev_time1 = currentTime;
    }

    // 모터 방향 및 속도 제어
    motor1Reached = applyMotorControl(output1, angle1, targetAngle1, motorDirPin1, motorPwmPin1);
    motor2Reached = applyMotorControl(output2, angle2, targetAngle2, motorDirPin2, motorPwmPin2);
    motor3Reached = applyMotorControl(output3, angle3, targetAngle3, motorDirPin3, motorPwmPin3);

    // cmd_vel 선속도가 0이면서 각속도가 있을 경우 조인트 각도에 도달한 후 바퀴 모터를 움직임
    if (msg.linear.x == 0.0 && msg.linear.y == 0.0 && msg.angular.z != 0.0) {
        if (motor1Reached && motor2Reached && motor3Reached) {
            controlWheelMotor();  // 바퀴 모터 동작
        } else {
            stopAllWheelMotors();  // 바퀴 모터 정지
        }
    } else {
        controlWheelMotor();  // 선속도가 0이 아니면 정상적으로 바퀴 제어
    }
}


bool applyMotorControl(double output, long currentAngle, long targetAngle, int motorDirPin, int motorPwmPin) {
  long angleError = targetAngle - currentAngle;
  if (angleError > 180) {
    angleError -= 360;
  } else if (angleError < -180) {
    angleError += 360;
  }
  
  if (abs(angleError) <= 5) {
    analogWrite(motorPwmPin, 0); // 오차가 3도 이내이면 모터를 정지
    return true;
  } else {
    if (angleError > 0) {
      digitalWrite(motorDirPin, LOW);  // 정방향
    } else {
      digitalWrite(motorDirPin, HIGH);  // 역방향
    }
    analogWrite(motorPwmPin, constrain(output, minMotorSpeed, maxMotorSpeed)); // PID 출력값을 모터 속도로 사용
    return false;
  }
}

// 엔코더 입력을 처리하는 인터럽트 핸들러들
void rising1() {
  attachInterrupt(digitalPinToInterrupt(pinPWMINPUT1), falling1, FALLING);
  prev_time1 = micros();
}

void falling1() {
  attachInterrupt(digitalPinToInterrupt(pinPWMINPUT1), rising1, RISING);
  pwm_value1 = micros() - prev_time1;
}

void rising2() {
  attachInterrupt(digitalPinToInterrupt(pinPWMINPUT2), falling2, FALLING);
  prev_time2 = micros();
}

void falling2() {
  attachInterrupt(digitalPinToInterrupt(pinPWMINPUT2), rising2, RISING);
  pwm_value2 = micros() - prev_time2;
}

void rising3() {
  attachInterrupt(digitalPinToInterrupt(pinPWMINPUT3), falling3, FALLING);
  prev_time3 = micros();
}

void falling3() {
  attachInterrupt(digitalPinToInterrupt(pinPWMINPUT3), rising3, RISING);
  pwm_value3 = micros() - prev_time3;
}

// 첫 번째 엔코더 업데이트 함수
void updateEncoder_A() {
  int MSB_A = digitalRead(encoderPin1_A); // MSB = most significant bit
  int LSB_A = digitalRead(encoderPin2_A); // LSB = least significant bit

  int encoded_A = (MSB_A << 1) | LSB_A; // 두 핀 값을 하나의 숫자로 변환
  int sum_A = (lastEncoded_A << 2) | encoded_A; // 이전 값과 현재 값을 합침

  if (sum_A == 0b1101 || sum_A == 0b0100 || sum_A == 0b0010 || sum_A == 0b1011) encoderValue_A++;
  if (sum_A == 0b1110 || sum_A == 0b0111 || sum_A == 0b0001 || sum_A == 0b1000) encoderValue_A--;

  lastEncoded_A = encoded_A; // 다음 업데이트를 위해 값 저장
}

// 두 번째 엔코더 업데이트 함수
void updateEncoder_B() {
  int MSB_B = digitalRead(encoderPin1_B); // MSB = most significant bit
  int LSB_B = digitalRead(encoderPin2_B); // LSB = least significant bit

  int encoded_B = (MSB_B << 1) | LSB_B; // 두 핀 값을 하나의 숫자로 변환
  int sum_B = (lastEncoded_B << 2) | encoded_B; // 이전 값과 현재 값을 합침

  if (sum_B == 0b1101 || sum_B == 0b0100 || sum_B == 0b0010 || sum_B == 0b1011) encoderValue_B++;
  if (sum_B == 0b1110 || sum_B == 0b0111 || sum_B == 0b0001 || sum_B == 0b1000) encoderValue_B--;

  lastEncoded_B = encoded_B; // 다음 업데이트를 위해 값 저장
}

// 세 번째 엔코더 업데이트 함수
void updateEncoder_C() {
  int MSB_C = digitalRead(encoderPin1_C); // MSB = most significant bit
  int LSB_C = digitalRead(encoderPin2_C); // LSB = least significant bit

  int encoded_C = (MSB_C << 1) | LSB_C; // 두 핀 값을 하나의 숫자로 변환
  int sum_C = (lastEncoded_C << 2) | encoded_C; // 이전 값과 현재 값을 합침

  if (sum_C == 0b1101 || sum_C == 0b0100 || sum_C == 0b0010 || sum_C == 0b1011) encoderValue_C++;
  if (sum_C == 0b1110 || sum_C == 0b0111 || sum_C == 0b0001 || sum_C == 0b1000) encoderValue_C--;

  lastEncoded_C = encoded_C; // 다음 업데이트를 위해 값 저장
}
