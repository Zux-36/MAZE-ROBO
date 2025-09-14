#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <NewPing.h>

// Chân điều khiển L298N
#define IN1 5  // PWM, Motor A (bánh phải)
#define IN2 6  // PWM, Motor A (bánh phải)
#define IN3 11 // PWM, Motor B (bánh trái)
#define IN4 10 // PWM, Motor B (bánh trái)

// PWM tốc độ
const int basePWM = 80; // Tốc độ cơ bản (0-255)
const int turnPWM = 100; // Tốc độ khi quay
int rightPWM = basePWM;  // PWM bánh phải
int leftPWM = basePWM;   // PWM bánh trái

// Chân cảm biến siêu âm
#define TRIG_PIN_RIGHT A0  // Cảm biến bên phải (Trig)
#define ECHO_PIN_RIGHT A1  // Cảm biến bên phải (Echo)
#define TRIG_PIN_FRONT A2  // Cảm biến phía trước (Trig)
#define ECHO_PIN_FRONT A3  // Cảm biến phía trước (Echo)
#define MAX_DISTANCE 200   // Khoảng cách tối đa (cm)

// Khởi tạo cảm biến siêu âm
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

// Khởi tạo MPU6050
Adafruit_MPU6050 mpu;

// Biến toàn cục
const int wallDistance = 10; // Khoảng cách tới tường (cm)
float yaw = 0;               // Góc yaw hiện tại (độ)
float targetYaw = 0;         // Góc mục tiêu
float gyroBias = 0;          // Bias của gyro
unsigned long lastTime = 0;  // Thời gian cập nhật trước
const float yawTolerance = 2; // Dung sai góc (độ)
const float kp = 5.0;        // Hệ số PID
const float ki = 0.02;       // Hệ số tích phân
const float kd = 1.0;        // Hệ số vi phân
float error = 0;             // Sai lệch góc
float lastError = 0;         // Sai lệch trước
float integral = 0;          // Thành phần tích phân
bool canTurnRight = true;    // Cờ cho phép rẽ phải

// Hàm điều khiển động cơ
void moveForward() {
  analogWrite(IN1, rightPWM);
  analogWrite(IN2, 0);
  analogWrite(IN3, leftPWM);
  analogWrite(IN4, 0);
}

void turnLeft() {
  analogWrite(IN1, turnPWM);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, turnPWM);
}

void turnRight() {
  analogWrite(IN1, 0);
  analogWrite(IN2, turnPWM);
  analogWrite(IN3, turnPWM);
  analogWrite(IN4, 0);
}

void moveBackward() {
  analogWrite(IN1, 0);
  analogWrite(IN2, basePWM);
  analogWrite(IN3, 0);
  analogWrite(IN4, basePWM);
}

void stopMotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

// Hàm đọc khoảng cách cảm biến siêu âm
int getDistance(NewPing &sonar) {
  int distance = sonar.ping_cm();
  return distance == 0 ? MAX_DISTANCE : distance;
}

// Hàm khởi tạo MPU6050
void initMPU6050() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 Found!");
}

// Hàm hiệu chỉnh bias gyro
void calibrateGyro() {
  Serial.println("Calibrating gyro... Keep MPU still!");
  float gyroSum = 0;
  int samples = 500;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroSum += g.gyro.z * 180 / PI;
    delay(10);
  }
  gyroBias = gyroSum / samples;
  Serial.print("Gyro Bias: ");
  Serial.print(gyroBias, 4);
  Serial.println(" deg/s");
}

// Hàm cập nhật yaw
void updateYaw() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  yaw += (g.gyro.z * 180 / PI - gyroBias) * dt;
  lastTime = currentTime;
}

// Hàm PID đi thẳng
void adjustStraight() {
  updateYaw();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  error = yaw - targetYaw; // Lệch trái: error > 0, lệch phải: error < 0
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float correction = kp * error + ki * integral + kd * derivative;

  // Sửa lỗi: Lệch trái (error > 0) -> rẽ phải (tăng leftPWM, giảm rightPWM)
  rightPWM = constrain(basePWM - correction, 0, 255); // Giảm bánh phải
  leftPWM = constrain(basePWM + correction, 0, 255);  // Tăng bánh trái
  lastError = error;

  moveForward();
}

// Hàm điều chỉnh tại chỗ
void adjustToTargetYaw() {
  updateYaw();
  while (abs(yaw - targetYaw) > yawTolerance) {
    updateYaw();
    error = yaw - targetYaw;
    if (error > 0) { // Lệch trái -> xoay phải
      turnRight();
    } else { // Lệch phải -> xoay trái
      turnLeft();
    }
  }
  stopMotors();
}

// Hàm quay phải 90°
void turnRight90() {
  updateYaw();
  float startYaw = yaw;
  targetYaw = startYaw - 90;
  while (abs(yaw - targetYaw) > yawTolerance) {
    updateYaw();
    turnRight();
  }
  stopMotors();
  targetYaw = yaw; // Reset gốc tọa độ mới
}

// Hàm quay trái 90°
void turnLeft90() {
  updateYaw();
  float startYaw = yaw;
  targetYaw = startYaw + 90;
  while (abs(yaw - targetYaw) > yawTolerance) {
    updateYaw();
    turnLeft();
  }
  stopMotors();
  targetYaw = yaw; // Reset gốc tọa độ mới
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to initialize
  }
  Serial.println("Starting Maze Bot on Arduino Nano...");

  // Cấu hình chân động cơ
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Wire.begin(); // Initialize I2C (A4: SDA, A5: SCL)

  // Check I2C connection
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: MPU6050 not found!");
    while (1);
  }

  initMPU6050();
  calibrateGyro();
  lastTime = millis();
  stopMotors();

  Serial.println("Setup complete.");
}

void loop() {
  // Đọc khoảng cách
  int frontDistance = getDistance(sonarFront);
  int rightDistance = getDistance(sonarRight);

  // Cập nhật yaw
  updateYaw();

  // In thông tin
  Serial.print("Yaw: "); Serial.print(yaw, 2);
  Serial.print(" | TargetYaw: "); Serial.print(targetYaw, 2);
  Serial.print(" | Front: "); Serial.print(frontDistance);
  Serial.print(" | Right: "); Serial.print(rightDistance);
  Serial.println();

  // Thuật toán bám tường bên phải
  if (rightDistance > wallDistance && canTurnRight) {
    // Mất tường bên phải
    stopMotors();
    delay(500); // Dừng 0.5s
    adjustToTargetYaw(); // Điều chỉnh tại chỗ
    delay(500);
    turnRight90(); // Quay phải 90°
    delay(500); // Dừng 0.5s
    targetYaw = yaw; // Reset gốc tọa độ
    canTurnRight = false; // Đặt cờ
  } else if (rightDistance <= wallDistance && frontDistance > wallDistance) {
    // Bên phải có tường, phía trước thoáng
    canTurnRight = true; // Bật cờ
    adjustStraight(); // Đi thẳng PID
  } else if (rightDistance <= wallDistance && frontDistance <= wallDistance) {
    // Bên phải và phía trước có tường
    stopMotors();
    delay(500); // Dừng 0.5s
    adjustToTargetYaw(); // Điều chỉnh tại chỗ
    delay(500);
    turnLeft90(); // Quay trái 90°
    delay(500); // Dừng 0.5s
    moveBackward(); // Lùi 1.5s
    delay(1500);
    stopMotors();
    delay(500); // Dừng 0.5s
    targetYaw = yaw; // Reset gốc tọa độ
  }

  delay(50); // Tốc độ vòng lặp
}