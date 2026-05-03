#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Servo.h"

MPU6050 mpu;
Servo servo1, servo2, servo3, servo4, servo5;

// DMP
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// Target sudut = 0 semua (mau posisi datar)
float targetPitch = 0;
float targetRoll  = 0;
float targetYaw   = 0;

// PID state
float errorPitch, errorRoll, errorYaw;
float prevErrorPitch = 0, prevErrorRoll = 0, prevErrorYaw = 0;
float integralPitch  = 0, integralRoll  = 0, integralYaw  = 0;

// PID gains
float kp = 2.0, ki = 0.05, kd = 0.0;

// Anti windup
const float INTEGRAL_LIMIT = 50.0;

// Timing
float dt;
unsigned long lastTime;
const float DT_MIN = 0.001;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 tidak terdeteksi!");
    while (true);
  }

  devStatus = mpu.dmpInitialize();

  // Isi dengan nilai kalibrasi kamu
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  // Pin servo
  servo1.attach(3);  // kanan  (pitch)
  servo2.attach(5);  // kiri   (pitch)
  servo3.attach(6);  // bawah  (yaw)
  servo4.attach(9);  // belakang  (rolling)
  servo5.attach(10); // depan (rolling)

  // Posisi netral semua servo
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
  

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("SIAP!");
  } else {
    Serial.print("DMP GAGAL, kode: ");
    Serial.println(devStatus);
  }

  lastTime = millis();
}

void loop() {
  if (!dmpReady) return;

  // Baca FIFO
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

  // Ambil sudut dari sensor
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yaw   = ypr[0] * 180.0 / M_PI;
  float pitch = ypr[1] * 180.0 / M_PI;
  float roll  = ypr[2] * 180.0 / M_PI;

  // Hitung dt
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt < DT_MIN) dt = DT_MIN;

  // Hitung error
  errorPitch = targetPitch - pitch;
  errorRoll  = targetRoll  - roll;
  errorYaw   = targetYaw   - yaw;

  // Integral + anti windup
  integralPitch += errorPitch * dt;
  integralRoll  += errorRoll  * dt;
  integralYaw   += errorYaw   * dt;
  integralPitch = constrain(integralPitch, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  integralRoll  = constrain(integralRoll,  -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  integralYaw   = constrain(integralYaw,   -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Derivative
  float dPitch = (errorPitch - prevErrorPitch) / dt;
  float dRoll  = (errorRoll  - prevErrorRoll)  / dt;
  float dYaw   = (errorYaw   - prevErrorYaw)   / dt;

  // Output PID
  float outPitch = kp*errorPitch + ki*integralPitch + kd*dPitch;
  float outRoll  = kp*errorRoll  + ki*integralRoll  + kd*dRoll;
  float outYaw   = kp*errorYaw   + ki*integralYaw   + kd*dYaw;

  prevErrorPitch = errorPitch;
  prevErrorRoll  = errorRoll;
  prevErrorYaw   = errorYaw;

  // Mapping servo sesuai posisi fisik
  float s1 = 90 + outRoll;   // kanan   → koreksi heaving
  float s2 = 90 - outRoll;   // kiri    → berlawanan dengan kanan
  float s3 = 90 + outYaw;    // bawah   → koreksi yaw
  float s4 = 90 + outPitch;  // depan   → koreksi rolling
  float s5 = 90 - outPitch;  // belakang → berlawanan dengan depan

  s1 = constrain(s1, 65, 155);
  s2 = constrain(s2, 42, 120);
  s3 = constrain(s3, 70, 180);
  s4 = constrain(s4, 65, 130);
  s5 = constrain(s5, 20, 125);

  servo1.write((int)s1);
  servo2.write((int)s2);
  servo3.write((int)s3);
  servo4.write((int)s4);
  servo5.write((int)s5);

  // Debug Serial Monitor
  Serial.print("Pitch: "); Serial.print(pitch, 1);
  Serial.print("  Roll: ");  Serial.print(roll,  1);
  Serial.print("  Yaw: ");   Serial.print(yaw,   1);
  Serial.print("  | S1:"); Serial.print((int)s1);
  Serial.print(" S2:"); Serial.print((int)s2);
  Serial.print(" S3:"); Serial.print((int)s3);
  Serial.print(" S4:"); Serial.print((int)s4);
  Serial.print(" S5:"); Serial.println((int)s5);
}
