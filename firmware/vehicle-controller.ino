//by Yisun Yoo
//This is the final version. DO NOT CHANGE EXCEPT parameters of RUNSTART inside loop()

/*
  this code uses modified PID and odometry to move the differential drive robot with two motors from one point to another point as precise as possible, with a gate to pass through.
  the given sensors are
  1. two AS5600 encoder
  - Operating Voltage: 3.3V
  - Resolution: 12-bit
  - Accuracy: 0.087°
  - Output Method: I2C
  - Interface: GH1.25-4Pin

  2.BNO 055  absolute orientation sensor also with I2C

  The vehicle has wheel radius of 3.0cm, wheel diameter of 6.0cm (with wheel base of 8.1cm)
  The motor is 4015 gimbal motor 
  - wire to wire resistance of 4.8 ohms
  - rated voltage up to 24v and current max up to 4A
  - 11 pole pairs 

  The vehicle runs on eight AA batteries, with 1.2v rated voltage (rechargeble NiMH aa 2800mah)
  the voltage output when fully charged was meausred around 10V (+- 0.5V)
  The code should assume the batteries are always fully charged

  The vehicle has a reference point called measuring point(MP) 
  MP of the vehicle is placed 5cm in front of the vehicle midpoint of the axle connecting the two driving wheels.
  MP is the point that is measured from the target. (goal: MP = target)

  center line is the line connecting the starting point and the end point

  task: 
  1. the vehicle MP is placed on top of the starting point, and the vehicle is set to face the target point
  2. once the button is pressed, the vehicle should start to run
  3. the vehicle should pass though the gate
  4. the gate is placed to the left of the center line, and is placed on the midway between starting point and endpoint
  5. the outer side of the gate is fixed 100cm away from the center line
  6. the distance between the outer side and the inner side of the gate is chosen by us, and should be one of the inpput variable (gate distance < 100cm , the code should aim for going though middle of the gates)
  7. the vehicle should arrive at target point, which is defined by the distance, and target distance is another input variable. the possible distance vary from 700cm to 1000cm
  8. lastly, the vehicle should arrive at the target point at exact time set, which is the last input variable. the possible target time vary from 10 seconds to 20 seconds.
  9. the score is calculated as 100 + Distance Score + Time Score + Bonuses + Run Penalties
    c. Distance Score = 2.0 pt/cm x Vehicle Distance.
    d. Time Score = Absolute value of (Target Time - Run Time)
    e. Can Bonus = -0.5 x (110 - Inside Can Distance)
  10. the less score is better.

  plan:
  1. the vehicle will take three actions: moveStrait(from (y1 to y2 or x1 to x2))); turn(ang) degrees; and final precision movement for vehicle MP to precisely placed on target
*/

#include <Wire.h>
#include <SimpleFOC.h>
#include <AS5600.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

AS5600 as5600;
BNO08x myIMU;

// ---- Motor pins ----
#define M1_U D2
#define M1_V D3
#define M1_W D10
#define M1_EN D9

#define M2_U D7
#define M2_V D6
#define M2_W D5
#define M2_EN D4

// ---- Encoder channels ----
#define M1_CH 2
#define M2_CH 7

// ---- Button ----
#define Button D8
const unsigned long debounce_ms = 50;
bool last_button = HIGH;
unsigned long last_change = 0;

// ---- IMU ----
#define BNO08X_CS A1
#define BNO08X_INT A3
#define BNO08X_RST A2
#define BNO08X_REPORT_INTERVAL_MS 25

struct EulerAngles {
  float yawDeg;
};

// ---- TCA9548A ----
#define TCA_ADDR 0x70
static void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool readMuxedAS5600(uint8_t channel, uint16_t &raw, float &angleDeg, const char *label) {
  tcaSelect(channel);
  raw = as5600.rawAngle();
  const int err = as5600.lastError();
  if (err != AS5600_OK) {
    Serial.print("AS5600 read failed on ");
    Serial.print(label);
    Serial.print(" channel ");
    Serial.print(channel);
    Serial.print(" error ");
    Serial.println(err);
    return false;
  }

  angleDeg = raw * (360.0f / 4096.0f);
  return true;
}

float imuRawYawDeg = 0.0f;
float imuYawOffsetDeg = 0.0f;
bool imuHasOrientation = false;
bool imuReportsEnabled = false;
bool imuDebugPrinted = false;

// ---- Forward direction signs (adjust if needed) ----
const float M1_DIR = -1.0f;   // forward is + for motor1
const float M2_DIR = 1.0f;    // forward is + for motor2

MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_U, M1_V, M1_W, M1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_U, M2_V, M2_W, M2_EN);


// ========================== TUNE ================================= //
const float WHEEL_DIAMETER = 9.0f;
const float WHEEL_RADIUS = WHEEL_DIAMETER / 2.0f;
const float WHEEL_BASE = 8.5f;
const float MIN_VOLTAGE = 0.8f; //min voltage vehicle moves stably
const float max_voltage = 12.0f;

// ---- values ----
float M1_zero_pos = 0.0f;  //deg
float M2_zero_pos = 0.0f;  //deg
float M1_pos = 0.0f;       //rad
float M2_pos = 0.0f;       //rad
float x_pos = 0.0f;        //cm
float y_pos = 0.0f;        //cm
float yaw = 0.0f;          //deg
float startTime = 0.0f; //s
float runTime = 0.0f; //s

// ---- odometry ----
float M1_prev = 0.0f;      //rad
float M2_prev = 0.0f;      //rad
float yaw_prev = 0.0f;     //deg
float deg_prev = PI/2.0f;

uint16_t raw1 = 0, raw2 = 0;
float angle1 = 0, angle1_prev = 0, cumulative_angle1 = 0;
float angle2 = 0, angle2_prev = 0, cumulative_angle2 = 0;

// --- move equal ---
float M1_pos_ref = 0.0f;
float M2_pos_ref = 0.0f;

static inline float wrapPi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

// ================================================ IMU FUNCTIONS ================================================

float clampUnit(float value) {
  if (value > 1.0f) {
    return 1.0f;
  }
  if (value < -1.0f) {
    return -1.0f;
  }
  return value;
}

float wrap180(float deg) {
  while (deg > 180.0f) {
    deg -= 360.0f;
  }
  while (deg < -180.0f) {
    deg += 360.0f;
  }
  return deg;
}

EulerAngles quaternionToEuler(float real, float i, float j, float k) {
  EulerAngles angles;
  const float sinyCosp = 2.0f * (real * k + i * j);
  const float cosyCosp = 1.0f - 2.0f * (j * j + k * k);
  angles.yawDeg = atan2f(sinyCosp, cosyCosp) * RAD_TO_DEG;

  return angles;
}

bool setIMUReports() {
  Serial.println("Setting desired reports");
  if (myIMU.enableGameRotationVector(BNO08X_REPORT_INTERVAL_MS) == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));
    imuReportsEnabled = true;
    imuDebugPrinted = false;
    delay(100);
    return true;
  }

  Serial.println("Could not enable rotation vector");
  imuReportsEnabled = false;
  delay(100);
  return false;
}

void zeroYaw() {
  if (!imuHasOrientation) {
    return;
  }

  imuYawOffsetDeg = imuRawYawDeg;
  yaw = 0.0f;
}

bool beginIMU() {
  Serial.println();
  Serial.println("BNO08x Read Example");

  const bool started = myIMU.beginSPI(BNO08X_CS, BNO08X_INT, BNO08X_RST);

  Serial.print("myIMU.begin: ");
  Serial.println(started ? "OK" : "FAIL");
  if (!started) return false;

  Serial.println("BNO08x found!");
  setIMUReports();

  const unsigned long retry_start_ms = millis();
  unsigned long last_retry_ms = retry_start_ms;
  while (!imuReportsEnabled && (millis() - retry_start_ms) < 1500) {
    if (myIMU.wasReset()) {
      Serial.print("sensor was reset ");
      setIMUReports();
    } else if ((millis() - last_retry_ms) >= 150) {
      last_retry_ms = millis();
      setIMUReports();
    }
    delay(10);
    Serial.println("?");
  }

  if (!imuReportsEnabled) return false;

  Serial.println("Reading events");
  delay(100);

  yaw = 0.0f;
  imuRawYawDeg = 0.0f;
  imuYawOffsetDeg = 0.0f;
  imuHasOrientation = false;

  Serial.println("beginIMU: done");
  return true;
}

float getYawDeg() {
  return yaw;
}
bool updateIMU_latest() {
  if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    if (!setIMUReports()) return false;
  }

  if (!imuReportsEnabled) {
    if (!setIMUReports()) {
      return false;
    }
  }

  // Match the vendor examples: only poll once when INT says data is ready.
  // Repeatedly calling getSensorEvent() until false can force a 500ms timeout
  // inside the library and trigger an IMU reset when no fresh packet is queued.
  if (digitalRead(BNO08X_INT) != LOW) {
    return false;
  }

  if (!myIMU.getSensorEvent()) {
    return false;
  }

  if (myIMU.getSensorEventID() != SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    return false;
  }

  const float qi = myIMU.getGameQuatI();
  const float qj = myIMU.getGameQuatJ();
  const float qk = myIMU.getGameQuatK();
  const float qr = myIMU.getGameQuatReal();
  const EulerAngles a = quaternionToEuler(qr, qi, qj, qk);

  imuRawYawDeg = a.yawDeg;
  if (!imuHasOrientation) { imuHasOrientation = true; zeroYaw(); }

  yaw = wrap180(imuYawOffsetDeg - imuRawYawDeg); // update global yaw

  const bool got = true;
  if (got && !imuDebugPrinted) {
    //Serial.println("IMU stream active");
    imuDebugPrinted = true;
  }
  return got;
}

// ================================================ IMU FUNCTIONS END ================================================

void setupMotor(BLDCMotor &motor, BLDCDriver3PWM &driver, MagneticSensorI2C &sensor, uint8_t ch) {
  //encoder
  tcaSelect(ch);
  sensor.init(&Wire);
  motor.linkSensor(&sensor);
  sensor.min_elapsed_time = 0.001;

  //driver
  driver.voltage_power_supply = 10.6f;
  driver.voltage_limit = 10.0f;
  driver.pwm_frequency = 25000;
  driver.init();
  motor.linkDriver(&driver);
  float max_velocity = 30.0;                       // rad/s
  float motor_frequency_hz = max_velocity / (2 * PI); // ~16 Hz
  float filter_cutoff_hz = motor_frequency_hz * 5;    // ~80 Hz
  motor.LPF_velocity.Tf = 1.0 / (2.0 * PI * filter_cutoff_hz); 

  motor.controller = MotionControlType::velocity; //voltage
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;

  motor.voltage_sensor_align = 3.0f;
  motor.sensor_direction = Direction::UNKNOWN;

  motor.init();
  motor.initFOC();
}

void resetValues() {
  // Encoder 1 snapshot (degrees)
  if (!readMuxedAS5600(M1_CH, raw1, angle1, "M1")) return;
  angle1_prev = angle1;
  cumulative_angle1 = angle1;
  M1_zero_pos = cumulative_angle1;   // NOTE: stored in degrees

  // Encoder 2 snapshot (degrees)
  if (!readMuxedAS5600(M2_CH, raw2, angle2, "M2")) return;
  angle2_prev = angle2;
  cumulative_angle2 = angle2;
  M2_zero_pos = cumulative_angle2;   // NOTE: stored in degrees

  zeroYaw();

  // Reset odometry state
  x_pos = 0.0f;
  y_pos = 0.0f;

  M1_pos = 0.0f;
  M2_pos = 0.0f;
  M1_prev = 0.0f;
  M2_prev = 0.0f;

  // Heading convention:
  // radians: PI/2 = North (+Y)
  // degrees: 0 = North
  deg_prev = PI * 0.5f;
  yaw = 0.0f;
  yaw_prev = 0.0f;

  M1_pos_ref = 0.0f;
  M2_pos_ref = 0.0f;

  startTime = millis() / 1000.0f;
}

void updateOdometry() {

  // ---- Encoder 1 (degrees -> cumulative degrees -> radians) ----
  if (!readMuxedAS5600(M1_CH, raw1, angle1, "M1")) return;

  float delta_angle1 = angle1 - angle1_prev;
  if (delta_angle1 < -180.0f) delta_angle1 += 360.0f;
  if (delta_angle1 >  180.0f) delta_angle1 -= 360.0f;

  cumulative_angle1 += delta_angle1;
  angle1_prev = angle1;

  // M1_zero_pos is in degrees
  M1_pos = (cumulative_angle1 - M1_zero_pos) * (PI / 180.0f) * M1_DIR; // radians


  // ---- Encoder 2 ----
  if (!readMuxedAS5600(M2_CH, raw2, angle2, "M2")) return;

  float delta_angle2 = angle2 - angle2_prev;
  if (delta_angle2 < -180.0f) delta_angle2 += 360.0f;
  if (delta_angle2 >  180.0f) delta_angle2 -= 360.0f;

  cumulative_angle2 += delta_angle2;
  angle2_prev = angle2;

  //yaw calculation
  updateIMU_latest();
  while (yaw < 0.0f) yaw += 360.0f;
  while (yaw >= 360.0f) yaw -= 360.0f;

  // M2_zero_pos is in degrees
  M2_pos = (cumulative_angle2 - M2_zero_pos) * (PI / 180.0f) * M2_DIR; // radians

  // ---- Differential drive odometry ----
  float D_l = radToCm(M1_pos - M1_prev);         // cm
  float D_r = radToCm(M2_pos - M2_prev);         // cm
  float D_avg = 0.5f * (D_l + D_r);              // cm
  float D_yaw_deg = yaw - yaw_prev;
  if (D_yaw_deg > 180.0f) D_yaw_deg -= 360.0f;
  if (D_yaw_deg < -180.0f) D_yaw_deg += 360.0f;
  float D_ang = -D_yaw_deg * PI / 180.0f;   // minus sign: yaw is CW+, math angle is CCW+


  float heading_mid = degToRad(yaw_prev) + 0.5f * D_ang;   // radians, PI/2 is North
  x_pos += D_avg * cosf(heading_mid);            // +X east
  y_pos += D_avg * sinf(heading_mid);            // +Y north

  deg_prev = wrapPi(deg_prev + D_ang);           // radians

  // Convert radians heading to compass-like yaw:
  // yaw = 0 at North, clockwise positive

  M1_prev = M1_pos;
  M2_prev = M2_pos;
  yaw_prev = yaw;
}

float radToCm(float rad){
  return (float) rad * WHEEL_RADIUS;
}
float degToRad(float deg){ //0 deg north to 0rad east (for BNO 055)
  float rad0 = (90-deg) * PI / 180.0f;
  rad0 = fmod(rad0, 2.0f * PI);
  if (rad0 < 0) {
    rad0 += 2.0f * PI;
  }
  return rad0;
}
float rpsToRads(float rps){ //rps to radians per seconds
  return (float) rps*2.0*PI;
}
float RadsTorps(float Rads){
  return (float) Rads/(2.0*PI);
}

void moveEqual(int dir , float base_voltage){ // dir = 1 --> forward. dir = -1 --> rotate
  motor1.controller = MotionControlType::torque; //setting voltage control mode 
  motor2.controller = MotionControlType::torque; //setting voltage control mode 
  motor1.enable();
  motor2.enable();

  float M1_ang_displacement = M1_pos - M1_pos_ref;
  float M2_ang_displacement = M2_pos - M2_pos_ref;

  if(dir == 1){
    motor1.move(M1_DIR * constrain(base_voltage + 5.0f * (M2_ang_displacement - M1_ang_displacement),-12,12)); //voltage
    motor2.move(M2_DIR * constrain(base_voltage + 5.0f * (M1_ang_displacement - M2_ang_displacement),-12,12)); //voltage
  }

  if(dir == -1){
    motor1.move(M1_DIR * constrain(base_voltage - 5.0f * (M2_ang_displacement + M1_ang_displacement),-12,12));
    motor2.move(M2_DIR * constrain(-1.0f * base_voltage - 5.0f * (M2_ang_displacement + M1_ang_displacement),-12,12));
  }

}
//move straight using horizontal difference in odometry
void moveStrait(float x1, float y1, float x2, float y2, float base_rps, bool accelramp, bool decelramp) {
  if (y2 < y1) return;
  if (x1 != x2) return; // vertical only

  motor1.controller = MotionControlType::velocity; 
  motor2.controller = MotionControlType::velocity; 

  motor1.enable();
  motor2.enable();
  bool heading_solved = true;

  const float MAX_RPS = 5.0f;
  base_rps = constrain(base_rps, 0.0f, MAX_RPS);

  unsigned long lastUpdateTime = millis();
  unsigned long currentTime = millis();

  const double interval = 3; // ms
  unsigned long lastFOC_us = 0;
  const unsigned long foc_interval_us = 1000; // ~1kHz

  // Lateral/heading steering (rps domain)
  const float x_P = 0.05f;    // rps/cm
  const float x_I = 0.005f;    // rps/(cm*s)
  const float x_D = 0.01f;    // rps/(cm/s)
  const float yaw_P = 0.021f; // rps/deg
  const float horizontal_I_threshold = 4.0f; // cm

  // Base speed profile
  const float accel_P = 0.02f; // rps/cm^2
  const float decel_P = 0.2f; // rps/cm
  const float min_move_rps = 0.5f;

  // heading barrier by side of centerline
  const float YAW_LIM = 45.0f;   // deg
  const float YAW_LIM_CLOSE = 10.0f;
  const float YAW_LIM_CLOSE_SPECIAL_MULTIPLIER = 4.0f;
  const float excessive_yaw_P = 0.06f; //rps / deg
  const float X_EPS   = 4.0f;    // cm deadband around centerline

  float x_err_prev = 0.0f;
  float i_x_err = 0.0f;

  updateOdometry();
  float y_start = y_pos;

  const unsigned long start_ms = millis();
  const unsigned long timeout_ms = 30000; // safety

  while (fabs(y2 - y_pos) > 1.0f) {
    if (millis() - start_ms > timeout_ms) break;

    unsigned long now_us = micros();
    if ((unsigned long)(now_us - lastFOC_us) >= foc_interval_us) {
      lastFOC_us = now_us;
      tcaSelect(M1_CH); motor1.loopFOC();
      tcaSelect(M2_CH); motor2.loopFOC();
    }

    currentTime = millis();
    if (currentTime - lastUpdateTime >= interval) {
      updateOdometry();

      float yaw_signed = yaw;
      if (yaw_signed > 180.0f) yaw_signed -= 360.0f; // [-180,180]

      float x_err = x_pos - x2; // cm
      float y_err = y2 - y_pos; // cm

      float target_deg = 0.0f; // north
      float yaw_err = wrapPi(degToRad(target_deg) - degToRad(yaw)) * 180.0f / PI; // deg

      float dt = (float)(currentTime - lastUpdateTime) / 1000.0f;
      lastUpdateTime = currentTime;

      if (fabs(x_err) < horizontal_I_threshold) i_x_err += x_err * dt;
      else i_x_err = 0.0f;

      float d_x_err = (x_err - x_err_prev) / dt;
      x_err_prev = x_err;

      float steer_rps = x_P * x_err + x_D * d_x_err + x_I * i_x_err + yaw_P * yaw_err;

      float y_dist = y_pos - y_start;
      float accel_base = constrain(accel_P * ((y_dist + 1.0f) * (y_dist + 1.0f)), min_move_rps, base_rps);
      float decel_base = constrain(decel_P * y_err,  min_move_rps, base_rps);

      float base = base_rps;
      if (accel_P * y_dist < base && accelramp == true) base = accel_base;
      if (decel_P * y_err  < base && decelramp == true) base = decel_base;

      if (x_err > X_EPS && yaw_signed <= -YAW_LIM && steer_rps > 0.0f) {
        // right of line, already too left, and command still turns more left
        float excessive_yaw_err = yaw_signed - (-YAW_LIM);   // negative when too left
        steer_rps = excessive_yaw_P * excessive_yaw_err;     // drives back toward -YAW_LIM
      }

      if (x_err < -X_EPS && yaw_signed >= YAW_LIM && steer_rps < 0.0f) {
        // left of line, already too right, and command still turns more right
        float excessive_yaw_err = yaw_signed - (YAW_LIM);    // positive when too right
        steer_rps = excessive_yaw_P * excessive_yaw_err;     // drives back toward +YAW_LIM
      }

      // normal magnitude limit after directional barrier
      float steer_lim = min(1.2f, 0.3f * base);
      steer_rps = constrain(steer_rps, -steer_lim, steer_lim);

      if((fabs(x_err) < X_EPS && fabs(yaw_signed) > YAW_LIM_CLOSE) || (heading_solved == false)){ //close but too much right or left
        heading_solved = false;
        steer_rps = YAW_LIM_CLOSE_SPECIAL_MULTIPLIER * yaw_P * yaw_err; //too much heading error when close special feedback
        if (fabs(yaw_signed) < YAW_LIM_CLOSE) heading_solved = true;
      }
      float left_rps  = constrain(base - steer_rps, 0.0f, MAX_RPS);
      float right_rps = constrain(base + steer_rps, 0.0f, MAX_RPS);

      motor1.move(rpsToRads(left_rps)  * M1_DIR);
      motor2.move(rpsToRads(right_rps) * M2_DIR);
    }
  }

  motor1.move(0.0f);
  motor2.move(0.0f);
}

//turn to any angle (currently in voltage mode)
void moveTurn(float target_deg){

  motor1.controller = MotionControlType::torque; //setting voltage control mode 
  motor2.controller = MotionControlType::torque; //setting voltage control mode 
  motor1.enable();
  motor2.enable();
  unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();

  const long interval = 5; //ms

  unsigned long lastFOC_us = 0;
  const unsigned long foc_interval_us = 1000; // ~1kHz

  const float Ang_P = 0.07f; // V/deg
  float Ang_I = 0.40f; //  V*s / deg
  const float Ang_D = 0.008f; // V / (deg/s) 0.006

  float deg_err_prev = 0.0f;
  float i_deg_err = 0.0f;
  float d_deg_err = 0.0f;
  float deg_err = 360.0f;

  float voltage_M1 = 0.0f;
  float voltage_M2 = 0.0f;

  M1_pos_ref = M1_pos;
  M2_pos_ref = M2_pos;

  lastUpdateTime = millis();
  lastFOC_us = micros();

  while(fabs(deg_err) > 0.2 || fabs(d_deg_err) > 1){
    unsigned long now_us = micros();
    if ((unsigned long)(now_us - lastFOC_us) >= foc_interval_us) {
      lastFOC_us = now_us;
      tcaSelect(M1_CH);
      motor1.loopFOC();
      tcaSelect(M2_CH);
      motor2.loopFOC();
      
    }
    currentTime = millis();
    if (currentTime - lastUpdateTime >= interval) {

      updateOdometry();

      deg_err = wrapPi(degToRad(target_deg) - degToRad(yaw)) * 180.0f / PI; //positive if vehicle heading right (deg)
      float dt = (float) (currentTime - lastUpdateTime)/1000.0f;
      if (fabs(deg_err) > 10) i_deg_err = 0.0f;
      if (fabs(deg_err) < 10) i_deg_err = (float) i_deg_err + (deg_err * dt);
      d_deg_err = (float) (deg_err - deg_err_prev) / dt;

      deg_err_prev = deg_err;

      float ang_cmd = constrain(Ang_P * deg_err + Ang_I * i_deg_err + Ang_D * d_deg_err,-1.2,1.2);
      moveEqual(-1 , -1.0f * ang_cmd);
      lastUpdateTime = currentTime;
    }
  }
}

void moveSmall(float small_dist) {
  // target_dist in cm (signed, forward positive)

  motor1.controller = MotionControlType::torque; // voltage mode
  motor2.controller = MotionControlType::torque;
  motor1.enable();
  motor2.enable();

  M1_pos_ref = M1_pos;
  M2_pos_ref = M2_pos;

  const unsigned long foc_interval_us = 1000; // 1kHz
  const unsigned long ctrl_interval_ms = 3;
  const unsigned long timeout_ms = 20000UL;

  // Tune these on your robot
  const float KP = 0.9f;         // V/cm
  const float KD = 0.0f;         // V/(cm/s)
  const float MAX_V = 1.4f;       // max command voltage
  const float MIN_V = 0.6f;      // minimum move voltage
  const float DIST_TOL = 0.05f;   // cm
  const float VEL_TOL_CM_S = 0.35f;
  const float SHAFT_VEL_TOL = 0.50f; // rad/s
  const uint8_t SETTLE_CYCLES = 2;

  unsigned long start_ms = millis();
  unsigned long last_ctrl_ms = millis();
  unsigned long last_foc_us = micros();

  float disp_prev = 0.0f;
  uint8_t settle_count = 0;

  while ((millis() - start_ms) < timeout_ms) {
    // FOC service
    unsigned long now_us = micros();
    if ((unsigned long)(now_us - last_foc_us) >= foc_interval_us) {
      last_foc_us = now_us;
      tcaSelect(M1_CH); motor1.loopFOC();
      tcaSelect(M2_CH); motor2.loopFOC();
    }

    // control update
    unsigned long now_ms = millis();
    if ((now_ms - last_ctrl_ms) < ctrl_interval_ms) {
      yield();
      continue;
    }

    float dt = (now_ms - last_ctrl_ms) * 0.001f;
    if (dt < 0.001f) dt = 0.001f;
    last_ctrl_ms = now_ms;

    updateOdometry();

    float disp1 = WHEEL_RADIUS * (M1_pos - M1_pos_ref);  // cm
    float disp2 = WHEEL_RADIUS * (M2_pos - M2_pos_ref);  // cm
    float disp  = 0.5f * (disp1 + disp2);                // cm
    float v_cm_s = (disp - disp_prev) / dt;
    disp_prev = disp;

    float err = small_dist - disp;                      // cm (signed)

    // PD command with signed forward/reverse correction
    float cmd_v = KP * err - KD * v_cm_s;
    cmd_v = constrain(cmd_v, -MAX_V, MAX_V);

    // overcome stiction when still far from target
    if (fabsf(err) > DIST_TOL && fabsf(cmd_v) < MIN_V) {
      cmd_v = (cmd_v >= 0.0f) ? MIN_V : -MIN_V;
    }

    // stop criteria: close enough and slow enough for several cycles
    if (fabsf(err) <= DIST_TOL &&
        fabsf(v_cm_s) <= VEL_TOL_CM_S &&
        fabsf(motor1.shaft_velocity) <= SHAFT_VEL_TOL &&
        fabsf(motor2.shaft_velocity) <= SHAFT_VEL_TOL) {
      settle_count++;
      cmd_v = 0.0f;
    } else {
      settle_count = 0;
    }

    moveEqual(1, cmd_v);

    if (settle_count >= SETTLE_CYCLES) break;
  }

  moveEqual(1, 0.0f);
  motor1.move(0.0f);
  motor2.move(0.0f);
}

void moveFinal(float small_distance, float target_time){
  runTime = millis() / 1000.0f - startTime;
  float time_end_small = 0.0f;
  float time_end_all = 0.0f;
  int loopNum = 0;
  while(runTime + time_end_small*1.5f < target_time){
    runTime = millis() / 1000.0f - startTime;
    float time_end_small_start = millis();
    moveSmall(small_distance);
    moveSmall(-small_distance);
    float time_end_small_end = millis();
    loopNum += 1;
    time_end_all += (time_end_small_end - time_end_small_start)/1000.0f;
    time_end_small = time_end_all / loopNum;
  }
}

void RUNSTART(float target_distance, float target_can_distance, float target_time, int leftOrRight){ //left side can -1
  resetValues();
  delay(300);
  motor1.enable();
  motor2.enable();

  float can_centerLine = leftOrRight * (100.0f - target_can_distance / 2.0f);
  float start_to_can_distance = target_distance / 2.0f;
  moveStrait(can_centerLine, 0, can_centerLine, start_to_can_distance + 10.0f, 4.0f, 1, 0); 
  moveStrait(0.0f, 0.0f, 0.0f, target_distance - 10.0f, 4.0f, 0, 1); 
  moveFinal(1,target_time);
  motor1.disable();
  motor2.disable();
}

void printTelemetry() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 50) return; // 20 Hz max
  lastPrint = millis();

  char line[160];
  snprintf(line, sizeof(line),
           "%.2f / %.2f / %.2f / %.3f / %.2f / %.2f",
           M1_pos * 180.0f / PI,
           M2_pos * 180.0f / PI,
           yaw,
           degToRad(yaw),
           x_pos,
           y_pos);
  Serial.println(line);
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) {Serial.println("?");}

  pinMode(Button, INPUT_PULLUP);

  if (!beginIMU()) {
    Serial.println("BNO08x startup failed.");
    while (1) {
      delay(10);
    }
  }

  Wire.begin(A4, A5);
  Wire.setClock(400000);
  Wire.setTimeOut(20);

  tcaSelect(M1_CH);
  if (!as5600.begin()) {
    Serial.println("AS5600 init failed on M1 channel.");
  }

  setupMotor(motor1, driver1, sensor1, M1_CH);
  setupMotor(motor2, driver2, sensor2, M2_CH);

  resetValues();
}

void loop(){
  unsigned long now = millis();

  bool btn = digitalRead(Button);
  if (btn != last_button && (now - last_change) > debounce_ms) {
    last_change = now;
    if (btn == LOW) {
      RUNSTART(920, 15, 10, -1); // distance cm, can distance cm , time s, -1 if left 1 if right
    }
    last_button = btn;
  }

  updateOdometry();
  printTelemetry();
}
