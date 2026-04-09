#include <Wire.h>
#include <SimpleFOC.h>

// ---- Motor pins ----
#define M1_U 12
#define M1_V 11
#define M1_W 10
#define M1_EN 9

#define M2_U 7
#define M2_V 6
#define M2_W 5
#define M2_EN 4

// ---- Encoder channels ----
#define M1_CH 2
#define M2_CH 7

// ---- Button ----
#define Button 8
const unsigned long debounce_ms = 50;
bool last_button = HIGH;
unsigned long last_change = 0;
bool ONOFF = 0;

float M1_pos = 0.0f;       //rad
float M2_pos = 0.0f;       //rad

float M1_pos_prev = 0.0f;
float M2_pos_prev = 0.0f;

long vel_time = micros();
long vel_time_prev = micros();

float M1_vel = 0.0f; //rad / s
float M2_vel = 0.0f; //rad / s

float dt = 0.0f;

float M1_i_vel = 0.0f;
float M2_i_vel = 0.0f;

// ---- TCA9548A ----
#define TCA_ADDR 0x70
static void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ---- Forward direction signs (adjust if needed) ----
const float M1_DIR = -1.0f;   // forward is + for motor1
const float M2_DIR = 1.0f;    // forward is + for motor2

MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);

BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_U, M1_V, M1_W, M1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_U, M2_V, M2_W, M2_EN);

void setupMotor(BLDCMotor &motor, BLDCDriver3PWM &driver, MagneticSensorI2C &sensor, uint8_t ch) {
  //encoder
  tcaSelect(ch);
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  //driver
  driver.voltage_power_supply = 12.0f;
  driver.voltage_limit = 12.0f;
  driver.pwm_frequency = 25000;
  driver.init();
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::torque; //voltage

  motor.current_limit = 1.5;
  motor.voltage_limit = 12.0f;

  motor.voltage_sensor_align = 3.0f;
  motor.sensor_direction = Direction::UNKNOWN;

  motor.init();
  motor.initFOC();
}

void updateVelocity(){
  vel_time = micros();
  tcaSelect(M1_CH);
  motor1.loopFOC();
  M1_pos = (motor1.shaftAngle()) * M1_DIR; //rad (sign corrected)

  //motor 2 angle
  tcaSelect(M2_CH);
  motor2.loopFOC();
  M2_pos = (motor2.shaftAngle()) * M2_DIR; //rad (sign corrected)

  dt = (float) (vel_time - vel_time_prev) / 1000000.0f;

  M1_vel = ((float) (M1_pos - M1_pos_prev) / dt) / (2.0f*PI); //rps
  M2_vel = ((float) (M2_pos - M2_pos_prev) / dt) / (2.0f*PI); //rps

  vel_time_prev = vel_time;
  M1_pos_prev = M1_pos;
  M2_pos_prev = M2_pos;
}
void updateVelocityControl(float M1_targetVelocity , float M2_targetVelocity){ //rps
  const float velocity_P = 0.0f; //V / (rps)
  const float velocity_I = 5.0f; //V / R
  updateVelocity();
  float M1_vel_err = M1_targetVelocity - M1_vel;
  float M2_vel_err = M2_targetVelocity - M2_vel;

  M1_i_vel = M1_i_vel + M1_vel_err * dt;
  M2_i_vel = M2_i_vel + M2_vel_err * dt;

  float M1_vel_cmd = velocity_P * M1_vel_err + velocity_I * M1_i_vel;
  float M2_vel_cmd = velocity_P * M2_vel_err + velocity_I * M2_i_vel;

  motor1.move(constrain(M1_vel_cmd,-10,10) * M1_DIR);
  motor2.move(constrain(M2_vel_cmd,-10,10) * M2_DIR);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Wire.begin(A4, A5);
  Wire.setClock(400000);

  pinMode(Button, INPUT_PULLUP);

  setupMotor(motor1, driver1, sensor1, M1_CH);
  setupMotor(motor2, driver2, sensor2, M2_CH);
}

unsigned long lastUpdateTime = 0;
unsigned long currentTime = millis();

const long interval = 1000; //ms

unsigned long lastFOC_us = 0;
const unsigned long foc_interval_us = 1000; // ~1kHz

void loop(){
  unsigned long now = millis();

  bool btn = digitalRead(Button);
  if (btn != last_button && (now - last_change) > debounce_ms) {
    last_change = now;
    if (btn == LOW) {
      (ONOFF == 0) ? ONOFF = 1 : ONOFF = 0;
    }
    last_button = btn;
  }
  unsigned long now_us = micros();
  if ((unsigned long)(now_us - lastFOC_us) >= foc_interval_us) {


    lastFOC_us = now_us;
    if (ONOFF == 1){
      motor1.enable();
      motor2.enable();
      updateVelocityControl(2 , 2);
    }else {
      motor1.disable();
      motor2.disable();;
    }
  }
  currentTime = millis();
  if (currentTime - lastUpdateTime >= interval) {
    Serial.print(M1_vel);
    Serial.print(" / ");
    Serial.println(M2_vel);
    lastUpdateTime = currentTime;
  }
}
