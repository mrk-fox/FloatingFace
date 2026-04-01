//goal:init all TMCs and start reading torque

//0->a 1->b 2->c
#include <TMCStepper.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <AS5600.h>
#include <Wire.h>

#define I2C_SDA 12
#define I2C_SCL 15
#define EN_0 16
#define STEP_0 17
#define DIR_0 18
#define EN_1 19
#define STEP_1 20
#define DIR_1 27
#define EN_2 32
#define STEP_2 33
#define DIR_2 34

#define UART_TMC_RX 42
#define UART_TMC_TX 43

#define DRIVER_ADDR_0 0b00
#define DRIVER_ADDR_1 0b01
#define DRIVER_ADDR_2 0b10
#define TCA_ADDR 0x70

#define SERIAL_PORT Serial0

#define R_SENSE 0.11f

//-------------------Motors-----------------------------

using namespace TMC2208_n;

TMC2209Stepper driver0(&SERIAL_PORT, R_SENSE, DRIVER_ADDR_0);
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, DRIVER_ADDR_1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER_ADDR_2);

TMC2209Stepper* drivers[] = { &driver0, &driver1, &driver2 };
const uint8_t NUM_DRV = 3;

//---------------------Angles----------------------------

// Forward declaration so AngleTracker can call tca_select before it's defined
void tca_select(uint8_t channel);

class AngleTracker {
  AS5600* _sensor;
  uint8_t _channel;
  float _startAngle;
  bool _active = false;

  float readDeg() {
    tca_select(_channel);
    return _sensor->readAngle() * AS5600_RAW_TO_DEGREES;
  }

public:
  AngleTracker(AS5600* sensor, uint8_t channel)
    : _sensor(sensor), _channel(channel) {}

  void begin() {
    tca_select(_channel);
    _sensor->begin();
    if (!_sensor->isConnected()) {
      Serial.print("ERR: Sensor on channel ");
      Serial.print(_channel);
      Serial.println(" not found");
    }
  }

  void start() {
    _startAngle = readDeg();
    _active = true;
  }

  void stop() {
    _active = false;
  }

  float delta() {
    if (!_active) return 0;
    float d = readDeg() - _startAngle;
    if (d > 180)  d -= 360;
    if (d < -180) d += 360;
    return d;
  }

  bool isActive() { return _active; }
};

//tracker0.begin() -> init sensor
//tracker0.start() -> set zero, begin measurement
//tracker0.stop()  -> end measurement
//tracker0.delta() -> angle change since start() in degrees
//tracker0.isActive() -> is the measurement active?

AS5600 sensor0;
AS5600 sensor1;
AS5600 sensor2;

AngleTracker tracker0(&sensor0, 0);
AngleTracker tracker1(&sensor1, 1);
AngleTracker tracker2(&sensor2, 2);

// Fixed: int[] is invalid in C++; use int name[size]
int p0[3] = {0, 0, 0};
int p1[3] = {0, 0, 0};
int a[3]  = {0, 0, 0}; // set anchor values here
int b[3]  = {0, 0, 0};
int c[3]  = {0, 0, 0};
float n = 0; // set cm/unit value here

void tca_select(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

//--------------------------SETUP--------------------------

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\nStart...");

  SERIAL_PORT.begin(115200);

  for (int i = 0; i < NUM_DRV; i++) {
    drivers[i]->beginSerial(115200);
    drivers[i]->begin();
    drivers[i]->toff(4);
    drivers[i]->TCOOLTHRS(0xFFFFFF); // Fixed: missing semicolon
    drivers[i]->SGTHRS(40);
    drivers[i]->en_spreadCycle(false);
    drivers[i]->pdn_disable(true);
  }

  pinMode(EN_0,   OUTPUT);
  pinMode(EN_1,   OUTPUT);
  pinMode(EN_2,   OUTPUT);
  pinMode(DIR_0,  OUTPUT);
  pinMode(DIR_1,  OUTPUT);
  pinMode(DIR_2,  OUTPUT);
  pinMode(STEP_0, OUTPUT);
  pinMode(STEP_1, OUTPUT);
  pinMode(STEP_2, OUTPUT);

  digitalWrite(EN_0,  LOW);
  digitalWrite(EN_1,  LOW);
  digitalWrite(EN_2,  LOW);
  digitalWrite(DIR_0, HIGH); // direction TBD in testing
  digitalWrite(DIR_1, HIGH);
  digitalWrite(DIR_2, HIGH);

  // Fixed: replaced manual sensor init + undeclared offset0/1/2
  //        with the AngleTracker system that already handles this
  tracker0.begin();
  tracker1.begin();
  tracker2.begin();
}

//-------------------------LOOP----------------------------------

void loop() {
  for (int i = 0; i < NUM_DRV; i++) { // Fixed: was "int = 0"
    uint16_t sg = drivers[i]->sg_result();
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" SG: "); // Fixed: missing semicolon
    Serial.println(sg);
  }
  delay(100); // Fixed: missing semicolon
}

//-----------------METHODS-------------------------

// Forward declaration so tension/blind_x can call get_torque_n before it's defined
uint16_t get_torque_n(int num);

bool tension() {
  for (int i = 500; i > 150; i -= 50) { // Fixed: was "int = 500"
    while (get_torque_n(0) >= i || get_torque_n(1) >= i || get_torque_n(2) >= i) {
      digitalWrite(STEP_0, HIGH);
      digitalWrite(STEP_1, HIGH);
      digitalWrite(STEP_2, HIGH);
      delay(200);
      digitalWrite(STEP_0, LOW);
      digitalWrite(STEP_1, LOW);
      digitalWrite(STEP_2, LOW);
      delay(200);
    }
  }
  return true;
}

//--------------------CALIBRATION--------------------

bool tdc() {
  // TODO: implement
  return false;
}

bool blind_a() {
  digitalWrite(DIR_0, HIGH);
  digitalWrite(DIR_1, LOW);
  digitalWrite(DIR_2, LOW);
  while (get_torque_n(0) != 0) { // TODO: replace with threshold, != 0 is fragile on noisy SG
    if (get_torque_n(1) < 100) {
      digitalWrite(STEP_1, HIGH);
      digitalWrite(STEP_1, LOW);
    }
    if (get_torque_n(2) < 100) {
      digitalWrite(STEP_2, HIGH);
      digitalWrite(STEP_2, LOW);
    }
    digitalWrite(STEP_0, HIGH);
    delay(200);
    digitalWrite(STEP_0, LOW);
    delay(200);
  }
  return true; // Fixed: missing semicolons on all return true
}

bool blind_b() {
  digitalWrite(DIR_0, LOW);
  digitalWrite(DIR_1, HIGH);
  digitalWrite(DIR_2, LOW);
  while (get_torque_n(1) != 0) {
    if (get_torque_n(0) < 100) {
      digitalWrite(STEP_0, HIGH);
      digitalWrite(STEP_0, LOW);
    }
    if (get_torque_n(2) < 100) {
      digitalWrite(STEP_2, HIGH);
      digitalWrite(STEP_2, LOW);
    }
    digitalWrite(STEP_1, HIGH);
    delay(200);
    digitalWrite(STEP_1, LOW);
    delay(200);
  }
  return true;
}

bool blind_c() {
  digitalWrite(DIR_0, LOW);
  digitalWrite(DIR_1, LOW);
  digitalWrite(DIR_2, HIGH);
  while (get_torque_n(2) != 0) {
    if (get_torque_n(1) < 100) {
      digitalWrite(STEP_1, HIGH);
      digitalWrite(STEP_1, LOW);
    }
    if (get_torque_n(0) < 100) {
      digitalWrite(STEP_0, HIGH);
      digitalWrite(STEP_0, LOW);
    }
    digitalWrite(STEP_2, HIGH);
    delay(200);
    digitalWrite(STEP_2, LOW);
    delay(200);
  }
  return true;
}

float angles_to_cm(float angle) {
  // TODO: implement properly using spool circumference
  return angle * n;
}

// Fixed: tracker0.end() -> tracker0.stop() to match AngleTracker API
int calibrate_distance_ab() {
  blind_a();
  tracker0.start();
  blind_b();
  tracker0.stop();
  return angles_to_cm(tracker0.delta());
}

int calibrate_distance_bc() {
  blind_b();
  tracker1.start();
  blind_c();
  tracker1.stop();
  return angles_to_cm(tracker1.delta());
}

int calibrate_distance_ca() {
  blind_c();
  tracker2.start();
  blind_a();
  tracker2.stop();
  return angles_to_cm(tracker2.delta());
}

uint16_t get_torque_n(int num) {
  return drivers[num]->sg_result();
}

//--------------------------------MATH--------------------------------------


bool validate_inrange(int* pp1) {
  // TODO: make universal based on calibration data, currently hardcoded
  if (pp1[1] < (-2 * pp1[2] + 200) && pp1[1] > (0.5 * pp1[2])) {
    return true;
  }
  return false;
}

void subtract_vector(int* v1, int* v2, int* o) { // o = v2 - v1
  for (int i = 0; i < 3; i++) {
    o[i] = v2[i] - v1[i];
  }
}


float vector_length(int* vi) {
  return sqrt((float)(vi[0]*vi[0] + vi[1]*vi[1] + vi[2]*vi[2]));
}

void multiply_vector(float* v, float m, float* o) {
  for (int i = 0; i < 3; i++) {
    o[i] = v[i] * m;
  }
}

void divide_vector(float* v, float d, float* o) {
  for (int i = 0; i < 3; i++) {
    o[i] = v[i] / d;
  }
}

// Fixed: was empty
float find_max_v_val(float* v) {
  float maxVal = v[0];
  for (int i = 1; i < 3; i++) {
    if (v[i] > maxVal) maxVal = v[i];
  }
  return maxVal;
}


void calc_delta_g_n(int* pp0, int* pp1, float* o) {
  int tmp[3];

  subtract_vector(pp0, a, tmp);  float l_p0ma = vector_length(tmp);
  subtract_vector(pp0, b, tmp);  float l_p0mb = vector_length(tmp);
  subtract_vector(pp0, c, tmp);  float l_p0mc = vector_length(tmp);

  subtract_vector(pp1, a, tmp);  float l_p1ma = vector_length(tmp);
  subtract_vector(pp1, b, tmp);  float l_p1mb = vector_length(tmp);
  subtract_vector(pp1, c, tmp);  float l_p1mc = vector_length(tmp);

  o[0] = l_p1ma - l_p0ma;
  o[1] = l_p1mb - l_p0mb;
  o[2] = l_p1mc - l_p0mc;
}

float delta_cm[3];

void delta_g_n_to_cm(flaot* delta_g, float* n) {
  delta_cm[0] = delta_g[0] * n;
  delta_cm[1] = delta_g[1] * n;
  delta_cm[2] = delta_g[2] * n;
}

void delta_g_n_to_angles(float* delta_g, float* out_angles) {
  
}
