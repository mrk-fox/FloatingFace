//Copyright 2026 Mark Scharonow

//This source describes Open Hardware and is licensed under the CERN-OHL-S v2.

//You may redistribute and modify this source and make products using it
//under the terms of the CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2).

//This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
//INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A
//PARTICULAR PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.

//Source location: https://github.com/mrk-fox/FloatingFace

//goal:init all TMCs and start reading torque

//0->a 1->b 2->c
#include "driver/pulse_cnt.h"
#include <TMCStepper.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <AS5600.h>
#include <Wire.h>
#include <math.h>

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

#define LB_0 35
#define LB_1 30
#define LB_2 14


#define PULSE_GPIO_0  35
#define PULSE_GPIO_1  30
#define PULSE_GPIO_2  14

#define DRIVER_ADDR_0 0b00
#define DRIVER_ADDR_1 0b01
#define DRIVER_ADDR_2 0b10
#define TCA_ADDR 0x70

#define SERIAL_PORT Serial0

#define R_SENSE 0.11f

#define U 29.845

bool valid_tilt;

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
int b[3]  = {50, 100, 0};
int c[3]  = {100, 0, 0};
float n = 0; // set cm/unit value here

void tca_select(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

pcnt_unit_handle_t units[3];
int gpios[] = {PULSE_GPIO_0, PULSE_GPIO_1, PULSE_GPIO_2};

//------------------------TILT_ANGLES----------------------

MPU6050 mpu;

struct TiltResult {
  float tiltAngle;     // 0° = flat, 90° = vertical
  float tiltDirection; // -180° to +180° relative to sensor X-axis
  bool  valid;         // false if sensor read failed (e.g. free-fall)
};

TiltResult getTilt() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ax_f = ax / 16384.0f;
  float ay_f = ay / 16384.0f;
  float az_f = az / 16384.0f;

  float mag = sqrt(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
  if (mag < 0.001f) return {0, 0, false}; // free-fall / bad read

  ax_f /= mag;
  ay_f /= mag;
  az_f /= mag;

  return {
    acos(constrain(az_f, -1.0f, 1.0f)) * 180.0f / PI,
    atan2(ay_f, ax_f) * 180.0f / PI,
    true
  };
}



//--------------------------SETUP--------------------------

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\nStart...");

  SERIAL_PORT.begin(115200);
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (true);
  }

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

  pinMode(LB_0, INPUT);
  pinMode(LB_1, INPUT);
  pinMode(LB_2, INPUT);

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

  pcnt_unit_config_t unit_config = {
    .low_limit  = -32768,
    .high_limit =  32767,
  };

  for (int i = 0; i < 3; i++) {
      pcntInitRising(gpios[i], &units[i]);
  }
  
  tension();
  tdc();
}


//pcnt_unit_get_count(units[i], &counts[i]);  /get prct and store it in counts[i]
//pcnt_unit_clear_count(units[i]); //set prct unit i to 0

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

void read_tilt_angles() {
  TiltResult t = getTilt();
  if (t.valid) { valid_tilt = true;} else {
    Serial.println("Invalid reading (free-fall or sensor error)");
  }
}

float get_tilt_angle() { TiltResult t = getTilt(); return t.tiltAngle; }
float get_tilt_dir()   { TiltResult t = getTilt(); return t.tiltDirection; }

int get_max_v_pos(float* delta_g, float match) {
  if (delta_g[0] == match) {
    return 0;
  } else if (delta_g[1] == match) {
    return 1;
  } else {
    return 2;
  }
}


//--------------------CALIBRATION--------------------

bool tdc() {
  calibrate_distance_ab();
  calibrate_distance_bc();
  calibrate_distance_ca();
  return true;
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
  return true;
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

void calibrate_angle() {
  while (get_tilt_angle() > 10.0) {
    float dir = get_tilt_dir();
    float mot_dir_diff[3];
    for (int i = 0; i < 3; i++) {
      mot_dir_diff[i] = 180*i - dir;
    }
    float max_match = find_max_v_val(mot_dir_diff);
    if (get_max_v_pos(mot_dir_diff, max_match) == 0) {
      run_b(true, 100); //tension dir.
    }
    else if (get_max_v_pos(mot_dir_diff, max_match) == 1) {
      run_c(true, 100);
    }
    else {
      run_a(true, 100);
    }
  }
}


//--------------------------------MOVEMENT----------------------------------

void move_to(int x, int y, int z) {
  int coords[3] =  {x, y, z};
  float alpha = 0;
  float beta = 0;
  float gamma = 0;
  float angles[3] = {alpha, beta, gamma};
  float path[3] = {a, b, c};
  float k = 0;
  float l = 0;
  float m = 0;
  float coeff[3] = {k, l, m};
  bool dirs[3];
  bool confirm_0 = true;
  bool confirm_1 = true;
  bool confirm_2 = true;
  float to_run[3];
  int full_turns_left[3];
  bool full_clear = false;
  bool a_clear = false;
  bool b_clear = false;
  bool c_clear = false;
  bool finished = false;
  bool a_finished = false;
  bool b_finished = false;
  bool c_finished = false;

  int counts[3];
    for (int i = 0; i < 3; i++) {
        pcnt_unit_clear_count(units[i]);  // reset all to 0
    }



  if(validate_inrange(coords)) {
    calc_delta_g_n(p0, coords, path);                 
    for (int i = 0; i < 3; i++) {                     
      if(path[i]>0) {dirs[i]=true;}                     //Dir(true/false) still unknown
      else {dirs[i]=false;}
    }
    delta_g_n_to_angle(path, angles);
    for (int i = 0; i < 3; i++) {
      to_run[i] = angles[i];
    }
    coeff_calc(path, coeff);
    tracker0.start();
    tracker1.start();
    tracker2.start();

    while(confirm_0 && confirm_1 && confirm_2) {
      run_a(dirs[0], 400*coeff[0]);
      run_b(dirs[1], 400*coeff[1]);
      run_c(dirs[2], 400*coeff[2]);
      if(read_prct0()>0) {
        confirm_0 = false;
        }
      if(read_prct1()>0) {
        confirm_1 = false;
        }
      if(read_prct2()>0) {
        confirm_2 = false;
        }
    }

    tracker0.stop();
    tracker1.stop();
    tracker2.stop();
    null_all_prcts();

    to_run[0] = to_run[0] - tracker0.delta();
    to_run[1] = to_run[1] - tracker1.delta();
    to_run[2] = to_run[2] - tracker2.delta();

    full_turns(to_run, full_turns_left);

    while(!full_clear) {
      if (full_turns_left[0] - read_prct0() > 3) {
        run_a(dirs[0], 100*coeff[0]);
      }
      else {
        a_clear = true;
      }
      if (full_turns_left[1] - read_prct1() > 3) {
        run_b(dirs[0], 100*coeff[1]);
      }
      else {
        b_clear = true;
      }
      if (full_turns_left[2] - read_prct2() > 3) {
        run_c(dirs[0], 100*coeff[2]);
      }
      else {
        c_clear = true;
      }
      if (a_clear && b_clear && c_clear) {
        full_clear = true;
      }
    }


    to_run[0] = to_run[0] - read_prct0() * 360;
    to_run[1] = to_run[1] - read_prct1() * 360;
    to_run[2] = to_run[2] - read_prct2() * 360;

    tracker0.start();
    tracker1.start();
    tracker2.start();

    while(!finished) {
      if (to_run[0]-tracker0.delta() > 10) {
        run_a(dirs[0], 400*coeff[0]);
      }
      else {
        a_finished = true;
      }
      if (to_run[1]-tracker1.delta() > 10) {
        run_b(dirs[1], 100*coeff[1]);
      }
      else {
        b_finished = true;
      }
      if (to_run[2]-tracker2.delta() > 10) {
        run_c(dirs[2], 100*coeff[2]);
      }
      else {
        c_finished = true;
      }
      if (a_finished && b_finished && c_finished) {
        finished = true;
      }
    }
  	
    calibrate_angle();
  }

}


int read_prct0() {
  int count;
  pcnt_unit_get_count(units[0], &count);
  return count;
}

int read_prct1() {
  int count;
  pcnt_unit_get_count(units[1], &count);
  return count;
}

int read_prct2() {
  int count;
  pcnt_unit_get_count(units[2], &count);
  return count;
}

void null_prct0() {
  pcnt_unit_clear_count(units[0]);
}

void null_prct1() {
  pcnt_unit_clear_count(units[1]);
}

void null_prct2() {
  pcnt_unit_clear_count(units[2]);
}

void null_all_prcts() {
  for (int i = 0; i < 3; i++) {
    pcnt_unit_clear_count(units[i]);
  }
}

void run_a(bool dir, int delay) {
  if(dir) {digitalWrite(DIR_0, HIGH);}
  else {digitalWrite(DIR_0, LOW);}
  digitalWrite(STEP_0, HIGH);
  delay(delay);
  digitalWrite(STEP_0, LOW);
  delay(delay);
}

void run_b(bool dir, int delay) {
  if(dir) {digitalWrite(DIR_1, HIGH);}
  else {digitalWrite(DIR_1, LOW);}
  digitalWrite(STEP_1, HIGH);
  delay(delay);
  digitalWrite(STEP_1, LOW);
  delay(delay);
}

void run_c(bool dir, int delay) {
  if(dir) {digitalWrite(DIR_2, HIGH);}
  else {digitalWrite(DIR_2, LOW);}
  digitalWrite(STEP_2, HIGH);
  delay(delay);
  digitalWrite(STEP_2, LOW);
  delay(delay);
}

void pcntInitRising(int gpio_num, pcnt_unit_handle_t *out_unit) {
    pcnt_unit_config_t unit_cfg = {
        .low_limit  = -32768,
        .high_limit =  32767,
    };
    pcnt_new_unit(&unit_cfg, out_unit);

    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num  = gpio_num,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t chan;
    pcnt_new_channel(*out_unit, &chan_cfg, &chan);

    pcnt_channel_set_edge_action(chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // rising  → +1
        PCNT_CHANNEL_EDGE_ACTION_HOLD       // falling → ignore
    );

    pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(*out_unit, &filter_cfg);

    pcnt_unit_enable(*out_unit);
    pcnt_unit_clear_count(*out_unit);
    pcnt_unit_start(*out_unit);
}

//pcnt_unit_clear_count(units[0]);  // reset unit 0 back to 0

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

// Fixed: internal array was int[] but function works on floats
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

// TODO: implement — convert cable length deltas to motor step angles
void delta_g_n_to_angle(float* delta_g, float* out_angles) {
  float tmp[3], tmp2[3];
  multiply_vector(delta_g, n, tmp);
  divide_vector(tmp, U, tmp2);
  multiply_vector(tmp2, 360.0, out_angles);
}

void full_turns(float* angles, int* out_turns) {
  for (int i = 0; i < 3; i++)
    out_turns[i] = (int)floor(angles[i] / 360.0f);
}

void coeff_calc(float* delta_g, float* out_coeff) {
  float coeff1[3];
  float coeff2[3];
  for(int i = 0; i < 3; i++) {
    coeff1[i] = abs(delta_g[i]);
  }
  float maxVal = find_max_v_val(coeff1);
  for (int i = 0; i < 3; i++) {
    coeff2[i] = delta_g[i] / maxVal;     // use it
  }
  for(int i = 0; i < 3; i++) {
    out_coeff[i] = 1/abs(coeff2[i]);
  }
}

float angles_to_cm(float deg) {
  float rotations = deg / 360.0f;
  float cm = rotations * PI * 8.5f;
  return cm;
}

