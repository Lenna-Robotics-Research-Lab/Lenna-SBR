// Includes
#include <Wire.h>

// Defines 
#define   ENR   26
#define   R1    27
#define   R2    14 

#define   ENL   25
#define   L1    32
#define   L2    33

#define   LED_BLINK   2

// Casting Functions
void blink_led(int times, double delay_ms);
void run_motor(int pwm_l, int pwm_r);

// Motor constants 
const int pwm_frequency = 20000;
const int pwm_channel_1 = 0;
const int pwm_channel_2 = 1;
const int resolution = 8;
const int max_pwm = 225;

// Sensor constants
const int MPU_ADDRESS = 0x68;
const int MPU_SMPLRT_DIV = 0x19;
const int MPU_CONFIG = 0x1A;
const int MPU_GYRO_CONFIG = 0x1B;
const int MPU_ACCEL_CONFIG = 0x1C;
const int MPU_ACCEL_XOUT_H = 0x3B;
const int MPU_TEMP_OUT_H = 0x41;
const int MPU_GYRO_XOUT_H = 0x43;

const int MPU_CALIBRATION_SAMPLES = 3000;
const double MPU_SCALE_FACTOR_GYRO = 65.5;
const double MPU_SCALE_FACTOR_ACCEL = 8192;

// Filter parameters
const double FILTER_TAU = 0.98;

// Variables
long loopTimer, loopTimer2;
int temperature;
double accelPitch;
double accelRoll;
long acc_x, acc_y, acc_z;
double accel_x, accel_y, accel_z;
double gyroRoll, gyroPitch, gyroYaw;
int gyro_x, gyro_y, gyro_z;
double gyro_x_cal = 603.63, gyro_y_cal = 61657.97, gyro_z_cal = 57512.39;
double rotation_x, rotation_y, rotation_z;
double roll = 0.0, pitch = 0.0;
double angle_limit_l = 0.0, angle_limit_h = 5.0;
double dt;
bool flag_measure = 0;

// PID Parameters
const double kp = 68, ki = 6.55, kd = 0.0005;
double pid_p, pid_i, pid_d;
double error, error_previous;
double setpoint = 0;
int PID, dir;

void setup() {
  // Start I2C communication
  Wire.begin();
  Serial.begin(115200);

  // Setup Digital I/O Pins
  pinMode(LED_BLINK, OUTPUT);

  // Setup Motor Pins
  pinMode(ENL, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT); 
  pinMode(ENR, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);

  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);

  // Setup Motor PWM Parameters
  ledcSetup(pwm_channel_1, pwm_frequency, resolution);
  ledcAttachPin(ENR, pwm_channel_1);
  ledcSetup(pwm_channel_2, pwm_frequency, resolution);
  ledcAttachPin(ENL, pwm_channel_2);

  blink_led(5, 100);

  // Setup the registers of the MPU-6050 and start up
  setup_mpu_6050_registers();

  // Gyro calibration
  Serial.println("Calibrating gyro, place on level surface and do not move.");

  // Take calibration readings and find average offset
  // gyro_calibration();

  Serial.println("GYRO. CALIBRATION");
  Serial.println(gyro_x_cal);
  Serial.println(gyro_y_cal);
  Serial.println(gyro_z_cal);
  
  Serial.println("\nLET'S GO!\n");

  blink_led(5, 100);
  // Display headers
  // Serial.print("\nNote 1: Yaw is not filtered and will drift!\n");
  // Serial.print("\nNote 2: Make sure sampling frequency is ~250 Hz\n");
  // Serial.print("LET'S GO!\n");
  // Serial.print("Roll (deg)\t\t");
  // Serial.print("Pitch (deg)\t\t");
  // Serial.print("Yaw (deg)\t\t\n");
  delay(2000);

  error = 0;
  error_previous = error;

  // Reset the loop timer
  loopTimer = micros();
  loopTimer2 = micros();
}

void loop() {
  dt = (micros() - loopTimer2) * 1e-6;
  loopTimer2 = micros();

  // Read raw accelerometer and gyroscope data
  read_mpu_6050_data();

  // Subtract the offset calibration value
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // Convert to instantaneous degrees per second
  rotation_x = (double)gyro_x / MPU_SCALE_FACTOR_GYRO;
  rotation_y = (double)gyro_y / MPU_SCALE_FACTOR_GYRO;
  rotation_z = (double)gyro_z / MPU_SCALE_FACTOR_GYRO;

  // Convert to g-force
  accel_x = (double)acc_x / MPU_SCALE_FACTOR_ACCEL;
  accel_y = (double)acc_y / MPU_SCALE_FACTOR_ACCEL;
  accel_z = (double)acc_z / MPU_SCALE_FACTOR_ACCEL;

  // Complementary filter
  accelPitch = atan2(accel_y, accel_z) * RAD_TO_DEG;
  accelRoll = atan2(accel_x, accel_z) * RAD_TO_DEG;

  // gyroPitch += rotation_x * dt;
  // gyroRoll -= rotation_y * dt;
  // gyroYaw += rotation_z * dt;

  pitch = (FILTER_TAU) * (pitch + (rotation_x * dt)) + (1 - FILTER_TAU) * (accelPitch);
  roll = (FILTER_TAU) * (roll - (rotation_y * dt)) + (1 - FILTER_TAU) * (accelRoll);

  gyroRoll = roll - 11.2;

  if( abs(gyroRoll) >= angle_limit_l && abs(gyroRoll) <= angle_limit_h )
  {
    digitalWrite(LED_BLINK, LOW);

    // PID Implementation
    error = gyroRoll - setpoint;
    error = ((int)(error * 100 + .5) / 100.0);

    pid_p =  error;
    pid_i += error * dt;
    pid_d =  (error - error_previous) / dt;

    if( abs(gyroRoll) <= 0.1)
    {
      pid_p = 0;
      pid_i = 0;
      pid_d = 0;
    }

    PID = kp*pid_p + ki*pid_i + kd*pid_d;

    if (abs(gyroRoll) < 1)
    {
      dir = PID / abs(PID);
      PID += dir * 80;
    }

    if( abs(PID) >= max_pwm )
    {
      PID /= abs(PID);
      PID *= max_pwm;
    }

    run_motor(PID, PID);
    error_previous = error;
  }
  else
  {
    digitalWrite(LED_BLINK, HIGH);
    run_motor(0, 0);
  }

  // Data out to serial monitor
  Serial.print(error, 2);
  Serial.print("\t\t");
  Serial.print(pid_d, 2);
  Serial.print("\t\t");
  Serial.println(PID);

  // Wait until the loopTimer reaches 10638 (194Hz) before next loop
  while (micros() - loopTimer <= 8000);
  loopTimer = micros();
}

/* LED Functions */
void blink_led(int times, double delay_ms)
{
  for(int i = 0; i < times; i++)
  {
    digitalWrite(LED_BLINK, 1);
    delay(delay_ms);
    digitalWrite(LED_BLINK, 0);
    delay(delay_ms);
  }
}

/* Motor Functions */
void run_motor(int pwm_l, int pwm_r)
{
  
  if(pwm_r >= 0)
  {
    ledcWrite(pwm_channel_1, pwm_r);
    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
  }
  else
  {
    ledcWrite(pwm_channel_1, -1*pwm_r);
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
  }

  if(pwm_l >= 0)
  {
    ledcWrite(pwm_channel_2, pwm_l);
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
  }
  else
  {
    ledcWrite(pwm_channel_2, -1*pwm_l);
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
  }
}

/* IMU Functions */
void read_mpu_6050_data() 
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 14, true);

  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}

void setup_mpu_6050_registers() 
{
  // Activate the MPU-6050
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure the sample rate divider
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU_SMPLRT_DIV);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure the accelerometer
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU_ACCEL_CONFIG);
  Wire.write(0x08); // 4g range
  Wire.endTransmission();

  // Configure the gyro
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU_GYRO_CONFIG);
  Wire.write(0x08); // 500 deg/s range
  Wire.endTransmission();
}

void gyro_calibration() 
{
  for (int cal_int = 0; cal_int < MPU_CALIBRATION_SAMPLES; cal_int++) {
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;

    delay(3);
  }

  gyro_x_cal /= MPU_CALIBRATION_SAMPLES;
  gyro_y_cal /= MPU_CALIBRATION_SAMPLES;
  gyro_z_cal /= MPU_CALIBRATION_SAMPLES;
}
