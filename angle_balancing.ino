#include <MPU6050Sensor.h>
#include <HM10BLE.h>
#include <stdlib.h>

#define THROTTLE_INI 10
#define THROTTLE_CHANGE 5

MPU6050Sensor Sensor;
HM10BLE Ble;

float throttle_prev;
float angle_sum;

void setup() {
  Sensor.begin(0x68);
  Ble.begin(115200);

  Serial.begin(115200);

  init_motor_speed(THROTTLE_INI);
}

void init_motor_speed(int speed)
{
  throttle_prev = speed;
  
  analogWrite(6, speed);
  analogWrite(10, speed);
  analogWrite(9, speed);
  analogWrite(5, speed);
}

void loop()
{
  static float roll_target_angle = 0, pitch_target_angle = 0, yaw_target_angle = 0;
  static float throttle = 0;
  
  float roll_angle, pitch_angle, yaw_angle;
  Sensor.read_rpy_angle(roll_angle, pitch_angle, yaw_angle);

  float roll_gyro, pitch_gyro, yaw_gyro;
  Sensor.read_rpy_angular_velocity(roll_gyro, pitch_gyro, yaw_gyro);

  int roll_user, pitch_user, yaw_user, throttle_user;
  Ble.receive_user_rpyt(roll_user, pitch_user, yaw_user, throttle_user);

  float roll_balancing, pitch_balancing, yaw_balancing;
  calc_balancing_with_angle_and_gyro(
    roll_target_angle, pitch_target_angle, yaw_target_angle,
    roll_angle, pitch_angle, yaw_angle,
    roll_gyro, pitch_gyro, yaw_gyro,
    roll_balancing, pitch_balancing, yaw_balancing);

  float motorA_speed, motorB_speed, motorC_speed, motorD_speed;
  calc_motor_speed(
    throttle, roll_balancing, pitch_balancing, yaw_balancing,
    motorA_speed, motorB_speed, motorC_speed, motorD_speed, roll_angle, pitch_angle);

  control_trpy(
    throttle, roll_target_angle, pitch_target_angle, yaw_target_angle,
    throttle_user, roll_user, pitch_user, yaw_user);

  update_motor_speed(
    motorA_speed, motorB_speed, motorC_speed, motorD_speed);
  
  //print_motor_speed(
  //  roll_angle, pitch_angle, yaw_angle,
  //  roll_balancing, pitch_balancing, yaw_balancing,
  //  motorA_speed, motorB_speed, motorC_speed, motorD_speed);
  
  Serial.print("ROLL USER : ");
  Serial.print(roll_user);
  Serial.print("\t");
  Serial.print("PITCH_USER : ");
  Serial.print(pitch_user);
  Serial.print("\t");
  Serial.print("YAW_USER : ");
  Serial.print(yaw_user);
  Serial.print("\t");
  Serial.print("D : ");
  Serial.print(motorD_speed);
  Serial.print("\t");
  Serial.println();
}

void calc_balancing_with_angle_and_gyro(
  float roll_target_angle,
  float pitch_target_angle,
  float yaw_target_angle,
  float roll_angle,
  float pitch_angle,
  float yaw_angle,
  float roll_gyro,
  float pitch_gyro,
  float yaw_gyro,
  float &roll_balancing,
  float &pitch_balancing,
  float &yaw_balancing)
{
  roll_balancing = roll_target_angle - roll_angle - roll_gyro;
  pitch_balancing = pitch_target_angle - pitch_angle - pitch_gyro;
  yaw_balancing = yaw_target_angle - yaw_angle - yaw_gyro;
}


void calc_motor_speed(float throttle,
    float roll_balancing,
    float pitch_balancing,
    float yaw_balancing,
    float &motorA_speed,
    float &motorB_speed,
    float &motorC_speed,
    float &motorD_speed,
    float roll_angle,
    float pitch_angle)
{
  /*
   motorA_speed = (throttle == 0) ? 0 :
    throttle + yaw_balancing + roll_balancing + pitch_balancing + 8;
  motorB_speed = (throttle == 0) ? 0 :
    throttle - yaw_balancing - roll_balancing + pitch_balancing + 37;
  motorC_speed = (throttle == 0) ? 0 :
    throttle + yaw_balancing - roll_balancing - pitch_balancing + 15;
  motorD_speed = (throttle == 0) ? 0 :
    throttle - yaw_balancing + roll_balancing - pitch_balancing - 2;
  */  
  float angle_sum = 0; //5 * (abs(roll_angle) + abs(pitch_angle));
  
  motorA_speed = (throttle == 0) ? 0 :
    throttle + yaw_balancing + roll_balancing + pitch_balancing + angle_sum - 2;
  motorB_speed = (throttle == 0) ? 0 :
    throttle - yaw_balancing - roll_balancing + pitch_balancing + angle_sum + 2;
  motorC_speed = (throttle == 0) ? 0 :
    throttle + yaw_balancing - roll_balancing - pitch_balancing + angle_sum + 8;
  motorD_speed = (throttle == 0) ? 0 :
    throttle - yaw_balancing + roll_balancing - pitch_balancing + angle_sum + 4;
    
  if(motorA_speed < 0) motorA_speed = 0;
  if(motorA_speed > 255) motorA_speed = 255;
  if(motorB_speed < 0) motorB_speed = 0;
  if(motorB_speed > 255) motorB_speed = 255;
  if(motorC_speed < 0) motorC_speed = 0;
  if(motorC_speed > 255) motorC_speed = 255;
  if(motorD_speed < 0) motorD_speed = 0;
  if(motorD_speed > 255) motorD_speed = 255;
}



void update_motor_speed(float motorA_speed,
  float motorB_speed,
  float motorC_speed,
  float motorD_speed)
{
  analogWrite(6, motorA_speed);
  analogWrite(10, motorB_speed);
  analogWrite(9, motorC_speed);
  analogWrite(5, motorD_speed);
}

void control_trpy(float &throttle,
  float &roll_target_angle,
  float &pitch_target_angle,
  float &yaw_target_angle,
  float throttle_user,
  float roll_user,
  float pitch_user,
  float yaw_user)
{ 
  throttle = 0.2 * throttle;
  float throttle_now;
  
  if (throttle_user <= throttle_prev - THROTTLE_CHANGE)
    throttle_now = throttle_prev - THROTTLE_CHANGE;
  else throttle_now = throttle_user;

  throttle = throttle_now;
  throttle_prev = throttle_now;
  
  if(roll_user <= 120) roll_target_angle = -(125 - roll_user);
  else if (roll_user >= 130) roll_target_angle = roll_user - 125;
  else roll_target_angle = 0;
  
  if(pitch_user <= 110) pitch_target_angle = 45;
  else if (pitch_user >= 126) pitch_target_angle = -45;
  else pitch_target_angle = 0;
  
  if(yaw_user <= 120) yaw_target_angle = 30;
  else if (yaw_user >= 130) yaw_target_angle = -30;
  else yaw_target_angle = 0;
}
