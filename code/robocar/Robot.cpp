#include "Robot.h"
#include <cstdio>
#define PI 3.14159265358979323846f

// GP0/1 bezet door UART
// Motor A: ENA=GP8, IN1=GP2, IN2=GP3
// Motor B: ENB=GP9, IN3=GP6, IN4=GP7
constexpr int MOTOR_LEFT_PWM      = 8;
constexpr int MOTOR_LEFT_FORWARD  = 2;
constexpr int MOTOR_LEFT_BACKWARD = 3;

constexpr int ENCODER_LEFT_PULS    = 11;
constexpr int ENCODER_LEFT_PULSRES = 10;

constexpr int MOTOR_RIGHT_PWM      = 9;
constexpr int MOTOR_RIGHT_FORWARD  = 6;
constexpr int MOTOR_RIGHT_BACKWARD = 7;

constexpr int ENCODER_RIGHT_PULS    = 14;
constexpr int ENCODER_RIGHT_PULSRES = 15;

constexpr int IMU_SDA  = 4;
constexpr int IMU_SCL  = 5;


constexpr int LIDAR_BAUD = 115200;

constexpr float WHEELBASE = 0.219f;  // afstand tussen wielen in meters
constexpr int   THRESHOLD = 0.01;    


Robot::Robot()
    : motorLeft (MOTOR_LEFT_PWM,  MOTOR_LEFT_FORWARD,  MOTOR_LEFT_BACKWARD),
      motorRight(MOTOR_RIGHT_PWM, MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_BACKWARD),
      sensorHub(ENCODER_LEFT_PULS, ENCODER_LEFT_PULSRES,
                ENCODER_RIGHT_PULS, ENCODER_RIGHT_PULSRES,
                IMU_SDA, IMU_SCL),
      drive(motorLeft, motorRight, sensorHub, WHEELBASE, THRESHOLD),
      localisation(WHEELBASE, THRESHOLD)  

{
}

void Robot::Update(float dt) {
    sensorHub.UpdateSensors(); 
    float Angvelocity = sensorHub.GetAngVelocity();
    float imuYawRate = Angvelocity* (PI / 180.0f);
    float vLeft = sensorHub.GetSpeedLeft();
    float vRight =sensorHub.GetSpeedRight();
    localisation.Update(vLeft, vRight, imuYawRate, dt);
    drive.Execute(DriveCommand(378.0f, 0.0f),dt);
    printf("vL %.2f vR %.2f omega %.3f theta %8f\n",
       vLeft, vRight, imuYawRate, localisation.GetTheta());
}