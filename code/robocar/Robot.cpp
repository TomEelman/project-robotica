#include "Robot.h"
#include <cstdio>


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
    kalmanLinks(50.0f, 36.0f),   // Q=50, R=36
    kalmanRechts(50.0f, 36.0f)
{
}

void Robot::Update() {
    sensorHub.UpdateSensors();
    drive.Execute(DriveCommand(378.0f, 0.0f));

    // Aanmaken met jouw ruiswaarden
    KalmanFilter kalmanLinks(50.0f, 36.0f);   // Q=50, R=36
    KalmanFilter kalmanRechts(50.0f, 36.0f);

// In Robot::Update()
float vLinks  = kalmanLinks.Update(sensorHub.GetSpeedLeft());
float vRechts = kalmanRechts.Update(sensorHub.GetSpeedRight());
    /*
    float yaw = sensorHub.GetCurrentYaw();
static float prevLeft = 0.0f;
static float prevRight = 0.0f;

float left = sensorHub.GetDistanceLeft();   // totaal!
float right = sensorHub.GetDistanceRight(); // totaal!
printf("Left%f Right%f\n",left, right);
float dLeft = left - prevLeft;
float dRight = right - prevRight;

prevLeft = left;
prevRight = right;

float wheelBase = 0.219f; // meters

float dYaw = (dRight - dLeft) / wheelBase;

static float encYaw = 0.0f;
encYaw += dYaw;
    printf("yaw:%f encyaw%f\n",yaw, encYaw);
    */
}