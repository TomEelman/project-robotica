#include "Robot.h"
#include <cstdio>

// ── Pin definities ───────────────────────────────────────────────
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

// LiDAR is connected via USB-serial, bridged to uart1 on the Pico.
// Default baud rate for most LiDAR sensors is 115200.
// TODO: make sure GP8/GP9 (or whichever pins you wired) are configured
//       as uart1 TX/RX in your CMakeLists / board init.
constexpr int LIDAR_BAUD = 115200;

constexpr float WHEELBASE = 0.219f;  // afstand tussen wielen in meters
constexpr int   THRESHOLD = 0.01;    // minimale snelheid voor rijden

// ── Constructor ──────────────────────────────────────────────────
Robot::Robot()
    : motorLeft (MOTOR_LEFT_PWM,  MOTOR_LEFT_FORWARD,  MOTOR_LEFT_BACKWARD),
      motorRight(MOTOR_RIGHT_PWM, MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_BACKWARD),
      sensorHub(ENCODER_LEFT_PULS, ENCODER_LEFT_PULSRES,
                ENCODER_RIGHT_PULS, ENCODER_RIGHT_PULSRES,
                IMU_SDA, IMU_SCL,
                uart1, LIDAR_BAUD),
      drive(motorLeft, motorRight, sensorHub, WHEELBASE, THRESHOLD)
{
}

// ── Update (aanroepen in je hoofdlus) ────────────────────────────
void Robot::Update() {
    sensorHub.UpdateSensors();
    sleep_ms(1000);
    drive.Execute(DriveCommand(0.0f, 0.1f));
    sleep_ms(1000);
    drive.Execute(DriveCommand(448.0f, 0.0f));
    sleep_ms(1000);
    drive.Execute(DriveCommand(0.0f, 0.0f));
    sleep_ms(1000);
    drive.Stop();
}