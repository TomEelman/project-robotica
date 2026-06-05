#include "Robot.h"

// GPIO pin assignments
// GP0/GP1 are reserved for UART (in picouarthandler), so motor and encoder signals start from GP2 onwards.
static constexpr int MOTOR_LEFT_PWM       = 8;
static constexpr int MOTOR_LEFT_FORWARD   = 2;
static constexpr int MOTOR_LEFT_BACKWARD  = 3;

static constexpr int ENCODER_LEFT_PULSE   = 11;
static constexpr int ENCODER_LEFT_RES     = 10;

static constexpr int MOTOR_RIGHT_PWM      = 9;
static constexpr int MOTOR_RIGHT_FORWARD  = 6;
static constexpr int MOTOR_RIGHT_BACKWARD = 7;

static constexpr int ENCODER_RIGHT_PULSE  = 14;
static constexpr int ENCODER_RIGHT_RES    = 15;

static constexpr int IMU_SDA = 4;
static constexpr int IMU_SCL = 5;

// Physical robot dimensions.
static constexpr float WHEELBASE_M = 0.219f; // distance between wheel contact points meters

Robot::Robot()
    : motorLeft (MOTOR_LEFT_PWM,  MOTOR_LEFT_FORWARD,  MOTOR_LEFT_BACKWARD),
      motorRight(MOTOR_RIGHT_PWM, MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_BACKWARD),
      sensorHub(ENCODER_LEFT_RES,ENCODER_RIGHT_RES,
                IMU_SDA, IMU_SCL),
      drive(motorLeft, motorRight, sensorHub, WHEELBASE_M, 
            13.0f,  // minAngleVel
            35.0f,  // maxAngleVel
            55.0f,  // minPwmLeft
            55.0f) // minPwmRight
{
}

void Robot::UpdateSensors()
{
    sensorHub.UpdateSensors();
}

void Robot::Execute(const DriveCommand& command)
{
    drive.Execute(command);
}

float Robot::GetAngVelocity() const { return sensorHub.GetAngVelocity(); }
float Robot::GetCurrentYaw()  const { return sensorHub.GetCurrentYaw();  }