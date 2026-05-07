#include "IMU.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// BNO055 register map (relevant subset).
static constexpr uint8_t BNO055_CHIP_ID_VALUE = 0xA0;
static constexpr uint8_t REG_CHIP_ID          = 0x00;
static constexpr uint8_t REG_ACC_LSB          = 0x08;
static constexpr uint8_t REG_GYR_LSB          = 0x14;
static constexpr uint8_t REG_EULER_H_LSB      = 0x1A;
static constexpr uint8_t REG_CALIB_STAT       = 0x35;
static constexpr uint8_t REG_OPR_MODE         = 0x3D;
static constexpr uint8_t REG_PWR_MODE         = 0x3E;
static constexpr uint8_t REG_SYS_TRIGGER      = 0x3F;

// BNO055 operation modes.
static constexpr uint8_t OPR_CONFIG_MODE = 0x00;
static constexpr uint8_t OPR_NDOF_MODE   = 0x0C; // 9-DOF sensor fusion

// Calibration: top 2 bits of CALIB_STAT reflect system calibration level (0–3).
static constexpr uint8_t CALIB_SYS_MASK = 0xC0;

static constexpr i2c_inst_t* I2C_BUS = i2c0;

// I2C clock: 100 kHz (standard mode). The BNO055 also supports 400 kHz fast
// mode, but 100 kHz is safer over longer or unshielded traces.
static constexpr uint I2C_CLOCK_HZ = 100'000;

static uint8_t BnoRead8(uint8_t addr, uint8_t reg)
{
    uint8_t val;
    i2c_write_blocking(I2C_BUS, addr, &reg, 1, true);
    i2c_read_blocking (I2C_BUS, addr, &val, 1, false);
    return val;
}

static void BnoReadLen(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len)
{
    i2c_write_blocking(I2C_BUS, addr, &reg, 1, true);
    i2c_read_blocking (I2C_BUS, addr, buf, len, false);
}

static void BnoWrite8(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_BUS, addr, buf, 2, false);
}

static int16_t ToInt16(uint8_t lsb, uint8_t msb)
{
    return static_cast<int16_t>((msb << 8) | lsb);
}

IMU::IMU(int sdaPin, int sclPin, int i2cAddress)
    : sdaPin(sdaPin),
      sclPin(sclPin),
      i2cAddress(i2cAddress),
      linearVelocity(0.0f),
      angularVelocity(0.0f),
      currentYaw(0.0f),
      calibrated(false)
{
    i2c_init(I2C_BUS, I2C_CLOCK_HZ);
    gpio_set_function(sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sclPin, GPIO_FUNC_I2C);
    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);

    uint8_t chipId = BnoRead8(i2cAddress, REG_CHIP_ID);
    if (chipId != BNO055_CHIP_ID_VALUE)
        return; // calibrated stays false — caller can check GetCalibrationStatus()

    BnoWrite8(i2cAddress, REG_OPR_MODE, OPR_CONFIG_MODE);
    sleep_ms(20);

    // Soft-reset: clears sensor state and reloads factory calibration.
    BnoWrite8(i2cAddress, REG_SYS_TRIGGER, 0x20);
    sleep_ms(700); // BNO055 datasheet: ≥650 ms after reset before it is ready

    BnoWrite8(i2cAddress, REG_PWR_MODE, 0x00); // Normal power
    sleep_ms(10);

    BnoWrite8(i2cAddress, REG_OPR_MODE, OPR_NDOF_MODE);
    sleep_ms(50); // Mode switch settling time per datasheet

    calibrated = true;
}

bool IMU::Update()
{
    uint8_t eul[6];
    BnoReadLen(i2cAddress, REG_EULER_H_LSB, eul, 6);
    // BNO055 Euler output is in 1/16 degree units (fixed-point).
    currentYaw = ToInt16(eul[0], eul[1]) / 16.0f;

    uint8_t gyr[6];
    BnoReadLen(i2cAddress, REG_GYR_LSB, gyr, 6);
    // Gyro output is in 1/16 degree/s units in NDOF mode.
    angularVelocity = ToInt16(gyr[0], gyr[1]) / 16.0f;

    uint8_t acc[6];
    BnoReadLen(i2cAddress, REG_ACC_LSB, acc, 6);
    // Accelerometer output is in 1/100 m/s² units.
    linearVelocity = ToInt16(acc[0], acc[1]) / 100.0f;

    uint8_t calib = BnoRead8(i2cAddress, REG_CALIB_STAT);
    calibrated = ((calib & CALIB_SYS_MASK) == CALIB_SYS_MASK);

    return true;
}

float IMU::GetLinVelocity()       const { return linearVelocity;  }
float IMU::GetAngVelocity()       const { return angularVelocity; }
float IMU::GetCurrentYaw()        const { return currentYaw;      }
bool  IMU::GetCalibrationStatus() const { return calibrated;      }

void  IMU::SetI2CAddress(int address) { i2cAddress = address; }
int   IMU::GetI2CAddress()      const { return i2cAddress;    }
int   IMU::GetSDAPin()          const { return sdaPin;        }
int   IMU::GetSCLPin()          const { return sclPin;        }