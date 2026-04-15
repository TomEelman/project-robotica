#include "IMU.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define BNO055_CHIP_ID      0xA0
#define REG_CHIP_ID         0x00
#define REG_OPR_MODE        0x3D
#define REG_PWR_MODE        0x3E
#define REG_SYS_TRIGGER     0x3F
#define REG_CALIB_STAT      0x35
#define REG_EULER_H_LSB     0x1A
#define REG_ACC_LSB         0x08
#define REG_GYR_LSB         0x14

#define I2C_PORT            i2c0




static uint8_t bno_read8(uint8_t addr, uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, &val, 1, false);
    return val;
}

static void bno_readLen(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buf, len, false);
}

static void bno_write8(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}

static int16_t to_int16(uint8_t lsb, uint8_t msb) {
    return (int16_t)((msb << 8) | lsb);
}

// ── Constructor ──────────────────────────────────────────────────
IMU::IMU(int SDAPin, int SCLPIN, int ComAdress) {
    sdaPin              = SDAPin;
    sclPin              = SCLPIN;
    communicationAdress = ComAdress;  // standaard BNO055 adres
    linearVelocity      = 0.0f;
    angularVelocity     = 0.0f;
    currentYaw          = 0.0f;
    calibrationStatus   = false;
    Updated             = false;

    // I2C initialiseren
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sclPin, GPIO_FUNC_I2C);
    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);

    // Controleer chip ID
    uint8_t chipId = bno_read8(communicationAdress, REG_CHIP_ID);
    if (chipId != BNO055_CHIP_ID) {
        calibrationStatus = false;
        return;
    }

    // Config mode
    bno_write8(communicationAdress, REG_OPR_MODE, 0x00);
    sleep_ms(20);

    // Reset
    bno_write8(communicationAdress, REG_SYS_TRIGGER, 0x20);
    sleep_ms(700);

    // Normal power mode
    bno_write8(communicationAdress, REG_PWR_MODE, 0x00);
    sleep_ms(10);

    // NDOF fusion mode
    bno_write8(communicationAdress, REG_OPR_MODE, 0x0C);
    sleep_ms(50);

    calibrationStatus = true;
}

// ── Update ───────────────────────────────────────────────────────
bool IMU::Update() {
    // Euler hoeken uitlezen (yaw)
    uint8_t eul[6];
    bno_readLen(communicationAdress, REG_EULER_H_LSB, eul, 6);
    currentYaw = to_int16(eul[0], eul[1]) / 16.0f;

    // Gyroscoop uitlezen voor hoeksnelheid
    uint8_t gyr[6];
    bno_readLen(communicationAdress, REG_GYR_LSB, gyr, 6);
    angularVelocity = to_int16(gyr[0], gyr[1]) / 16.0f;

    // Accelerometer uitlezen voor lineaire snelheid (X-as)
    uint8_t acc[6];
    bno_readLen(communicationAdress, REG_ACC_LSB, acc, 6);
    linearVelocity = to_int16(acc[0], acc[1]) / 100.0f;  // m/s²

    // Kalibratiestatus uitlezen
    uint8_t calib = bno_read8(communicationAdress, REG_CALIB_STAT);
    calibrationStatus = ((calib & 0xC0) == 0xC0);  // sys calib bits
    printf("yaw:%f\n", currentYaw);
    Updated = true;
    return true;
}

float IMU::GetLinVelocity()const {
    return linearVelocity;
}

float IMU::GetAngVelocity()const {
    return angularVelocity;
}

float IMU::GetCurrentYaw()const {
    return currentYaw;
}

bool IMU::GetCalibrationStatus()const {
    return calibrationStatus;
}

void IMU::SetComAdress(int Adres) {
    communicationAdress = Adres;
}

int IMU::GetComAdress()const {
    return communicationAdress;
}

int IMU::GetSDAPin()const {
    return sdaPin;
}

int IMU::GetSCLPin()const{
    return sclPin;
}