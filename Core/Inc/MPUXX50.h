#ifndef MPUXX50_H
#define MPUXX50_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* I2C address selection: AD0 low = 0x68, AD0 high = 0x69. */
#define AD0_LOW  0x68U
#define AD0_HIGH 0x69U

#define AFSR_2G  0U
#define AFSR_4G  1U
#define AFSR_8G  2U
#define AFSR_16G 3U

#define GFSR_250DPS  0U
#define GFSR_500DPS  1U
#define GFSR_1000DPS 2U
#define GFSR_2000DPS 3U

#define I2C_TIMOUT_MS 100U

#define WHO_AM_I       0x75U
#define WHO_AM_I_9250_ANS 0x71U
#define WHO_AM_I_6500_ANS 0x70U
#define WHO_AM_I_6050_ANS 0x68U
#define PWR_MGMT_1     0x6BU
#define ACCEL_CONFIG   0x1CU
#define GYRO_CONFIG    0x1BU
#define ACCEL_XOUT_H   0x3BU
#define RAD2DEG        57.29577951308232f

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} MPU_RawData_t;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} MPU_SensorData_t;

typedef struct {
    float x, y, z;
} MPU_GyroCal_t;

typedef struct {
    float r, p, y;
} MPU_Attitude_t;

extern MPU_RawData_t rawData;
extern MPU_SensorData_t sensorData;
extern MPU_GyroCal_t gyroCal;
extern MPU_Attitude_t attitude;
extern float aScaleFactor;
extern float gScaleFactor;

uint8_t MPU_begin(I2C_HandleTypeDef *I2Cx, uint8_t addr,
                  uint8_t aScale, uint8_t gScale,
                  float tau, float dt);
void MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale);
void MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale);
void MPU_readRawData(I2C_HandleTypeDef *I2Cx);
void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints);
void MPU_readProcessedData(I2C_HandleTypeDef *I2Cx);
void MPU_calcAttitude(I2C_HandleTypeDef *I2Cx);

#endif /* MPUXX50_H */
