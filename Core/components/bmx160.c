/*!
 * @file DFRobot_BMX160.cpp
 * @brief define DFRobot_BMX160 class infrastructure, the implementation of basic methods
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [luoyufeng] (yufeng.luo@dfrobot.com)
 * @maintainer [Fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-10-20
 * @url https://github.com/DFRobot/DFRobot_BMX160
 */
#include "bmx160.h"

#if defined(__cpluplus)
extern "C"
{
#endif


float BMX160_ACCEL_RANGE_CONST = BMX160_ACCEL_MG_LSB_2G * 9.8;
float BMX160_GYRO_RANGE_CONST = BMX160_GYRO_SENSITIVITY_250DPS;
const uint8_t BMX160_ADDR = 0x68 << 1;

sBmx160Dev_t bmx160_Obmx160_s;

I2C_HandleTypeDef *_pWire;

void bmx160_set_i2c(I2C_HandleTypeDef *pWire)
{
    _pWire = pWire;
}

const uint8_t int_mask_lookup_table[13] = {
    BMX160_INT1_SLOPE_MASK,
    BMX160_INT1_SLOPE_MASK,
    BMX160_INT2_LOW_STEP_DETECT_MASK,
    BMX160_INT1_DOUBLE_TAP_MASK,
    BMX160_INT1_SINGLE_TAP_MASK,
    BMX160_INT1_ORIENT_MASK,
    BMX160_INT1_FLAT_MASK,
    BMX160_INT1_HIGH_G_MASK,
    BMX160_INT1_LOW_G_MASK,
    BMX160_INT1_NO_MOTION_MASK,
    BMX160_INT2_DATA_READY_MASK,
    BMX160_INT2_FIFO_FULL_MASK,
    BMX160_INT2_FIFO_WM_MASK
};

bool bmx160_begin()
{
    HAL_I2C_Init(_pWire);

    if (bmx160_scan() == true){
        bmx160_softReset();
        bmx160_writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11);
        osDelay(50);
        /* Set gyro to normal mode */
        bmx160_writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
        osDelay(100);
        /* Set mag to normal mode */
        bmx160_writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x19);
        osDelay(10);
        bmx160_setMagnConf();
        return true;
    }
    else
        return false;
}

void bmx160_setLowPower(){
    bmx160_softReset();
    osDelay(100);
    bmx160_setMagnConf();
    osDelay(100);
    bmx160_writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x12);
    osDelay(100);
    /* Set gyro to normal mode */
    bmx160_writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x17);
    osDelay(100);
    /* Set mag to normal mode */
    bmx160_writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x1B);
    osDelay(100);
}

void bmx160_wakeUp(){
    bmx160_softReset();
    osDelay(100);
    bmx160_setMagnConf();
    osDelay(100);
    bmx160_writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11);
    osDelay(100);
    /* Set gyro to normal mode */
    bmx160_writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
    osDelay(100);
    /* Set mag to normal mode */
    bmx160_writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x19);
    osDelay(100);
}

bool bmx160_softReset()
{
  int8_t rslt = BMX160_OK;
  rslt = bmx160__softReset(&bmx160_Obmx160_s);
  if (rslt == 0)
    return true;
  else
    return false;
}

int8_t bmx160__softReset(sBmx160Dev_t *dev)
{
  int8_t rslt = BMX160_OK;
  uint8_t data = BMX160_SOFT_RESET_CMD;
  if (dev == NULL){
    rslt = BMX160_E_NULL_PTR;
  }
  bmx160_writeBmxReg(BMX160_COMMAND_REG_ADDR, data);
  osDelay(BMX160_SOFT_RESET_DELAY_MS);
  if (rslt == BMX160_OK){
    bmx160_defaultParamSettg(dev);
  }  
  return rslt;
}

void bmx160_defaultParamSettg(sBmx160Dev_t *dev)
{
  // Initializing accel and gyro params with
  dev->gyroCfg.bw = BMX160_GYRO_BW_NORMAL_MODE;
  dev->gyroCfg.odr = BMX160_GYRO_ODR_100HZ;
  dev->gyroCfg.power = BMX160_GYRO_SUSPEND_MODE;
  dev->gyroCfg.range = BMX160_GYRO_RANGE_2000_DPS;
  dev->accelCfg.bw = BMX160_ACCEL_BW_NORMAL_AVG4;
  dev->accelCfg.odr = BMX160_ACCEL_ODR_100HZ;
  dev->accelCfg.power = BMX160_ACCEL_SUSPEND_MODE;
  dev->accelCfg.range = BMX160_ACCEL_RANGE_2G;
  

  dev->prevMagnCfg = dev->magnCfg;
  dev->prevGyroCfg = dev->gyroCfg;
  dev->prevAccelCfg = dev->accelCfg;
}

void bmx160_setMagnConf()
{
    bmx160_writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x80);
    osDelay(50);
    // Sleep mode
    bmx160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x01);
    bmx160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4B);
    // REPXY regular preset
    bmx160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x04);
    bmx160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x51);
    // REPZ regular preset
    bmx160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x0E);
    bmx160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x52);
    
    bmx160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x02);
    bmx160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4C);
    bmx160_writeBmxReg(BMX160_MAGN_IF_1_ADDR, 0x42);
    bmx160_writeBmxReg(BMX160_MAGN_CONFIG_ADDR, 0x08);
    bmx160_writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x03);
    osDelay(50);
}

void bmx160_setGyroRange(eGyroRange_t bits){
    switch (bits){
        case eGyroRange_125DPS:
            BMX160_GYRO_RANGE_CONST = BMX160_GYRO_SENSITIVITY_125DPS;
            break;
        case eGyroRange_250DPS:
            BMX160_GYRO_RANGE_CONST = BMX160_GYRO_SENSITIVITY_250DPS;
            break;
        case eGyroRange_500DPS:
            BMX160_GYRO_RANGE_CONST = BMX160_GYRO_SENSITIVITY_500DPS;
            break;
        case eGyroRange_1000DPS:
            BMX160_GYRO_RANGE_CONST = BMX160_GYRO_SENSITIVITY_1000DPS;
            break;
        case eGyroRange_2000DPS:
            BMX160_GYRO_RANGE_CONST = BMX160_GYRO_SENSITIVITY_2000DPS;
            break;
        default:
            BMX160_GYRO_RANGE_CONST = BMX160_GYRO_SENSITIVITY_250DPS;
            break;
    }
}

void bmx160_setAccelRange(eAccelRange_t bits){
    switch (bits){
        case eAccelRange_2G:
            BMX160_ACCEL_RANGE_CONST = BMX160_ACCEL_MG_LSB_2G * 10;
            break;
        case eAccelRange_4G:
            BMX160_ACCEL_RANGE_CONST = BMX160_ACCEL_MG_LSB_4G * 10;
            break;
        case eAccelRange_8G:
            BMX160_ACCEL_RANGE_CONST = BMX160_ACCEL_MG_LSB_8G * 10;
            break;
        case eAccelRange_16G:
            BMX160_ACCEL_RANGE_CONST = BMX160_ACCEL_MG_LSB_16G * 10;
            break;
        default:
            BMX160_ACCEL_RANGE_CONST = BMX160_ACCEL_MG_LSB_2G * 10;
            break;
    }
}

void bmx160_getAllData(sBmx160SensorData_t *magn, sBmx160SensorData_t *gyro, sBmx160SensorData_t *accel){

    uint8_t data[23];
    memset(data, 0, sizeof(data));
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    // put your main code here, to run repeatedly:
    bmx160_readReg(BMX160_MAG_DATA_ADDR, data, 23);
    if(magn){
        x = (int16_t) (((uint16_t)data[1] << 8) | data[0]);
        y = (int16_t) (((uint16_t)data[3] << 8) | data[2]);
        z = (int16_t) (((uint16_t)data[5] << 8) | data[4]);
        magn->x = x * BMX160_MAGN_UT_LSB;
        magn->y = y * BMX160_MAGN_UT_LSB;
        magn->z = z * BMX160_MAGN_UT_LSB;
    }
    if(gyro){
        x = (int16_t) (((uint16_t)data[9] << 8) | data[8]);
        y = (int16_t) (((uint16_t)data[11] << 8) | data[10]);
        z = (int16_t) (((uint16_t)data[13] << 8) | data[12]);
        gyro->x = x * BMX160_GYRO_RANGE_CONST;
        gyro->y = y * BMX160_GYRO_RANGE_CONST;
        gyro->z = z * BMX160_GYRO_RANGE_CONST;
    }
    if(accel){
        x = (int16_t) (((uint16_t)data[15] << 8) | data[14]);
        y = (int16_t) (((uint16_t)data[17] << 8) | data[16]);
        z = (int16_t) (((uint16_t)data[19] << 8) | data[18]);
        accel->x = x * BMX160_ACCEL_RANGE_CONST;
        accel->y = y * BMX160_ACCEL_RANGE_CONST;
        accel->z = z * BMX160_ACCEL_RANGE_CONST;
    }
}

void bmx160_writeBmxReg(uint8_t reg, uint8_t value)
{
    bmx160_writeReg(reg, &value, 1);
}

void bmx160_writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
	uint16_t total_len = len + 1;
	uint8_t temp[total_len];
	temp[0] = reg;
	memcpy(&temp[1], pBuf, len);
    HAL_I2C_Master_Transmit(_pWire, BMX160_ADDR, temp, total_len, HAL_MAX_DELAY);
}

void bmx160_readReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
	HAL_I2C_Master_Transmit(_pWire, BMX160_ADDR, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(_pWire, BMX160_ADDR, pBuf, len, HAL_MAX_DELAY);
}

bool bmx160_scan()
{
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(_pWire, BMX160_ADDR, 100, HAL_MAX_DELAY);

    if (ret == HAL_OK) {
    	return true;
    } else {
    	return false;
    }
}

#if defined(__cplusplus)
}
#endif // __cpluplus
