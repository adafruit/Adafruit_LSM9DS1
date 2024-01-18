/***************************************************************************
  This is a library for the LSM9DS1 Accelerometer and magnetometer/compass

  Designed specifically to work with the Adafruit LSM9DS1 Breakouts

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __LSM9DS1_H__
#define __LSM9DS1_H__

#include "Arduino.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>

#define LSM9DS1_ADDRESS_ACCELGYRO (0x6B)
#define LSM9DS1_ADDRESS_MAG (0x1E)
#define LSM9DS1_XG_ID (0b01101000)

// Linear Acceleration: mg per LSB
#define LSM9DS1_ACCEL_MG_LSB_2G (0.061F)
#define LSM9DS1_ACCEL_MG_LSB_4G (0.122F)
#define LSM9DS1_ACCEL_MG_LSB_8G (0.244F)
#define LSM9DS1_ACCEL_MG_LSB_16G (0.732F)

// Magnetic Field Strength: gauss range
#define LSM9DS1_MAG_MGAUSS_4GAUSS (0.14F)
#define LSM9DS1_MAG_MGAUSS_8GAUSS (0.29F)
#define LSM9DS1_MAG_MGAUSS_12GAUSS (0.43F)
#define LSM9DS1_MAG_MGAUSS_16GAUSS (0.58F)

// Angular Rate: dps per LSB
#define LSM9DS1_GYRO_DPS_DIGIT_245DPS (0.00875F)
#define LSM9DS1_GYRO_DPS_DIGIT_500DPS (0.01750F)
#define LSM9DS1_GYRO_DPS_DIGIT_2000DPS (0.07000F)

// Temperature: LSB per degree celsius
#define LSM9DS1_TEMP_LSB_DEGREE_CELSIUS (8) // 1°C = 8, 25° = 200, etc.

#define MAGTYPE (true)
#define XGTYPE (false)

/* Forward reference required for function pointers below. */
class Adafruit_LSM9DS1;

/* Pointer to member functions for read, get event, and get sensor.  These are
 * used */
/* by the Adafruit_LSM9DS1::Sensor class to read and retrieve individual
 * sensors. */
typedef void (Adafruit_LSM9DS1::*lsm9ds1_read_func)(void);
typedef void (Adafruit_LSM9DS1::*lsm9ds1_get_event_func)(sensors_event_t *,
                                                         uint32_t);
typedef void (Adafruit_LSM9DS1::*lsm9ds1_get_sensor_func)(sensor_t *);

/**! Interface object for LSM9DS1 9-DoF sensor */
class Adafruit_LSM9DS1 {
public:
  Adafruit_LSM9DS1(int32_t sensorID = 0);
  Adafruit_LSM9DS1(TwoWire *wireBus, int32_t sensorID = 0);
  Adafruit_LSM9DS1(int8_t xmcs, int8_t gcs, int32_t sensorID = 0);
  Adafruit_LSM9DS1(int8_t clk, int8_t miso, int8_t mosi, int8_t xmcs,
                   int8_t gcs, int32_t sensorID = 0);

  /**! Register mapping for accelerometer/gyroscope component */
  typedef enum {
    LSM9DS1_REGISTER_WHO_AM_I_XG = 0x0F,
    LSM9DS1_REGISTER_CTRL_REG1_G = 0x10,
    LSM9DS1_REGISTER_CTRL_REG2_G = 0x11,
    LSM9DS1_REGISTER_CTRL_REG3_G = 0x12,
    LSM9DS1_REGISTER_TEMP_OUT_L = 0x15,
    LSM9DS1_REGISTER_TEMP_OUT_H = 0x16,
    LSM9DS1_REGISTER_STATUS_REG = 0x17,
    LSM9DS1_REGISTER_OUT_X_L_G = 0x18,
    LSM9DS1_REGISTER_OUT_X_H_G = 0x19,
    LSM9DS1_REGISTER_OUT_Y_L_G = 0x1A,
    LSM9DS1_REGISTER_OUT_Y_H_G = 0x1B,
    LSM9DS1_REGISTER_OUT_Z_L_G = 0x1C,
    LSM9DS1_REGISTER_OUT_Z_H_G = 0x1D,
    LSM9DS1_REGISTER_CTRL_REG4 = 0x1E,
    LSM9DS1_REGISTER_CTRL_REG5_XL = 0x1F,
    LSM9DS1_REGISTER_CTRL_REG6_XL = 0x20,
    LSM9DS1_REGISTER_CTRL_REG7_XL = 0x21,
    LSM9DS1_REGISTER_CTRL_REG8 = 0x22,
    LSM9DS1_REGISTER_CTRL_REG9 = 0x23,
    LSM9DS1_REGISTER_CTRL_REG10 = 0x24,

    LSM9DS1_REGISTER_OUT_X_L_XL = 0x28,
    LSM9DS1_REGISTER_OUT_X_H_XL = 0x29,
    LSM9DS1_REGISTER_OUT_Y_L_XL = 0x2A,
    LSM9DS1_REGISTER_OUT_Y_H_XL = 0x2B,
    LSM9DS1_REGISTER_OUT_Z_L_XL = 0x2C,
    LSM9DS1_REGISTER_OUT_Z_H_XL = 0x2D,
  } lsm9ds1AccGyroRegisters_t;

  /**! Enumeration for accelerometer range (2/4/8/16 g) */
  typedef enum {
    LSM9DS1_ACCELRANGE_2G = (0b00 << 3),
    LSM9DS1_ACCELRANGE_16G = (0b01 << 3),
    LSM9DS1_ACCELRANGE_4G = (0b10 << 3),
    LSM9DS1_ACCELRANGE_8G = (0b11 << 3),
  } lsm9ds1AccelRange_t;

  /**! Enumeration for accelerometer data rate 10 - 952 Hz */
  typedef enum {
    LSM9DS1_ACCELDATARATE_POWERDOWN = (0b0000 << 5),
    LSM9DS1_ACCELDATARATE_10HZ = (0b001 << 5),
    LSM9DS1_ACCELDATARATE_50HZ = (0b010 << 5),
    LSM9DS1_ACCELDATARATE_119HZ = (0b011 << 5),
    LSM9DS1_ACCELDATARATE_238HZ = (0b100 << 5),
    LSM9DS1_ACCELDATARATE_476HZ = (0b101 << 5),
    LSM9DS1_ACCELDATARATE_952HZ = (0b110 << 5),
  } lsm9ds1AccelDataRate_t;

  /**! Enumeration for magnetometer scaling (4/8/12/16 gauss) */
  typedef enum {
    LSM9DS1_MAGGAIN_4GAUSS = (0b00 << 5),  // +/- 4 gauss
    LSM9DS1_MAGGAIN_8GAUSS = (0b01 << 5),  // +/- 8 gauss
    LSM9DS1_MAGGAIN_12GAUSS = (0b10 << 5), // +/- 12 gauss
    LSM9DS1_MAGGAIN_16GAUSS = (0b11 << 5)  // +/- 16 gauss
  } lsm9ds1MagGain_t;

  /**! Enumeration for gyroscope scaling (245/500/2000 dps) */
  typedef enum {
    LSM9DS1_GYROSCALE_245DPS =
        (0b00 << 3), // +/- 245 degrees per second rotation
    LSM9DS1_GYROSCALE_500DPS =
        (0b01 << 3), // +/- 500 degrees per second rotation
    LSM9DS1_GYROSCALE_2000DPS =
        (0b11 << 3) // +/- 2000 degrees per second rotation
  } lsm9ds1GyroScale_t;

  /**! 3D floating point vector with X Y Z components */
  typedef struct vector_s {
    float x; ///< X component
    float y; ///< Y component
    float z; ///< Z component
  } lsm9ds1Vector_t;

  lsm9ds1Vector_t
      accelData; ///< Last read accelerometer data will be available here
  lsm9ds1Vector_t gyroData; ///< Last read gyroscope data will be available here
  lsm9ds1Vector_t
      magData;         ///< Last read magnetometer data will be available here
  int16_t temperature; ///< Last read temperzture data will be available here

  bool begin(void);
  void read(void);
  void readAccel(void);
  void readGyro(void);
  void readMag(void);
  void readTemp(void);

  void setupAccel(lsm9ds1AccelRange_t range,
                  lsm9ds1AccelDataRate_t rate = LSM9DS1_ACCELDATARATE_10HZ);
  void setupMag(lsm9ds1MagGain_t gain);
  void setupGyro(lsm9ds1GyroScale_t scale);

  /* Adafruit Unified Sensor Functions (not standard yet ... the current base
   * class only */
  /* supports one sensor type, and we need to update the unified base class to
   * support   */
  /* multiple sensors in a single driver, returning an array */
  bool getEvent(sensors_event_t *accel, sensors_event_t *mag,
                sensors_event_t *gyro, sensors_event_t *temp);
  void getSensor(sensor_t *accel, sensor_t *mag, sensor_t *gyro,
                 sensor_t *temp);

  /**! Subclass to expose each sensor on the LSM9DS1 as an Adafruit_Sensor
   * instance. */
  class Sensor : public Adafruit_Sensor {
  public:
    /*! @brief Basic instatiator */
    Sensor() {}

    /*! @brief Instantiate based on existing sensor
        @param copy The Sensor object to 'copy' from */
    Sensor(const Sensor &copy)
        : _parent(copy._parent), _readFunc(copy._readFunc),
          _eventFunc(copy._eventFunc), _sensorFunc(copy._sensorFunc) {}

    /*! @brief Instantiate based on existing sensor
        @param parent The LSM9DS1 parent object
        @param readFunc The no-argument function to call to read data
        @param eventFunc The sensor_event_t function to call to get event data
        @param sensorFunc The sensor_t function to call to get sensor metadata
     */
    Sensor(Adafruit_LSM9DS1 *parent, lsm9ds1_read_func readFunc,
           lsm9ds1_get_event_func eventFunc, lsm9ds1_get_sensor_func sensorFunc)
        : _parent(parent), _readFunc(readFunc), _eventFunc(eventFunc),
          _sensorFunc(sensorFunc) {}

    /*! @brief Get sensor event data
        @param event Pointer to sensor_event_t to fill in
        @returns True on successful read */
    virtual bool getEvent(sensors_event_t *event) {
      (_parent->*_readFunc)();
      (_parent->*_eventFunc)(event, millis());
      return true;
    }

    /*! @brief Get sensor metadata - type and range information
        @param sensor Pointer to sensor_t to fill in */
    virtual void getSensor(sensor_t *sensor) {
      (_parent->*_sensorFunc)(sensor);
    }

  private:
    Adafruit_LSM9DS1 *_parent;
    lsm9ds1_read_func _readFunc;
    lsm9ds1_get_event_func _eventFunc;
    lsm9ds1_get_sensor_func _sensorFunc;
  };

  /*! @brief Return Adafruit_Sensor compatible interface for accelerometer
      @returns Reference to Adafruit_Sensor subclassed object  */
  Sensor &getAccel(void) { return _accelSensor; }
  /*! @brief Return Adafruit_Sensor compatible interface for gyroscope
      @returns Reference to Adafruit_Sensor subclassed object  */
  Sensor &getGyro(void) { return _gyroSensor; }
  /*! @brief Return Adafruit_Sensor compatible interface for temperature
      @returns Reference to Adafruit_Sensor subclassed object  */
  Sensor &getTemp(void) { return _tempSensor; }
  /*! @brief Return Adafruit_Sensor compatible interface for magnetometer
      @returns Reference to Adafruit_Sensor subclassed object  */
  Adafruit_Sensor &getMag(void) { return _magSensor; }

private:
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface
  void write8(boolean type, byte reg, byte value);
  byte read8(boolean type, byte reg);
  byte readBuffer(boolean type, byte reg, byte len, uint8_t *buffer);
  void initSensor(int32_t sensorID);

  TwoWire *_wire = NULL;
  int8_t _csm, _csxg, _mosi, _miso, _clk;
  float _accel_mg_lsb;
  float _gyro_dps_digit;
  int32_t _lsm9dso_sensorid_accel;
  int32_t _lsm9dso_sensorid_gyro;
  int32_t _lsm9dso_sensorid_temp;
  Sensor _accelSensor;
  Adafruit_LIS3MDL _magSensor;
  Sensor _gyroSensor;
  Sensor _tempSensor;

  /* Functions to get individual sensor measurements and metadata. */
  /* Note that these functions will NOT update the sensor state before getting
   */
  /* a new reading.  You MUST call read() manually to update the sensor state */
  /* before calling these functions! */
  void getAccelEvent(sensors_event_t *event, uint32_t timestamp);
  void getMagEvent(sensors_event_t *event, uint32_t timestamp);
  void getGyroEvent(sensors_event_t *event, uint32_t timestamp);
  void getTempEvent(sensors_event_t *event, uint32_t timestamp);
  void getAccelSensor(sensor_t *sensor);
  void getMagSensor(sensor_t *sensor);
  void getGyroSensor(sensor_t *sensor);
  void getTempSensor(sensor_t *sensor);
};

#endif
