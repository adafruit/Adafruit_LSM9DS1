/***************************************************************************
  This is a library for the LSM9DS1 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM9DS1 Breakouts

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <Adafruit_LSM9DS1.h>

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief Instantiate with hardware I2C interface
    @param wireBus The I2C wire interface you'd like to use
    @param sensorID Unique identifier you'd like for the sensors
*/
/**************************************************************************/
Adafruit_LSM9DS1::Adafruit_LSM9DS1(TwoWire *wireBus, int32_t sensorID) {
  initI2C(wireBus, sensorID);
}

/**************************************************************************/
/*!
    @brief Instantiate with default hardware I2C interface
    @param sensorID Unique identifier you'd like for the sensors
*/
/**************************************************************************/
Adafruit_LSM9DS1::Adafruit_LSM9DS1(int32_t sensorID) {
  initI2C(&Wire, sensorID);
}

/**************************************************************************/
/*!
    @brief Instantiate with hardware SPI interface
    @param xgcs SPI CS pin for accelerometer/gyro subchip
    @param mcs SPI CS pin for magnetometer subchip
    @param sensorID Unique identifier you'd like for the sensors
*/
/**************************************************************************/
Adafruit_LSM9DS1::Adafruit_LSM9DS1(int8_t xgcs, int8_t mcs, int32_t sensorID) {
  _i2c = false;
  // hardware SPI!
  _csm = mcs;
  _csxg = xgcs;
  _mosi = _miso = _clk = -1;
  _lsm9dso_sensorid_accel = sensorID + 1;
  //_lsm9dso_sensorid_mag = sensorID + 2;
  _lsm9dso_sensorid_gyro = sensorID + 3;
  _lsm9dso_sensorid_temp = sensorID + 4;
  _accelSensor = Sensor(this, &Adafruit_LSM9DS1::readAccel,
                        &Adafruit_LSM9DS1::getAccelEvent,
                        &Adafruit_LSM9DS1::getAccelSensor);
  _gyroSensor =
      Sensor(this, &Adafruit_LSM9DS1::readGyro, &Adafruit_LSM9DS1::getGyroEvent,
             &Adafruit_LSM9DS1::getGyroSensor);
  _tempSensor =
      Sensor(this, &Adafruit_LSM9DS1::readTemp, &Adafruit_LSM9DS1::getTempEvent,
             &Adafruit_LSM9DS1::getTempSensor);
}

void Adafruit_LSM9DS1::initI2C(TwoWire *wireBus, int32_t sensorID) {
  _i2c = true;
  _wire = wireBus;
  _lsm9dso_sensorid_accel = sensorID + 1;
  //_lsm9dso_sensorid_mag = sensorID + 2;
  _lsm9dso_sensorid_gyro = sensorID + 3;
  _lsm9dso_sensorid_temp = sensorID + 4;
  _accelSensor = Sensor(this, &Adafruit_LSM9DS1::readAccel,
                        &Adafruit_LSM9DS1::getAccelEvent,
                        &Adafruit_LSM9DS1::getAccelSensor);
  _gyroSensor =
      Sensor(this, &Adafruit_LSM9DS1::readGyro, &Adafruit_LSM9DS1::getGyroEvent,
             &Adafruit_LSM9DS1::getGyroSensor);
  _tempSensor =
      Sensor(this, &Adafruit_LSM9DS1::readTemp, &Adafruit_LSM9DS1::getTempEvent,
             &Adafruit_LSM9DS1::getTempSensor);
}

/**************************************************************************/
/*!
    @brief Instantiate with software SPI interface
    @param sclk SPI clock pin
    @param smiso SPI MISO pin
    @param smosi SPI MOSI pin
    @param xgcs SPI CS pin for accelerometer/gyro subchip
    @param mcs SPI CS pin for magnetometer subchip
    @param sensorID Unique identifier you'd like for the sensors
*/
/**************************************************************************/
Adafruit_LSM9DS1::Adafruit_LSM9DS1(int8_t sclk, int8_t smiso, int8_t smosi,
                                   int8_t xgcs, int8_t mcs, int32_t sensorID) {
  _i2c = false;
  // software SPI!
  _csm = mcs;
  _csxg = xgcs;
  _mosi = smosi;
  _miso = smiso;
  _clk = sclk;
  _lsm9dso_sensorid_accel = sensorID + 1;
  //_lsm9dso_sensorid_mag = sensorID + 2;
  _lsm9dso_sensorid_gyro = sensorID + 3;
  _lsm9dso_sensorid_temp = sensorID + 4;
  _accelSensor = Sensor(this, &Adafruit_LSM9DS1::readAccel,
                        &Adafruit_LSM9DS1::getAccelEvent,
                        &Adafruit_LSM9DS1::getAccelSensor);
  _gyroSensor =
      Sensor(this, &Adafruit_LSM9DS1::readGyro, &Adafruit_LSM9DS1::getGyroEvent,
             &Adafruit_LSM9DS1::getGyroSensor);
  _tempSensor =
      Sensor(this, &Adafruit_LSM9DS1::readTemp, &Adafruit_LSM9DS1::getTempEvent,
             &Adafruit_LSM9DS1::getTempSensor);
}

/**************************************************************************/
/*!
    @brief Initialize I2C or SPI and detect/initialize subsensors
    @returns True if both subsensors were detected on the desired interface
*/
/**************************************************************************/
bool Adafruit_LSM9DS1::begin() {
  if (_i2c) {
    _wire->begin();
    if (!_magSensor.begin_I2C(LSM9DS1_ADDRESS_MAG)) {
      return false;
    }
  } else if (_clk == -1) {
    // Hardware SPI
    pinMode(_csxg, OUTPUT);
    pinMode(_csm, OUTPUT);
    digitalWrite(_csxg, HIGH);
    digitalWrite(_csm, HIGH);
    SPI.begin();
    if (!_magSensor.begin_SPI(_csm)) {
      return false;
    }
  } else {
    // Serial.println("softSPI");
    // Sofware SPI
    pinMode(_clk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    pinMode(_csxg, OUTPUT);
    pinMode(_csm, OUTPUT);
    digitalWrite(_csxg, HIGH);
    digitalWrite(_csm, HIGH);
    digitalWrite(_clk, HIGH);
    if (!_magSensor.begin_SPI(_csm, _clk, _miso, _mosi)) { // soft SPI
      return false;
    }
  }

  // soft reset & reboot accel/gyro
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG8, 0x05);

  delay(10);

  uint8_t id = read8(XGTYPE, LSM9DS1_REGISTER_WHO_AM_I_XG);
  if (id != LSM9DS1_XG_ID)
    return false;

  // enable gyro continuous
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0); // on XYZ

  // Enable the accelerometer continous
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38); // enable X Y and Z axis
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG6_XL,
         0xC0); // 1 KHz out data rate, BW set by ODR, 408Hz anti-aliasing

  // enable mag continuous
  _magSensor.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  // Set default ranges for the various sensors
  setupAccel(LSM9DS1_ACCELRANGE_2G);
  setupMag(LSM9DS1_MAGGAIN_4GAUSS);
  setupGyro(LSM9DS1_GYROSCALE_245DPS);

  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief Read all four sensor subcomponents
*/
/**************************************************************************/
void Adafruit_LSM9DS1::read() {
  /* Read all the sensors. */
  readAccel();
  readGyro();
  readTemp();
  readMag();
}

/**************************************************************************/
/*!
    @brief Read the sensor magnetometer sensor component
*/
/**************************************************************************/
void Adafruit_LSM9DS1::readMag() {
  _magSensor.read();
  magData.x = _magSensor.x;
  magData.y = _magSensor.y;
  magData.z = _magSensor.z;
}

/**************************************************************************/
/*!
    @brief Read the sensor accelerometer sensor component
*/
/**************************************************************************/
void Adafruit_LSM9DS1::readAccel() {
  // Read the accelerometer
  byte buffer[6];
  readBuffer(XGTYPE, 0x80 | LSM9DS1_REGISTER_OUT_X_L_XL, 6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8;
  xhi |= xlo;
  yhi <<= 8;
  yhi |= ylo;
  zhi <<= 8;
  zhi |= zlo;
  accelData.x = xhi;
  accelData.y = yhi;
  accelData.z = zhi;
}

/**************************************************************************/
/*!
    @brief Read the sensor gyroscope sensor component
*/
/**************************************************************************/
void Adafruit_LSM9DS1::readGyro() {
  // Read gyro
  byte buffer[6];
  readBuffer(XGTYPE, 0x80 | LSM9DS1_REGISTER_OUT_X_L_G, 6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8;
  xhi |= xlo;
  yhi <<= 8;
  yhi |= ylo;
  zhi <<= 8;
  zhi |= zlo;

  gyroData.x = xhi;
  gyroData.y = yhi;
  gyroData.z = zhi;
}

/**************************************************************************/
/*!
    @brief Read the sensor temperature sensor component
*/
/**************************************************************************/
void Adafruit_LSM9DS1::readTemp() {
  // Read temp sensor
  byte buffer[2];
  readBuffer(XGTYPE, 0x80 | LSM9DS1_REGISTER_TEMP_OUT_L, 2, buffer);
  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];

  xhi <<= 8;
  xhi |= xlo;

  // Shift values to create properly formed integer (low byte first)
  temperature = xhi;
}

/**************************************************************************/
/*!
    @brief Configure the accelerometer ranging
    @param range Can be LSM9DS1_ACCELRANGE_2G, LSM9DS1_ACCELRANGE_4G,
    LSM9DS1_ACCELRANGE_8G, LSM9DS1_ACCELRANGE_16G
*/
/**************************************************************************/
void Adafruit_LSM9DS1::setupAccel(lsm9ds1AccelRange_t range) {
  uint8_t reg = read8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG6_XL);
  reg &= ~(0b00011000);
  reg |= range;
  // Serial.println("set range: ");
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG6_XL, reg);

  switch (range) {
  case LSM9DS1_ACCELRANGE_2G:
    _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;
    break;
  case LSM9DS1_ACCELRANGE_4G:
    _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_4G;
    break;
  case LSM9DS1_ACCELRANGE_8G:
    _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_8G;
    break;
  case LSM9DS1_ACCELRANGE_16G:
    _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_16G;
    break;
  }
}

/**************************************************************************/
/*!
    @brief Configure the magnetometer gain
    @param gain Can be LSM9DS1_MAGGAIN_4GAUSS, LSM9DS1_MAGGAIN_8GAUSS,
    LSM9DS1_MAGGAIN_12GAUSS, LSM9DS1_MAGGAIN_16GAUSS
*/
/**************************************************************************/
void Adafruit_LSM9DS1::setupMag(lsm9ds1MagGain_t gain) {
  switch (gain) {
  case LSM9DS1_MAGGAIN_4GAUSS:
    _magSensor.setRange(LIS3MDL_RANGE_4_GAUSS);
    break;
  case LSM9DS1_MAGGAIN_8GAUSS:
    _magSensor.setRange(LIS3MDL_RANGE_8_GAUSS);
    break;
  case LSM9DS1_MAGGAIN_12GAUSS:
    _magSensor.setRange(LIS3MDL_RANGE_12_GAUSS);
    break;
  case LSM9DS1_MAGGAIN_16GAUSS:
    _magSensor.setRange(LIS3MDL_RANGE_16_GAUSS);
    break;
  }
}

/**************************************************************************/
/*!
    @brief Configure the gyroscope scaling
    @param scale Can be LSM9DS1_GYROSCALE_245DPS, LSM9DS1_GYROSCALE_500DPS
    or LSM9DS1_GYROSCALE_2000DPS
*/
/**************************************************************************/
void Adafruit_LSM9DS1::setupGyro(lsm9ds1GyroScale_t scale) {
  uint8_t reg = read8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG1_G);
  reg &= ~(0b00110000);
  reg |= scale;
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG1_G, reg);

  switch (scale) {
  case LSM9DS1_GYROSCALE_245DPS:
    _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_245DPS;
    break;
  case LSM9DS1_GYROSCALE_500DPS:
    _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_500DPS;
    break;
  case LSM9DS1_GYROSCALE_2000DPS:
    _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_2000DPS;
    break;
  }
}

/***************************************************************************
 UNIFIED SENSOR FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Gets the most recent accel sensor events for all 4 sensors
    @param accelEvent The accelerometer event object we will fill, pass NULL to
   skip
    @param magEvent The magnetometer event object we will fill, pass NULL to
   skip
    @param gyroEvent The gyroscope event object we will fill, pass NULL to skip
    @param tempEvent The temperature event object we will fill, pass NULL to
   skip
    @returns True on successful reads
*/
/**************************************************************************/
bool Adafruit_LSM9DS1::getEvent(sensors_event_t *accelEvent,
                                sensors_event_t *magEvent,
                                sensors_event_t *gyroEvent,
                                sensors_event_t *tempEvent) {
  /* Grab new sensor reading and timestamp. */
  read();
  uint32_t timestamp = millis();

  /* Update appropriate sensor events. */
  if (accelEvent)
    getAccelEvent(accelEvent, timestamp);
  if (magEvent)
    _magSensor.getEvent(magEvent);
  if (gyroEvent)
    getGyroEvent(gyroEvent, timestamp);
  if (tempEvent)
    getTempEvent(tempEvent, timestamp);

  return true;
}

/**************************************************************************/
/*!
    @brief Gets the sensor_t data for all 4 sub-sensors at once call
    @param accel The accelerometer sensor_t object we will fill, pass NULL to
   skip
    @param mag The magnetometer sensor_t object we will fill, pass NULL to skip
    @param gyro The gyroscope sensor_t object we will fill, pass NULL to skip
    @param temp The temperature sensor_t object we will fill, pass NULL to skip
*/
/**************************************************************************/
void Adafruit_LSM9DS1::getSensor(sensor_t *accel, sensor_t *mag, sensor_t *gyro,
                                 sensor_t *temp) {
  /* Update appropriate sensor metadata. */
  if (accel)
    getAccelSensor(accel);
  if (mag)
    _magSensor.getSensor(mag);
  if (gyro)
    getGyroSensor(gyro);
  if (temp)
    getTempSensor(temp);
}

/**************************************************************************/
/*!
    @brief Fill in the details about the most recent accelerometer data read
    @param event The sensor_event_t object we will fill!
    @param timestamp Unused
*/
/**************************************************************************/
void Adafruit_LSM9DS1::getAccelEvent(sensors_event_t *event,
                                     uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _lsm9dso_sensorid_accel;
  event->type = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = timestamp;
  event->acceleration.x = accelData.x * _accel_mg_lsb;
  event->acceleration.x /= 1000;
  event->acceleration.x *= SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = accelData.y * _accel_mg_lsb;
  event->acceleration.y /= 1000;
  event->acceleration.y *= SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = accelData.z * _accel_mg_lsb;
  event->acceleration.z /= 1000;
  event->acceleration.z *= SENSORS_GRAVITY_STANDARD;
}

/**************************************************************************/
/*!
    @brief Fill in the details about the most recent magnetometer data read
    @param event The sensor_event_t object we will fill!
    @param timestamp Unused
*/
/**************************************************************************/
void Adafruit_LSM9DS1::getMagEvent(sensors_event_t *event, uint32_t timestamp) {
  _magSensor.getEvent(event);
}

/**************************************************************************/
/*!
    @brief Fill in the details about the most recent gyroscope data read
    @param event The sensor_event_t object we will fill!
    @param timestamp The millis timestamp when the read occured
*/
/**************************************************************************/
void Adafruit_LSM9DS1::getGyroEvent(sensors_event_t *event,
                                    uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _lsm9dso_sensorid_accel;
  event->type = SENSOR_TYPE_GYROSCOPE;
  event->timestamp = timestamp;
  event->gyro.x = gyroData.x * _gyro_dps_digit * SENSORS_DPS_TO_RADS;
  event->gyro.y = gyroData.y * _gyro_dps_digit * SENSORS_DPS_TO_RADS;
  event->gyro.z = gyroData.z * _gyro_dps_digit * SENSORS_DPS_TO_RADS;
}

/**************************************************************************/
/*!
    @brief Fill in the details about the most recent temperature data read
    @param event The sensor_event_t object we will fill!
    @param timestamp The millis timestamp when the read occured
*/
/**************************************************************************/
void Adafruit_LSM9DS1::getTempEvent(sensors_event_t *event,
                                    uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _lsm9dso_sensorid_temp;
  event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = timestamp;
  // This is just a guess since the staring point (21C here) isn't documented :(
  event->temperature = 21.0 + (float)temperature / 8;
  // event->temperature /= LSM9DS1_TEMP_LSB_DEGREE_CELSIUS;
}

/**************************************************************************/
/*!
    @brief Fill in the details about the accelerometer sensor component
    @param sensor The sensor_t object we will fill!
*/
/**************************************************************************/
void Adafruit_LSM9DS1::getAccelSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "LSM9DS1_A", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _lsm9dso_sensorid_accel;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->max_value = 156.8;      // +16 g = 156.8 m/s^s
  sensor->min_value = -156.8;     // -16 g = 156.8 m/s^s
  sensor->resolution = 0.0005978; // 0.061 mg = 0.0005978 m/s^2
}

void Adafruit_LSM9DS1::getMagSensor(sensor_t *sensor) {
  _magSensor.getSensor(sensor);
}

/**************************************************************************/
/*!
    @brief Fill in the details about the gyroscope sensor component
    @param sensor The sensor_t object we will fill!
*/
/**************************************************************************/
void Adafruit_LSM9DS1::getGyroSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "LSM9DS1_G", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _lsm9dso_sensorid_gyro;
  sensor->type = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay = 0;
  sensor->max_value = 34.91;             // +2000 dps = 34.906586 rad/s
  sensor->min_value = -34.91;            // "
  sensor->resolution = 0.00015271631375; // 8.75 mdps = 0.00015271631375 rad/s
}

/**************************************************************************/
/*!
    @brief Fill in the details about the temperature sensor component
    @param sensor The sensor_t object we will fill!
*/
/**************************************************************************/
void Adafruit_LSM9DS1::getTempSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "LSM9DS1_T", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _lsm9dso_sensorid_temp;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->max_value = 0.0;  // ToDo
  sensor->min_value = 0.0;  // ToDo
  sensor->resolution = 0.0; // ToDo
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM9DS1::write8(boolean type, byte reg, byte value) {
  byte address, _cs;

  if (type == MAGTYPE) {
    address = LSM9DS1_ADDRESS_MAG;
    _cs = _csm;
  } else {
    address = LSM9DS1_ADDRESS_ACCELGYRO;
    _cs = _csxg;
  }
  if (_i2c) {
    _wire->beginTransmission(address);
    _wire->write(reg);
    _wire->write(value);
    _wire->endTransmission();
    /*
    Serial.print("0x"); Serial.print(address, HEX);
    Serial.print(" $"); Serial.print(reg, HEX); Serial.print(" = ");
    Serial.println(value, HEX);
    */
  } else {
    digitalWrite(_cs, LOW);
    if (_clk == -1) // hardware SPI
      SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
    // set address
    spixfer(reg & 0x7F); // write data
    spixfer(value);
    if (_clk == -1) // hardware SPI
      SPI.endTransaction();
    digitalWrite(_cs, HIGH);
  }
}

byte Adafruit_LSM9DS1::read8(boolean type, byte reg) {
  uint8_t value;

  readBuffer(type, reg, 1, &value);

  return value;
}

byte Adafruit_LSM9DS1::readBuffer(boolean type, byte reg, byte len,
                                  uint8_t *buffer) {
  byte address, _cs;

  if (type == MAGTYPE) {
    address = LSM9DS1_ADDRESS_MAG;
    _cs = _csm;
  } else {
    address = LSM9DS1_ADDRESS_ACCELGYRO;
    _cs = _csxg;
  }

  if (_i2c) {
    _wire->beginTransmission(address);
    _wire->write(reg);
    _wire->endTransmission();
    if (_wire->requestFrom(address, (byte)len) != len) {
      return 0;
    }

    /*
      Serial.print("0x"); Serial.print(address, HEX);
      Serial.print(" $"); Serial.print(reg, HEX); Serial.print(": ");
    */

    for (uint8_t i = 0; i < len; i++) {
      buffer[i] = _wire->read();
      // Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    // Serial.println();

  } else {
    if (_clk == -1) // hardware SPI
      SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
    else
      digitalWrite(_clk, HIGH);
    // set address

    digitalWrite(_cs, LOW);

    spixfer(reg | 0x80); // readdata
    for (uint8_t i = 0; i < len; i++) {
      buffer[i] = spixfer(0);
    }
    if (_clk == -1) // hardware SPI
      SPI.endTransaction();
    else
      digitalWrite(_clk, HIGH);
    digitalWrite(_cs, HIGH);
  }

  return len;
}

uint8_t Adafruit_LSM9DS1::spixfer(uint8_t data) {

  if (_clk == -1) {
    // Serial.println("Hardware SPI");
    return SPI.transfer(data);
  } else {
    // Serial.println("Software SPI");
    uint8_t reply = 0;
    for (int i = 7; i >= 0; i--) {
      reply <<= 1;
      digitalWrite(_clk, LOW);
      digitalWrite(_mosi, data & (1 << i));
      digitalWrite(_clk, HIGH);
      if (digitalRead(_miso))
        reply |= 1;
    }
    return reply;
  }
}
