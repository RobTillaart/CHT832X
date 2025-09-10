//
//    FILE: CHT832X.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.1
// PURPOSE: Arduino library for CHT832X temperature and humidity sensor
//     URL: https://github.com/RobTillaart/CHT832X


#include "CHT832X.h"


//  COMMANDS datasheet  Page 12/13
const uint16_t CHT832X_CMD_READ              = 0xE000;

const uint16_t CHT832X_CMD_ENABLE_HEATER     = 0x306D;
const uint16_t CHT832X_CMD_DISABLE_HEATER    = 0x3066;
const uint16_t CHT832X_CMD_CONFIG_HEATER     = 0x306E;

const uint16_t CHT832X_CMD_READ_STATUS       = 0xF32D;
const uint16_t CHT832X_CMD_CLEAR_STATUS      = 0x3041;

const uint16_t CHT832X_CMD_SOFTWARE_RESET    = 0x30A2;

const uint16_t CHT832X_CMD_READ_NIST_BASE    = 0x3683;
const uint16_t CHT832X_CMD_READ_MANUFACTURER = 0x3781;

//  CONVERSION TIMING
const uint8_t CHT832X_READ_DELAY = 60;


/////////////////////////////////////////////////////
//
// PUBLIC
//
CHT832X::CHT832X(const uint8_t address, TwoWire *wire)
{
  _wire    = wire;
  _address = address;
}


int CHT832X::begin()
{
  //  check address range
  if ((_address < 0x44) || (_address > 0x47))
  {
    _error = CHT832X_ERROR_ADDR;
    return _error;
  }
  if (! isConnected()) 
  {
    _error = CHT832X_ERROR_CONNECT;
    return _error;
  }
  _error = CHT832X_OK;
  return _error;
}


bool CHT832X::isConnected()
{
  _wire->beginTransmission(_address);
  return (_wire->endTransmission() == 0);
}


uint8_t CHT832X::getAddress()
{
  return _address;
}


////////////////////////////////////////////////
//
//  READ THE SENSOR
//
int CHT832X::read()
{
  //  do not read too fast
  if (millis() - _lastRead < 1000)
  {
    _error = CHT832X_ERROR_LASTREAD;
    return _error;
  }
  _lastRead = millis();

  //  READ PART
  uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };
  _readRegister(CHT832X_CMD_READ, data, 6, CHT832X_READ_DELAY);
  if (_error != CHT832X_OK)
  {
    return _error;
  }

  //  TEMPERATURE PART
  _error = CHT832X_OK;
  int16_t tmp = (data[0] << 8 | data[1]);
  _temperature = -45 + (175.0 / 65535) * tmp;
  //  Handle temperature offset.
  if (_tempOffset != 0.0)
  {
    _temperature += _tempOffset;
  }
  //  CHECK CRC TEMPERATURE
  if (_crc8(tmp) != data[2])
  {
    _error = CHT832X_ERROR_CRC;
    //  fall through as value might be correct.
  }

  //  HUMIDITY PART
  tmp = (data[3] << 8 | data[4]);
  _humidity = (100.0 / 65535) * tmp;
  if (_humOffset  != 0.0)
  {
    _humidity += _humOffset;
    //  handle out of range - clipping.
    if (_humidity < 0.0)   _humidity = 0.0;
    if (_humidity > 100.0) _humidity = 100.0;
  }
  //  CHECK CRC HUMIDITY
  if (_crc8(tmp) != data[5])
  {
    _error = CHT832X_ERROR_CRC;
    //  fall through as value might be correct.
  }
  return _error;
}


uint32_t CHT832X::lastRead()
{
  return _lastRead;
}


float CHT832X::getHumidity()
{
  return _humidity;
}


float CHT832X::getTemperature()
{
  return _temperature;
}


////////////////////////////////////////////////
//
//  OFFSET
//
void CHT832X::setHumidityOffset(float offset)
{
  _humOffset = offset;
}


void CHT832X::setTemperatureOffset(float offset)
{
  _tempOffset = offset;
}


float CHT832X::getHumidityOffset()
{
  return _humOffset;
}


float CHT832X::getTemperatureOffset()
{
  return _tempOffset;
}


////////////////////////////////////////////////
//
//  HEATER - datasheet Page 16/17
//
void CHT832X::enableHeater()
{
  _writeRegister(CHT832X_CMD_ENABLE_HEATER, NULL, 0);
}


void CHT832X::enableHeaterFull()
{
  uint8_t buffer[3] = {0x3F, 0xFF, 0x06};
  _writeRegister(CHT832X_CMD_CONFIG_HEATER, buffer, 3);
}


void CHT832X::enableHeaterHalf()
{
  uint8_t buffer[3] = {0x03, 0xFF, 0x00};
  _writeRegister(CHT832X_CMD_CONFIG_HEATER, buffer, 3);
}


void CHT832X::enableHeaterQuarter()
{
  uint8_t buffer[3] = {0x00, 0x9F, 0x96};
  _writeRegister(CHT832X_CMD_CONFIG_HEATER, buffer, 3);
}


void CHT832X::disableHeater()
{
 _writeRegister(CHT832X_CMD_DISABLE_HEATER, NULL, 0);
}



////////////////////////////////////////////////
//
//  STATUS - datasheet Page 17
//
uint16_t CHT832X::getStatusRegister()
{
  uint8_t buffer[3] = {0, 0, 0};
  _readRegister(CHT832X_CMD_READ_STATUS, buffer, 3);
  uint16_t value = buffer[0] * 256 + buffer[1];
  //  check CRC.
  _error = CHT832X_OK;
  if (_crc8(value) != buffer[2])
  {
    _error = CHT832X_ERROR_CRC;
    //  Serial.println("debug: CRC-error");
  }
  //  mask reserved bits? (Page 17)
  //  value &= 0x2013
  return value;
}


void CHT832X::clearStatusRegister()
{
  _writeRegister(CHT832X_CMD_CLEAR_STATUS, NULL, 0);
}


////////////////////////////////////////////////
//
//  SOFTWARE RESET
//
void CHT832X::softwareReset()
{
  _writeRegister(CHT832X_CMD_SOFTWARE_RESET, NULL, 0);
}


////////////////////////////////////////////////
//
//  META DATA
//
uint16_t CHT832X::getNIST(uint8_t id)
{
  if (id > 2) return 0;
  uint8_t buffer[3] = {0, 0, 0};
  _readRegister(CHT832X_CMD_READ_NIST_BASE + id, buffer, 3);
  uint16_t value = buffer[0] * 256 + buffer[1];
  //  check CRC.
  _error = CHT832X_OK;
  if (_crc8(value) != buffer[2])
  {
    _error = CHT832X_ERROR_CRC;
    //  Serial.println("debug: CRC-error");
  }
  return value;
}

uint16_t CHT832X::getManufacturer()
{
  uint8_t buffer[3] = {0, 0};
  _readRegister(CHT832X_CMD_READ_MANUFACTURER, buffer, 3);
  uint16_t value = buffer[0] * 256 + buffer[1];
  //  check CRC.
  _error = CHT832X_OK;
  if (_crc8(value) != buffer[2])
  {
    _error = CHT832X_ERROR_CRC;
    //  Serial.println("debug: CRC-error");
  }
  return value;
}


int CHT832X::getError()
{
  int e = _error;
  _error = CHT832X_OK;
  return e;
}


////////////////////////////////////////////////
//
//  PRIVATE
//
int CHT832X::_readRegister(uint16_t command, uint8_t * buf, uint8_t size, uint8_t del)
{
  _wire->beginTransmission(_address);
  _wire->write(command >> 8);
  _wire->write(command & 0xFF);
  int n = _wire->endTransmission();
  if (n != 0)
  {
    //  Serial.println(n);
    _error = CHT832X_ERROR_I2C;
    return _error;
  }

  if (del > 0) delay(del);

  n = _wire->requestFrom(_address, size);
  if (n != size)
  {
    _error = CHT832X_ERROR_I2C;
    return _error;
  }

  for (uint8_t i = 0; i < size; i++)
  {
    buf[i] = _wire->read();
  }
  _error = CHT832X_OK;
  return _error;
}


int CHT832X::_writeRegister(uint16_t command, uint8_t * buf, uint8_t size)
{
  _wire->beginTransmission(_address);
  _wire->write(command >> 8);
  _wire->write(command & 0xFF);
  for (uint8_t i = 0; i < size; i++)
  {
    _wire->write(buf[i]);
  }
  int n = _wire->endTransmission();
  if (n != 0)
  {
    //  Serial.println(n);
    _error = CHT832X_ERROR_I2C;
    return _error;
  }
  _error = CHT832X_OK;
  return _error;
}


uint8_t CHT832X::_crc8(uint16_t data)
{
  uint8_t _crc = 0xFF;
  uint8_t value = (data >> 8);  //  MSB first
  uint8_t n = 2;
  while (n--)
  {
    _crc ^= value;
    for (uint8_t i = 8; i; i--)
    {
      if (_crc & (1 << 7))
      {
        _crc <<= 1;
        _crc ^= 0x31;
      }
      else
      {
        _crc <<= 1;
      }
    }
    value = (data & 0xFF);  //  LSB
  }
  return _crc;
}

//  -- END OF FILE --

