/*

  ModbusK3X.h - Library for communicating with Senseair K3X (K30, K33) sensors with modbus

  Created by Alexandre Tellier for EPFL SENSE Laboratory, 2024.

  Released into the public domain.

  Big up to Sailowtech <3

*/

#ifndef ModbusK3X_h

#define ModbusK3X_h
#define BUFFER_SIZE 128

#include "Arduino.h"
//#include <SoftwareSerial.h>

class ModbusK3X{
public:
  ModbusK3X(byte address, HardwareSerial& device, int RS485Pin = 9999);
  //ModbusK3X(byte address, SoftwareSerial& device, int RS485Pin = 9999);
  void begin(int baud = 9600);
  void modbusCRC(byte *modbusMessage, int len);
  void sendModbusMessage(byte *modbusMessage, int len);
  int receiveMessage();
  int getRawOutput(byte *outputBuffer);
  byte getErrorCode();
  uint8_t startSingleMeasurement();
  bool startContinuousMeasurement();
  bool stopContinuousMeasurement();
  uint16_t retrieveCO2Value();
  uint16_t retrieveTemperatureValue();
  uint16_t retrieveHumidityValue();
  int readMultipleRegisters(int function_code, uint16_t register_start_address, uint16_t nbr_registers, uint16_t* data_array);
  int writeMultipleRegisters(uint16_t register_start_address, uint16_t nbr_registers,  uint16_t* data_array);
  bool startZeroCalibration();
  bool startBackgroundCalibration();

private:
  byte _address;
  int _RS485Pin;
  bool _RS485mode;
  int _T1_5;
  int _T3_5;
  HardwareSerial* _hwStream;
  //SoftwareSerial* _swStream;
  Stream* _stream;
  byte _errorCode;
};

#endif