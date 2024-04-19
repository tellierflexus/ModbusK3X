/*

	ModbusK3X.h - Library for communicating with Senseair K3X (K30, K33) sensors with modbus

	Created by Alexandre Tellier for EPFL SENSE Laboratory, 2024.

	Released into the public domain.

	Big up to Sailowtech <3

*/

#include "Arduino.h"

#include "ModbusK3X.h"

byte _rcvBuffer[BUFFER_SIZE];


ModbusK3X::ModbusK3X(byte address, HardwareSerial& serial, int RS485Pin, int baud) : _SerialBus(serial) {
	_address = address;
	_RS485Pin = RS485Pin; //9999 Default value will be considered as no pin, therefore no need to activate transmission on a RS485 transceiver like MAX485
	serial.begin(baud, SERIAL_8N1);
	_SerialBus = serial;
	_baud = baud;
	_T1_5 = 15000000/baud; 
	_T3_5 = 35000000/baud;
}

void ModbusK3X::begin(){
	if (_RS485Pin != 9999){
		_RS485mode = true;
		pinMode(_RS485Pin, OUTPUT);
		digitalWrite(_RS485Pin, LOW);
	}
	else {
		_RS485mode = false;
	}
	
}

void ModbusK3X::modbusCRC(byte *modbusMessage, int len){
/*

	This code was adapted from https://forum.arduino.cc/t/modbus-crc-16-calculator-string-to-byte-function/405613

*/

  //Calc the raw_msg_data_byte CRC code
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)modbusMessage[pos];          // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  modbusMessage[len] = (byte) (0x00FF & crc);
  modbusMessage[len+1] = (byte) (crc >> 8);
}

void ModbusK3X::sendModbusMessage(byte *modbusMessage, int len){ 
	/* 
	Allow generic message sending to the sensor

	TODO, preload the message with another function to prevent buffer overflow,
	as the user define the array size of the message and we will add 2 bytes for CRC
	

	*/
	if (_RS485mode){
		digitalWrite(_RS485Pin, HIGH);
	}
	modbusCRC(modbusMessage, len);
	int nbrSent = _SerialBus.write(modbusMessage, len+2); // Len+2 as we added 2 bytes of CRC
	_SerialBus.flush();
	delayMicroseconds(_T3_5);
	if (_RS485mode){
		digitalWrite(_RS485Pin, LOW);
	}


}

int ModbusK3X::receiveMessage(){
    unsigned char buffer = 0;
    unsigned char overflowFlag = 0;
    while (_SerialBus.available()) {
        // The maximum number of bytes is limited to the serial buffer size of 128 bytes
        // If more bytes is received than the BUFFER_SIZE the overflow flag will be set and the
        // serial buffer will be red untill all the data is cleared from the receive buffer,
        // while the slave is still responding.
        if (overflowFlag)
            _SerialBus.read();
        else {
            if (buffer == BUFFER_SIZE)
                overflowFlag = 1;

            _rcvBuffer[buffer] = _SerialBus.read();
            buffer++;
        }
        int T1_5 = 15000000/9600; 
        delayMicroseconds(T1_5); // inter character time out
    }
  if (buffer == 0){
    return 0;
  }
  else {
    uint16_t crc_received = (_rcvBuffer[buffer-2] << 8) + _rcvBuffer[buffer-1];  
    modbusCRC(_rcvBuffer, buffer-2); //function modify the buffer to add the crc
    uint16_t crc_calculated = (_rcvBuffer[buffer-2] << 8) + _rcvBuffer[buffer-1]; 
    if (crc_received == crc_calculated){  
      return buffer;
    }
    else {
      return 0;
    }
  }
}

bool ModbusK3X::startSingleMeasurement(){

	/*

		Send a command to start measurement on sensor and return true if the sensor acknoledged 

	*/

	byte modbusPacket1[8] = {_address, 0x41, 0x00, 0x60, 0x01, 0x35};
	sendModbusMessage(modbusPacket1, 6);
	delay(30);
	if (receiveMessage() == 0){
		return false;
	}
	byte modbusPacket2[7] = {_address, 0x44, 0x00, 0x1D, 0x01};
	sendModbusMessage(modbusPacket2, 5);
	delay(30);
	int _nbrRcv = receiveMessage();
	if (_nbrRcv != 6){
		return false;
	}
	if ((_rcvBuffer[3] & 0x20)  == 0x20){///We check that the bit number 5 (0x20 = 00010000) is set to 1
		return true;
	} 
	else {
		return false;
	}
}

bool ModbusK3X::startContinuousMeasurement(){

	/*

		Send a command to start measurement on sensor and return true if the sensor acknoledged 

	*/

	byte modbusPacket1[8] = {_address, 0x41, 0x00, 0x60, 0x01, 0x30};
	sendModbusMessage(modbusPacket1, 6);
	delay(30);
	if (receiveMessage() == 0){
		return false;
	}
	byte modbusPacket2[7] = {_address, 0x44, 0x00, 0x1D, 0x01};
	sendModbusMessage(modbusPacket2, 5);
	delay(30);
	int _nbrRcv = receiveMessage();
	if (_nbrRcv != 6){
		return false;
	}
	if ((_rcvBuffer[3] & 0x40)  == 0x40){///We check that the bit number 6 (0x40 = 00100000) is set to 1
		return true;
	} 
	else {
		return false;
	}
}

bool ModbusK3X::stopContinuousMeasurement(){

	/*

		Send a command to start measurement on sensor and return true if the sensor acknoledged 

	*/

	byte modbusPacket1[8] = {_address, 0x41, 0x00, 0x60, 0x01, 0x31};
	sendModbusMessage(modbusPacket1, 6);
	delay(30);
	if (receiveMessage() == 0){
		return false;
	}
	byte modbusPacket2[7] = {_address, 0x44, 0x00, 0x1D, 0x01};
	sendModbusMessage(modbusPacket2, 5);
	delay(30);
	int _nbrRcv = receiveMessage();
	if (_nbrRcv != 6){
		return false;
	}
	if ((_rcvBuffer[3] & 0x40)  == 0x00){///We check that the bit number 6 (0x40 = 00100000) is set to 1
		return true;
	} 
	else {
		return false;
	}
}


uint16_t ModbusK3X::retrieveCO2Value(){
	byte modbusPacket[7] = {_address, 0x44, 0x0, 0x08, 0x02};
	sendModbusMessage(modbusPacket, 5);
	delay(30);
	if (receiveMessage() != 7){
		return 0;
	}
	return  (uint16_t) ((_rcvBuffer[3] >> 8) +_rcvBuffer[4]);
}

uint16_t ModbusK3X::retrieveTemperatureValue(){
	byte modbusPacket[7] = {_address, 0x44, 0x0, 0x12, 0x02};
	sendModbusMessage(modbusPacket, 5);
	delay(30);
	if (receiveMessage() != 7){
		return 0;
	}
	return  (uint16_t) ((_rcvBuffer[3] >> 8) +_rcvBuffer[4]);
}

uint16_t ModbusK3X::retrieveHumidityValue(){
	byte modbusPacket[7] = {_address, 0x44, 0x0, 0x14, 0x02};
	sendModbusMessage(modbusPacket, 5);
	delay(30);
	if (receiveMessage() != 7){
		return 0;
	}
	return  (uint16_t) ((_rcvBuffer[3] >> 8) +_rcvBuffer[4]);
}

bool ModbusK3X::startZeroCalibration(){
	byte modbusPacket1[9] = {_address, 0x41, 0x0, 0x42, 0x02, 0x7C, 0x07};
	sendModbusMessage(modbusPacket1, 9);
	delay(30);
	if (receiveMessage() == 0){
		return false;
	}
	delay(160000);
	byte modbusPacket2[7] = {_address, 0x44, 0x00, 0x40, 0x01};
	sendModbusMessage(modbusPacket2, 5);
	delay(30);
	int _nbrRcv = receiveMessage();
	if (_nbrRcv != 6){
		return false;
	}
	if ((_rcvBuffer[3] & 0x40)  == 0x40){//We check that the bit number 6 (0x40 = 00100000) is set to 1
		return true;
	} 
	else {
		return false;
	}	
}

bool ModbusK3X::startBackgroundCalibration(){
	byte modbusPacket1[9] = {_address, 0x41, 0x0, 0x42, 0x02, 0x7C, 0x06};
	sendModbusMessage(modbusPacket1, 9);
	delay(30);
	if (receiveMessage() == 0){
		return false;
	}
	delay(160000);
	byte modbusPacket2[7] = {_address, 0x44, 0x00, 0x40, 0x01};
	sendModbusMessage(modbusPacket2, 5);
	delay(30);
	int _nbrRcv = receiveMessage();
	if (_nbrRcv != 6){
		return false;
	}
	if ((_rcvBuffer[3] & 0x20)  == 0x20){//We check that the bit number 5 (0x20 = 00010000) is set to 1
		return true;
	} 
	else {
		return false;
	}	
}



