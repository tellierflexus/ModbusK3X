# Goals
This library aims to provide an easy way to communicate with Senseair sensors K3X (K33, K30...) using modbus, from an arduino based platform.

# Features
* Start single measurement
* Start/Stop continuous measurements
* Read CO2 value
* Read temperature and relative humidity values (K33 sensor only)
* Start zero and background calibration
* RS485 transceiver control (Control of the transmission enable pin)
* Allow selection of Serial Bus

This library was tested on the following platforms :
* Industruino D21G


# Usage
First, clone this repository and put it in a folder inside your libraries folder.

Include the library in your arduino sketch : 
```C++
#include <ModbusK3X.h>
```

Create an instance of the sensor (default parameters are shown in documentation section):
```C++

ModbusK3X sensor()

void setup(){
...
```
Initialize the communication with the sensor :
```C++
void setup(){
  sensor.begin()
}
```
Take a single CO2 measurement : 
```C++
void loop(){
  ...
  sensor.startSingleMeasurement(); //Launch all measurements (CO2, RH, Temperature...)
  delay(16000); //We wait for the sensor to finish measurement
  int co2_value = retrieveCO2Value(); 
  ...
}
```

#Available functions :
```C++
sendModbusMessage();
receiveMessage();
getRawOutput();
startSingleMeasurement();
startContinuousMeasurement();
stopContinuousMeasurement();
retrieveCO2Value();
retrieveTemperatureValue();
retrieveHumidityValue();
startZeroCalibration();
startBackgroundCalibration();
```

Retrieve temperature value :
```C++
  int temperature_value = retrieveTemperatureValue(); 
```

Retrieve relative humidity value :
```C++
  float RH = ((float) retrieveHumidityValue()/100);
  //Temperature and relative humidity are stored as 4 digits values in an int.
  //Therefore you should convert to fload and divide by 100 : 2333 -> 23.33
  
```

