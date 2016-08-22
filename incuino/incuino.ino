/* 
 # Temperature
 OneWire DS18S20, DS18B20, DS1822 Temperature Example

 http://www.pjrc.com/teensy/td_libs_OneWire.html

 The DallasTemperature library can do all this work for you!
 http://milesburton.com/Dallas_Temperature_Control_Library

 # Relay

 Connect 5V on Arduino to VCC on Relay Module
 Connect GND on Arduino to GND on Relay Module
 Connect GND on Arduino to the Common Terminal (middle terminal) on Relay Module.
*/

#include <OneWire.h>
#include <PID_v1.h>
#include <Thread.h>
#include <ThreadController.h>
#include <TimerOne.h>

#define RELAY_PIN 8   // Connect Digital Pin 8 on Arduino to ? on Relay Module
#define SENSOR_PIN 10
#define DELAY 1000
#define ERROR_RESPONSE -1000
#define ERROR_NO_MORE_ADDRESSES -2000
#define ERROR_CRC_IS_NOT_VALID -3000
#define ERROR_FAILED_TO_RESET -4000

const unsigned long WINDOW_SIZE = 100;
const unsigned long PID_INTERVAL = 60000;

unsigned long _windowStartTime;
double _setpoint, _input, _output;
double _kp = 70.0, _ki = 0.6, _kd = 0.6; // Initial tuning parameters
PID _pid(&_input, &_output, &_setpoint, _kp, _ki, _kd, DIRECT);
OneWire _ow(SENSOR_PIN);  // on pin 10 (a 4.7K resistor is necessary)

class Logger {

  public:
    void errorF(const char* msg, float value){
      Serial.print(msg);
      Serial.println(value);
    }
  
    void info(const char* msg) {
      Serial.println(msg);
    }

    void debug(const char* msg) {
      Serial.println(msg);
    }
  
    void infoF(const char* msg, float value){
      Serial.print(msg);
      Serial.println(value);
    }

    void infoD(const char* msg, double value){
      Serial.print(msg);
      Serial.println(value);
    }
    
    void infoI(const char* msg, unsigned int value){
      Serial.print(msg);
      Serial.println(value);
    }

    void infoL(const char* msg, unsigned int value){
      Serial.print(msg);
      Serial.println(value);
    }
};

Logger Log = Logger();

float getTemp() {
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  unsigned long startTime;

  _ow.reset_search();
  if ( !_ow.search(addr)) {
    _ow.reset_search();
    startTime = millis();
    while (millis() - startTime < 3000);
    return ERROR_NO_MORE_ADDRESSES;
  }
  if (OneWire::crc8(addr, 7) != addr[7]) {
    return ERROR_CRC_IS_NOT_VALID;
  }
  // the first ROM byte indicates which chip
  int type_s = getChipType(addr[0]);
  if (!_ow.reset()) {
    return ERROR_FAILED_TO_RESET;
  }
  _ow.select(addr);
  _ow.write(0x44, 1);        // start conversion, with parasite power on at the end
  startTime = millis();
  while (millis() - startTime < 1000); // maybe 750ms is enough, maybe not
  // we might do a _ow.depower() here, but the reset will take care of it.
  present = _ow.reset();
  if (!present) {
    Log.info("Failed to reset");
  }
  _ow.select(addr);
  _ow.write(0xBE);         // Read Scratchpad
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = _ow.read();
  }
  return convertTemp(data, type_s);
}

class SensorThread: public Thread {
  public:
    float value;
    int pin;

    void run() {
      value = getTemp();
      runned();
    }
};

ThreadController _controller = ThreadController();

void timerCallback() {
  _controller.run();
}

SensorThread tempSensor = SensorThread();

void setup(void) {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  tempSensor.pin = SENSOR_PIN;
  tempSensor.setInterval(10000);
  _controller.add(&tempSensor);
  Timer1.initialize(10000000);
  Timer1.attachInterrupt(timerCallback);
  Timer1.start();
  _windowStartTime = millis();
  _setpoint = 28.0;
  _pid.SetOutputLimits(0, WINDOW_SIZE); //tell the PID to range between 0 and the full window size
  _pid.SetMode(AUTOMATIC); //turn the PID on
}

int getChipType(int addr) {
  switch (addr) {
    case 0x10:
      Log.debug("Chip = DS18S20");  // or old DS1820
      return 1;
      break;
    case 0x28:
      Log.debug("Chip = DS18B20");
      return 0;
      break;
    case 0x22:
      Log.debug("Chip = DS1822");
      return 0;
      break;
    default:
      Log.debug("Device is not a DS18x20 family device.");
      return -1;
  }
}

float convertTemp(byte data[12], int type_s) {
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  }
  else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) {
      raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    }
    else if (cfg == 0x20) {
      raw = raw & ~3; // 10 bit res, 187.5 ms
    }
    else if (cfg == 0x40) {
      raw = raw & ~1; // 11 bit res, 375 ms}
      //// default is 12 bit resolution, 750 ms conversion time
    }
  }
  return (float) raw / 16.0;
}

void loop(void) {
  float tempOrErrow = tempSensor.value;
  if (tempOrErrow <= ERROR_RESPONSE) {
    Log.errorF("Code: ", tempOrErrow);
    return;
  }
  _windowStartTime = millis();
  _input = tempOrErrow;
  _pid.Compute();
  const float ratio = _output / WINDOW_SIZE;
  const unsigned long runtime = PID_INTERVAL * ratio; 
  Log.infoD("Input: ", _input); 
  Log.infoD("Output: ", _output); 

  Log.infoL("Heat ON: ", runtime); 
  digitalWrite(RELAY_PIN, LOW);
  while (millis() - _windowStartTime < runtime);
  
  Log.infoL("Heat OFF ", PID_INTERVAL - runtime); 
  digitalWrite(RELAY_PIN, HIGH);
  while (millis() - _windowStartTime < PID_INTERVAL);
}
