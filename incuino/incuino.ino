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
#include <DallasTemperature.h>

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
DallasTemperature _sensors(&_ow); //Tell Dallas Temperature Library to use oneWire Library

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
  _sensors.requestTemperatures();
  return _sensors.getTempCByIndex(0);
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

SensorThread _tempSensor = SensorThread();

void setup(void) {
  delay(1000);
  Serial.begin(9600);
  delay(1000);
  _sensors.begin();
  pinMode(RELAY_PIN, OUTPUT);
  _tempSensor.pin = SENSOR_PIN;
  _tempSensor.setInterval(10000);
  _controller.add(&_tempSensor);
  Timer1.initialize(10000000);
  Timer1.attachInterrupt(timerCallback);
  Timer1.start();
  _windowStartTime = millis();
  _setpoint = 28.0;
  _pid.SetOutputLimits(0, WINDOW_SIZE); //tell the PID to range between 0 and the full window size
  _pid.SetMode(AUTOMATIC); //turn the PID on
}

void loop(void) {
  float tempOrErrow = _tempSensor.value;
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
