#include <OneWire.h>
#include <PID_v1.h>
#include <Thread.h>
#include <ThreadController.h>
#include <TimerOne.h>


/*
 OneWire DS18S20, DS18B20, DS1822 Temperature Example

 http://www.pjrc.com/teensy/td_libs_OneWire.html

 The DallasTemperature library can do all this work for you!
 http://milesburton.com/Dallas_Temperature_Control_Library

 The pid is designed to Output an analog value, but the relay can only be On/Off.
 It's essentially a really slow version of PWM.

 Connect 5V on Arduino to VCC on Relay Module
 Connect GND on Arduino to GND on Relay Module
 Connect GND on Arduino to the Common Terminal (middle terminal) on Relay Module.
*/

#define RELAY_PIN 8   // Connect Digital Pin 8 on Arduino to ? on Relay Module
#define SENSOR_PIN 10
#define DELAY 1000
#define ERROR_RESPONSE -1000
#define ERROR_NO_MORE_ADDRESSES -2000
#define ERROR_CRC_IS_NOT_VALID -3000
#define ERROR_FAILED_TO_RESET -4000

class Logger {
  public:
    int DEBUG_LVL = 0;
    int INFO_LVL = 1;
    int WARN_LVL = 2;
    int ERROR_LVL = 3;
    
    void setLevel(int value) {
      level = value;
    }
    
    void ERROR(String message) {
      if(level <= ERROR_LVL) {
        print("ERROR", message);
      }
    }

    void ERROR(String message, int code) {
      ERROR(message + " " + String(code));
    }
    
    void WARN(String message) {
      if(level <= WARN_LVL) {
        print("WARN", message);
      }
    }

    void INFO(String message) {
      if(level <= INFO_LVL) {
        print("INFO", message);
      }
    }
    
    void DEBUG(String message) {
      if(level <= DEBUG_LVL) {
        print("DEBUG", message);
      }
    }
    
    void DEBUG(String message, float value) {
      DEBUG(message + " " + String(value));
    }
    
  private:
    int level = DEBUG_LVL;
    
    void print(String message) {
      Serial.println(message);
    }
    
    void print(String level, String message) {
      Serial.println(level + ": " + message);
    }
};

class SensorThread: public Thread {
  public:
    float value;
    int pin;

    void run() {
      value = getTemp();
      runned();
    }
};

Logger Log = Logger();
//Log.setLevel(1);
int WindowSize = 100;
uint32_t PID_interval = 60000;
unsigned long windowStartTime;
double Setpoint, Input, Output;
double Kp = 70.0, Ki = 0.6, Kd = 0.6; // Initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
OneWire ds(SENSOR_PIN);  // on pin 10 (a 4.7K resistor is necessary)
SensorThread tempSensor = SensorThread();
ThreadController controller = ThreadController();

void timerCallback() {
  controller.run();
}

void setup(void) {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  tempSensor.pin = SENSOR_PIN;
  tempSensor.setInterval(10000);
  controller.add(&tempSensor);
  Timer1.initialize(10000000);
  Timer1.attachInterrupt(timerCallback);
  Timer1.start();
  windowStartTime = millis();
  Setpoint = 28.0;
  myPID.SetOutputLimits(0, WindowSize); //tell the PID to range between 0 and the full window size
  myPID.SetMode(AUTOMATIC); //turn the PID on
  Log.INFO("Setup complete");
}

int getChipType(int addr) {
  switch (addr) {
    case 0x10:
      Log.INFO("  Chip = DS18S20");  // or old DS1820
      return 1;
      break;
    case 0x28:
      Log.INFO("  Chip = DS18B20");
      return 0;
      break;
    case 0x22:
      Log.INFO("  Chip = DS1822");
      return 0;
      break;
    default:
      Log.INFO("Device is not a DS18x20 family device.");
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

float getTemp() {
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  unsigned long startTime;

  ds.reset_search();
  if ( !ds.search(addr)) {
    ds.reset_search();
    startTime = millis();
    while (millis() - startTime < 3000);
    return ERROR_NO_MORE_ADDRESSES;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    return ERROR_CRC_IS_NOT_VALID;
  }

  // the first ROM byte indicates which chip
  int type_s = getChipType(addr[0]);

  if (!ds.reset()) {
    return ERROR_FAILED_TO_RESET;
  }
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  startTime = millis();
  while (millis() - startTime < 1000); // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  if (!present) {
    Log.INFO("Failed to reset");
  }
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  return convertTemp(data, type_s);
}

void loop(void) {
  double ratio;
  float temp = tempSensor.value;
  if (temp <= ERROR_RESPONSE) {
    Log.ERROR("Code:", temp);
    return;
  }
  Input = temp;
  myPID.Compute();
  windowStartTime = millis();
  ratio = Output / WindowSize;
  unsigned int runtime = PID_interval * ratio;
  Log.DEBUG("Input:", Input); 
  Log.DEBUG("Output:", Output); 

  Log.DEBUG("HEAT ON:", runtime); 
  digitalWrite(RELAY_PIN, LOW);
  while (millis() - windowStartTime < runtime);

  Log.DEBUG("HEAT OFF:", PID_interval - runtime); 
  digitalWrite(RELAY_PIN, HIGH);
  while (millis() - windowStartTime < PID_interval - runtime);
}
