#include <OneWire.h>
#include <PID_v1.h>
#include <Thread.h>
#include <ThreadController.h>
#include <TimerOne.h>


// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library


/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  the pid is designed to Output an analog value,
 * but the relay can only be On/Off.
 *
 *   to connect them together we use "time proportioning
 * control"  it's essentially a really slow version of PWM.
 * first we decide on a window size (5000mS say.) we then
 * set the pid to adjust its output between 0 and that window
 * size.  lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the
 * window being "Relay Off Time"
 ********************************************************/
 
 /* 
  Connect 5V on Arduino to VCC on Relay Module
  Connect GND on Arduino to GND on Relay Module 
  Connect GND on Arduino to the Common Terminal (middle terminal) on Relay Module. 
  */
  
#define RELAY_PIN 8   // Connect Digital Pin 8 on Arduino to ? on Relay Module
#define SENSOR_PIN 10
#define DELAY 1000

class SensorThread: public Thread {
  public:
    float value;
    int pin;

  void run(){
    value = getTemp();
    runned();
  }
};

OneWire ds(SENSOR_PIN);  // on pin 10 (a 4.7K resistor is necessary)

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=70.0, Ki=0.6, Kd=0.6;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 100; 
uint32_t PID_interval = 30000;
unsigned long windowStartTime;

SensorThread tempSensor = SensorThread();

// Instantiate a new ThreadController
ThreadController controller = ThreadController();

// This is the callback for the Timer
void timerCallback(){
  controller.run();
}

void setup(void) {  
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);

   // Configures Thread
  tempSensor.pin = SENSOR_PIN;
  tempSensor.setInterval(10000);

  // Add the Threads to our ThreadController
  controller.add(&tempSensor);

  Timer1.initialize(10000000);
  Timer1.attachInterrupt(timerCallback);
  Timer1.start();
  
  windowStartTime = millis();
  Setpoint = 28.0;
  myPID.SetOutputLimits(0, WindowSize); //tell the PID to range between 0 and the full window size
  myPID.SetMode(AUTOMATIC); //turn the PID on
}

int getChipType(int addr) {
  switch (addr) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      return 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      return 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      return 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
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
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
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
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    startTime = millis();
    while(millis() - startTime < 3000);
    return -1000;
  }

  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  // the first ROM byte indicates which chip
  int type_s = getChipType(addr[0]);
  

  if(!ds.reset()) {
    Serial.println("Failed to reset");
    return -1000;
  }
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  startTime = millis();
  while(millis() - startTime < 1000);
  // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  if(!present) {
    Serial.println("Failed to reset");
  }
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  return convertTemp(data, type_s);
}

void loop(void) {
  double ratio;
  float temp = tempSensor.value;
  if(temp == -1000){
    return;
  }
  Input = temp;
  myPID.Compute();
  windowStartTime = millis();
  Serial.println();
  Serial.print("Input: ");
  Serial.println(Input);
  Serial.print("Output: ");
  Serial.println(Output);

  ratio = Output / WindowSize;
  Serial.print("Ratio: ");
  Serial.println(ratio);
  Serial.print("Runtime: ");
  Serial.println(PID_interval * ratio);

  Serial.println("LOW");
  digitalWrite(RELAY_PIN, LOW);
  while(millis() - windowStartTime < PID_interval * ratio);
  
  Serial.println("HIGH");
  digitalWrite(RELAY_PIN, HIGH);
  while(millis() - windowStartTime < PID_interval);
}
