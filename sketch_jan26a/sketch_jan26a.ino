//updated on 15 Nov 2021 
//v 3.1
#include "Arduino.h"
#include <DS18B20.h>
#include "PIL.h"
#include <Adafruit_AHT10.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "Zanshin_BME680.h"
#define HASBME

#define DAC0_PIN      DAC0 
#define DAC1_PIN      DAC1
byte PELT_CMD_PIN =   DAC1;
#define PELT_SHDN     10 //peltier shut down pin
#define AI0_PIN       A10
#define AI1_PIN       A11
#define DI1 44        //I2C
#define DI2 46        //AHT10
#define DI3 48        //BME
#define DI4 50        //unused
#define PID_PERIOD            10000
int COMMUNICATION_PERIOD  = 3;

enum { // Temperature & humidity sensor options
  DS18,
  AHT10,
  BME,
  NOSENSOR
};

enum{ // Heater/cooler options
  PELTIER,
  RESISTOR
};

enum{ //MFC control options
  BUILTIN_ADC_DAC,
  I2C
};

enum STATE
{
  IDLE_STATE,
  WAITING_COMMAND,
  WAITING_CHANNEL,
  READY_TO_EXECUTE
};

enum COMMAND
{
  NONE,
  SET_VOLTAGE,
  SET_DUTYRATE,
  SET_TEMP,
  OPEN_VALVE,
  CLOSE_VALVE,
};

BME680_Class BME680;
DS18B20       ds(2);
Adafruit_AHT10 aht;
Adafruit_ADS1115 ads;
Adafruit_MCP4725 dac1, dac2;
// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit =     0.1875mV (default)
//Getting single-ended readings from AIN0..3
//ADC Range: +/- 6.144V (1 bit =  0.1875mV)

byte TempControlOption = DS18;
byte HeaterOption = RESISTOR;
byte MFCControlOption = BUILTIN_ADC_DAC;
COMMAND       command             = NONE;
STATE         comState            = IDLE_STATE;
long          dac_values[2]       = {0, 0};
byte          ADC_channels[2]     = {AI0_PIN,  AI1_PIN};
byte          DAC_channels[2]     = {DAC0_PIN, DAC1_PIN};
//Arduino pin numbers are different from SAM Pins!!! //OUTPUTS
uint32_t      DIO_channels[6]     = {24, 31, 23, 43, 42, 48};
long          pelt_cmd            = 0;
long          ticks               = 0;
long          com_ticks           = 0;
sensors_event_t humidity, temp;         //used with AHT10
int           temperature         = 0;  //used with DS18B20 temp sensor and for pid loop
int           set_temperature     = 0;
double        a                   = 8;   //6;  //3.5;   //3
double        b                   = -12; //-15; //-18.0; //18
double        c                   = 35;  //30;  //30
double        d                   = 0.05; //0.1;
double        previous_temperature = -12345.343;
double        delta_integral      = 0;
bool          pid_active          = false;
byte          active_ch           = 0;
long          dio                 = 0;
byte          package_number      = 0;
bool          ledState            = false;
bool          useNativeUSB        = true;
bool          debug = true && useNativeUSB; //if true, debug output to serial console enabled
                                            //debug can be true only if the main stream goes to NativeUSB
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  analogWrite(PELT_CMD_PIN, 0);
  while (!Serial)
  {
    ;
  }
  SerialUSB.begin(9600); //argument value has no sense, the datarate used is the maximum datarate supported by two devices
  while(!SerialUSB)
  {
    ;
  }
  if (debug) Serial.print("Program started\n");
  if (debug) Serial.print("NativeUSB initialized\n");
  for (int i = 0; i < 6; i++)
  {
    pinMode(DIO_channels[i], OUTPUT);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PELT_SHDN, OUTPUT);
  pinMode(DI1, INPUT_PULLUP);
  pinMode(DI2, INPUT_PULLUP);
  pinMode(DI3, INPUT_PULLUP);
  pinMode(DI4, INPUT_PULLUP);
  //ioport_set_port_mode()//enable pullups on inputs here
  if (digitalRead(DI1) == LOW)
    MFCControlOption = I2C;
  if ((digitalRead(DI3) == LOW) && (digitalRead(DI2)==HIGH))
    TempControlOption = AHT10;
  else if ((digitalRead(DI3) == LOW) && (digitalRead(DI2)==LOW)){
    TempControlOption = BME;
    COMMUNICATION_PERIOD = 10;
  }
  else if ((digitalRead(DI3) == HIGH) && (digitalRead(DI2) == HIGH)){
    TempControlOption = NOSENSOR;
    COMMUNICATION_PERIOD = 10;
  }
  if (digitalRead(DI4) == LOW){
    HeaterOption = PELTIER;  
    //PELT_CMD_PIN=DAC0;
    digitalWrite(PELT_SHDN, LOW); //Peltier is disabled
    if (debug) Serial.print("Peltier enabled\n");
  }
  if (MFCControlOption==I2C)
  {
    if (debug) Serial.print("I2C enabled\n");
    ads.begin();
    // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
    // For MCP4725A0 the address is 0x60 or 0x61
    // For MCP4725A2 the address is 0x64 or 0x65
    dac1.begin(0x60);
    dac2.begin(0x61);
  }
  
  if (TempControlOption==AHT10)
  {
    if (debug) Serial.print("AHT enabled\n");
    while (!aht.begin())
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }
  else if(TempControlOption == DS18)
  {
    if (debug) Serial.print("DS18 enabled\n");
    ds.selectNext();
    previous_temperature = ds.getTempC() * 10;
  }
  #ifdef HASBME
  else if(TempControlOption==BME)
  {
    if (debug) Serial.print("BME enabled\n");
    while(!BME680.begin(I2C_STANDARD_MODE)){
      if (debug)Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
      delay(5000);
    }
     BME680.setOversampling(TemperatureSensor, Oversample1);  // Use enumerated type values
     BME680.setOversampling(HumiditySensor, Oversample1);     // Use enumerated type values
     BME680.setOversampling(PressureSensor, SensorOff);     // Use enumerated type values
     if (debug)Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
     BME680.setIIRFilter(IIROff);  // Use enumerated type values
     if (debug)Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbols
     //BME680.setGas(320, 150);  // 320�c for 150 milliseconds
  }
  #endif
  
}

void reply()
{
  //Reply package:
  //[NUMBER][DAC0][DAC1][ADC0][ADC1][PORTA][DUTY][T][H]
  char tmp[128];
  package_number++;
  long porta = readDIO();
  char temp_humidity_string[16];

  if (TempControlOption==DS18)
  { //DS1620 sensor (default option)
    temperature = ds.getTempC() * 10;
    sprintf(temp_humidity_string, "%04X", temperature);
  }
  else if (TempControlOption==AHT10)
  { //break out board with AHT10
    digitalWrite(LED_BUILTIN, HIGH);
    aht.getEvent(&humidity, &temp);
    sprintf(temp_humidity_string, "%04X %04X", int(temp.temperature) * 10, int(humidity.relative_humidity) * 10);
    //delay(200);
    digitalWrite(LED_BUILTIN, LOW);
  }
  else if (TempControlOption == NOSENSOR)
  {
    int32_t bme_temp = 0;
    int32_t bme_humidity = 0;
    sprintf(temp_humidity_string, "%04X %08X", bme_temp, bme_humidity);
  }
  else if (TempControlOption == BME)
  {//new chamber version with internal BME680 
     int32_t  bme_temp, bme_humidity, bme_pressure, bme_gas;  // BME readings
     char     buf[16];                        // sprintf text buffer
     static uint16_t loopCounter = 0;         // Display iterations
    //BME680.triggerMeasurement();
    digitalWrite(LED_BUILTIN, HIGH);
    #ifdef HASBME
    BME680.getSensorData(bme_temp, bme_humidity, bme_pressure, bme_gas); //get readings
    if (loopCounter++ != 0) { 
      sprintf(buf, "%4d t=%d, ", (loopCounter - 1) % 9999,  // Clamp to 9999,
            bme_temp);   // Temp in decidegrees
      if (debug) Serial.print(buf);
      sprintf(buf, "h=%d\n", bme_humidity);  // Humidity milli-pct
      if (debug) Serial.print(buf);
      sprintf(temp_humidity_string, "%04X %08X", bme_temp, bme_humidity);
      temperature = bme_temp/10; 
    }
    #else
      bme_temp = 0;
      bme_humidity = 0;
      sprintf(temp_humidity_string, "%04X %08X", bme_temp, bme_humidity);
    #endif
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  
  if(pid_active)
  {
      double delta = set_temperature - temperature;
      delta_integral += delta;
      int action = 0;
      action = delta*a + c + b*(temperature-previous_temperature) +d*delta_integral;
          
      if (action>130) action = 130;
      if (action<0) action = 0;
      
      analogWrite(PELT_CMD_PIN, action);
      previous_temperature = temperature;
  }

  com_ticks++;
  if(com_ticks<COMMUNICATION_PERIOD)
    return;
  else
    com_ticks = 0;

  if (MFCControlOption==I2C) //external DAC ADC boards
  {
    int16_t adc0, adc1;
    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    if (adc0<0)
      adc0 = 0;
    if (adc1<0)
      adc1 = 0;
    sprintf(tmp, "%02X %04X %04X %04X %04X %08X %02X %s", package_number, dac_values[0], dac_values[1],
            adc0, adc1, porta, pelt_cmd, temp_humidity_string);
  } else //build-in Arduino Analog pins
  {
    sprintf(tmp, "%02X %02X %02X %04X %04X %08X %02X %s", package_number, dac_values[0], dac_values[1],
            analogRead(ADC_channels[0]), analogRead(ADC_channels[1]), porta, pelt_cmd, temp_humidity_string);
  } 
  //if(!debug) Serial.println(tmp);
  if(useNativeUSB) 
    SerialUSB.println(tmp);
  else
    Serial.println(tmp);
}

void loop() {

  if (ticks >= PID_PERIOD)
  {
    reply();
    ticks = 0;
  }
  ticks++;
  int bytesAvailable = 0;
  if (useNativeUSB)
  {
    bytesAvailable = SerialUSB.available();
  }
  else
  {
    bytesAvailable = Serial.available();
  }
  if (bytesAvailable > 0)
  {
    byte b = 0;
    if (useNativeUSB)
    {
      b = SerialUSB.read();
    }
    else
    {
      b = Serial.read();
    }
    if (b == '$') //$ starts the package
    {
      comState = WAITING_COMMAND;
      digitalWrite(LED_BUILTIN, HIGH);
      return;
    }
    int pelt_cmd = 0;
    int set_temperature = 0;
    if (comState == WAITING_COMMAND)
    {
      switch (b)
      {
        case 'b':   //next character is the channel number
          active_ch = 0;
          command   = SET_VOLTAGE;
          comState  = WAITING_CHANNEL;
          break;

        case 'c':   //next few digits are duty rate to set on PWM
          command   = SET_DUTYRATE;
          
          if (useNativeUSB)
          {
            pelt_cmd  = SerialUSB.parseInt(SKIP_ALL);
          }
          else
          {
            pelt_cmd  = Serial.parseInt(SKIP_ALL);
          }
          comState  = READY_TO_EXECUTE;
          break;

        case 'f':   //next digit is DIO the channel number
          active_ch = 0;
          command   = OPEN_VALVE;
          comState  = WAITING_CHANNEL;
          break;

        case 'g':   //next digit is DIO the channel number
          active_ch = 0;
          command   = CLOSE_VALVE;
          comState  = WAITING_CHANNEL;
          break;

        case 't':   //next few digits is integer representing temperature of perltier
          command     = SET_TEMP;
          
          if (useNativeUSB)
          {
            set_temperature = SerialUSB.parseInt(SKIP_ALL);//*10;
          }
          else{
            set_temperature = Serial.parseInt(SKIP_ALL);//*10;
          }
          comState    = READY_TO_EXECUTE;
          /*if(set_temperature!=0)
          {
            pid_active = true;
            digitalWrite(PELT_SHDN, HIGH);
          }
          else
          {
            pid_active = false;
            digitalWrite(PELT_SHDN , LOW);
          }*/
          digitalWrite(PELT_SHDN, HIGH);
          analogWrite(PELT_CMD_PIN, set_temperature);
          break;  
        
        default:
          comState = IDLE_STATE;
          command  = NONE;
          break;

          return;
      }
    } else if (comState == WAITING_CHANNEL)
    {
      if ((b < 48) || (b > 57)) //only '0' to '9' are accepted
      {
        command   = NONE;
        comState  = IDLE_STATE;
        //Serial.println("Invalid channel number");
        return;
      }
      active_ch = b - 48;
      comState  = READY_TO_EXECUTE;
      if (command == SET_VOLTAGE)
      {
        if(useNativeUSB)
          dac_values[active_ch] = SerialUSB.parseInt(SKIP_ALL);
        else
          dac_values[active_ch] = Serial.parseInt(SKIP_ALL);
      }
      return;
    }

    if (b == '#') //# ends the package and executes the command
    {
      if (comState != READY_TO_EXECUTE)
      {
        //Serial.println("Wrong command");
        comState  = IDLE_STATE;
        command   = NONE;
        return;
      }

      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);

      if (command == SET_VOLTAGE)
      {
        //sprintf(tmp, "Setting voltage: %X for channel %d", voltage, active_ch);
        if (I2C) {
          if (active_ch == 0) {
            dac1.setVoltage(dac_values[active_ch], false);
          }
          else
          {
            dac2.setVoltage(dac_values[active_ch], false);
          }
        } 
        /*else {  
          analogWrite(DAC_channels[active_ch], dac_values[active_ch]);
        }*/
        command   = NONE;
        comState  = IDLE_STATE;
        return;
      }

      if (command == SET_DUTYRATE)
      {
        //sprintf(tmp, "Setting duty rate: %X", duty);
        analogWrite(PELT_CMD_PIN, pelt_cmd);
        command   = NONE;
        comState  = IDLE_STATE;
        return;
      }

      if (command == SET_TEMP )
      {
        //Serial.println ("Not implemented");
        comState  = IDLE_STATE;
        command   = NONE;
        return;
      }

      if (command == OPEN_VALVE )
      {
        //sprintf(tmp, "Valve %d open", active_ch );
        digitalWrite(DIO_channels[active_ch], HIGH);
        comState  = IDLE_STATE;
        command   = NONE;
        return;
      }

      if (command == CLOSE_VALVE )
      {
        //sprintf(tmp, "Valve %d closed", active_ch );
        digitalWrite(DIO_channels[active_ch], LOW);
        comState  = IDLE_STATE;
        command   = NONE;
        return;
      }
    }
  }

}
