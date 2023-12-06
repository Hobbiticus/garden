#include <Arduino.h>
#include <WiFi.h>
#include <ezTime.h>

//DOIT ESP32 DEVKIT V1
//define MY_SSID, MY_WIFI_PASSWORD, RPI_ADDRESS, RPI_PORT
#include "passwd/passwd.h"
#include "Protocol/Protocol.h"
#include "MQTTDevice.h"
#include "MQTTSensor.h"
#include "MQTT.h"
#include <ArduinoHA.h>


WiFiClient client;
HADevice device;
HAMqtt hamqtt(client, device);
HASensorNumber SensorBattery("battery", HABaseDeviceType::PrecisionP2);

WiFiClient mqttWifi;
MQTTClient mqtt(256);

//#define DEBUG
#define ENABLE_DEBUGGING

#ifdef DEBUG
const int DeepSleepTimeUS = 10 * 1000000;
const int PumpTimeMS = 10 * 1000;
#else
const int DeepSleepTimeUS = 15*60 * 1000000;
const int PumpTimeMS = 5*60 * 1000;
#endif

//higher MEASURED values are dryer, but we are inverting the readings so that higher REPORTED values are wetter
//higher values are wetter
const int MoistureThreshold = 2000;
const int WaterAfterTime = 16 * 60 + 0; //hours + minutes (in minutes)


//3 to 2 yields range 0-8.25V, battery should get to max 7.3
#define NODE_ID 1

#if NODE_ID == 1
MQTTDevice GardenDevice(mqtt, "Test Garden", "test_garden");

const int NumSensors = 3;
const int VoltageCalibrationTableSize = 6;

//TODO: enter correct R values here - this is from node2
float R1 = 255.8;//300.0;
float R2 = 170.5;//200.0;
float VIN = 3.3; //TODO: calibrate this!

//TODO: this has not been calibrated - this is from node 2
//node 1
//actual voltage, measured voltage
float VoltageCalibrationTable[VoltageCalibrationTableSize][2] =
{
  { 5.0, 4.81 },
  { 5.5, 5.32 },
  { 6.0, 5.84 },
  { 6.5, 6.38 },
  { 7.0, 6.97 },
  { 7.5, 7.72 }
};


#elif NODE_ID == 2
MQTTDevice GardenDevice(mqtt, "Perenial Garden", "perenial_garden");

const int NumSensors = 3;
const int VoltageCalibrationTableSize = 6;

float R1 = 255.8;//300.0;
float R2 = 170.5;//200.0;
float VIN = 3.3; //TODO: calibrate this!

//node 2
//actual voltage, measured voltage
float VoltageCalibrationTable[VoltageCalibrationTableSize][2] =
{
  { 5.0, 4.81 },
  { 5.5, 5.32 },
  { 6.0, 5.84 },
  { 6.5, 6.38 },
  { 7.0, 6.97 },
  { 7.5, 7.72 }
};
#endif

MQTTSensor BatterySensor(GardenDevice, "Battery", "battery");


#define BATTERY_SENSE_PIN 36
const int SENSOR_PINS[5] = { 39, 34, 35, 32, 33 };
#define WATER_LEVEL_PIN 23
#define SENSOR_POWER_PIN 14
#define PUMP_POWER_PIN 13

//errors
#define ERROR_NO_SENSORS 1
#define ERROR_BAD_SENSOR 2
#define ERROR_OUT_OF_WATER 3

RTC_DATA_ATTR unsigned int CurrentTimeMS = 0;
RTC_DATA_ATTR int LastWateredDay = -1;

unsigned int bootTime = millis();
unsigned int GetTimeMS()
{
  return CurrentTimeMS + (millis() - bootTime);
}

#ifdef ENABLE_DEBUGGING

//socket debugging
#define DEBUG_ADDRESS "192.168.1.55"
#define DEBUG_PORT 4565
WiFiClient DebugSocket;
#endif

void DebugPrint(String str)
{
  Serial.print(str);
#ifdef ENABLE_DEBUGGING
  //reconnect if we're not connected
  if (!DebugSocket.connected())
  {
    if (!DebugSocket.connect(DEBUG_ADDRESS, DEBUG_PORT))
      return; //failed to reconnect - give up
    DebugSocket.setNoDelay(true);
  }

  DebugSocket.print(str);
  DebugSocket.flush();
#endif
}

float ReadBatteryVoltage()
{
  //sometimes this reads as 0 when there is definitely voltage here
  const int NumReadings = 5;
  //String out;

  int voltages[NumReadings] = {0};
  
  int readingCount = 0;
  const int MaxReadings = NumReadings * 20;
  
  for (int i = 0; i < NumReadings && readingCount < MaxReadings; readingCount++)
  {
    int voltage = analogRead(BATTERY_SENSE_PIN);
    //DebugPrint("Reading = " + String(voltage) + "\n");
    if (voltage != 0)
    {
      //out += " " + String(voltage);
      voltages[i] = voltage;
      i++;
    }
    if (i < NumReadings - 1)
      //really need to wait a while for this to be accurate for some reason
      delay(100);
  }
  //DebugPrint(out + "\n");

  //sort the voltages (go go gadget bubble sort!)
  bool sorted = false;
  while (!sorted)
  {
    sorted = true;
    for (int i = 0; i < NumReadings-1; i++)
    {
      if (voltages[i] > voltages[i+1])
      {
        sorted = false;
        int temp = voltages[i];
        voltages[i] = voltages[i+1];
        voltages[i+1] = temp;
        break;
      }
    }
  }

  //drop the top and bottom values and average
  float rawVoltage = 0;
  for (int i = 1; i < NumReadings-1; i++)
    rawVoltage += voltages[i];
  rawVoltage /= NumReadings - 2;
  
  Serial.print("Raw reading = ");
  Serial.println(rawVoltage);

  float vout = (rawVoltage * VIN) / 4096.0;
  float vin = vout / (R2 / (R1 + R2));
  DebugPrint("Meastured voltage = " + String(vin, 2) + String(" V\n"));

  //Serial.print("Measured voltage = ");
  //Serial.print(vin, 2);
  //Serial.println(" V");

  //use the calibration table to get a more accurate voltage reading
  if (vin < VoltageCalibrationTable[0][1])
  {
    //extrapolate down
    float ratio = VoltageCalibrationTable[0][0] / VoltageCalibrationTable[0][1];
    vin *= ratio;
  }
  else if (vin > VoltageCalibrationTable[VoltageCalibrationTableSize-1][1])
  {
    //extrapolate up
    float ratio = VoltageCalibrationTable[VoltageCalibrationTableSize-1][0] / VoltageCalibrationTable[VoltageCalibrationTableSize-1][1];
    vin *= ratio;
  }
  else
  {
    //interpolate!
    int i = 0;
    //find where to interpolate
    for (; i < VoltageCalibrationTableSize - 1; i++)
    {
      if (VoltageCalibrationTable[i+1][1] > vin)
        break;
    }
    DebugPrint("Interpolating between " + String(VoltageCalibrationTable[i][1], 2) + " and " + String(VoltageCalibrationTable[i+1][1], 2) + "\n");
    float ratio = (vin - VoltageCalibrationTable[i][1]) / (VoltageCalibrationTable[i+1][1] - VoltageCalibrationTable[i][1]);
    vin = (VoltageCalibrationTable[i+1][0] * ratio) + (VoltageCalibrationTable[i][0] * (1 - ratio));
  }
  DebugPrint("Actual voltage = " + String(vin, 2) + String(" V\n"));
  //Serial.print("Actual voltage = ");
  //Serial.print(vin, 2);
  //Serial.println(" V");

  return vin;
}

void SendStartup()
{
  //TODO: implement this
}

void SendBatteryStatus()
{
  float vin = ReadBatteryVoltage();

  BatterySensor.PublishValue(String(vin, 2));
  SensorBattery.setValue(vin);
}

void SendPumpEvent(unsigned char on)
{
  //TODO: implement this
}

void SendErrorMessage(int error, const char* message)
{
  //TODO: implement this
  DebugPrint("ERROR: " + String(message) + "\n");

  // ErrorMessage msg;
  // msg.m_ID = NODE_ID;
  // strcpy(msg.m_Message, message);
  // unsigned char buffer[300];
  // unsigned int bufferLength = msg.Serialize(buffer, sizeof(buffer));

  // if (!SocketSend(buffer, bufferLength))
  //   return;
}

void ClearError(int error)
{
  //TODO: implement this (maybe not)
}

void TakeMoistureReadings(unsigned short* readings)
{
  //turn on the power to the sensors
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  DebugPrint("Sensors are on now\n");

  //wait for the sensors to power up
  delay(1000);
  DebugPrint("Delayed!\n");

  //take readings
  for (int i = 0; i < NumSensors; i++)
  {
    readings[i] = 4095 - analogRead(SENSOR_PINS[i]);
    DebugPrint("Sensor" + String(i) + " = " + String(readings[i]) + "\n");
  }

  //turn off the sensors
  digitalWrite(SENSOR_POWER_PIN, LOW);
  DebugPrint("Sensors are off\n");
}

void SendMoistureReadings(unsigned short* readings)
{
  //TODO: implement this
}

//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================
Timezone myTZ;


void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(SENSOR_POWER_PIN, OUTPUT);
  pinMode(PUMP_POWER_PIN, OUTPUT);
  pinMode(WATER_LEVEL_PIN, INPUT_PULLUP);
  
  digitalWrite(SENSOR_POWER_PIN, LOW);
  digitalWrite(PUMP_POWER_PIN, LOW);

  //connect to WiFi
  WiFi.begin(MY_SSID, MY_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.println("Waiting for wifi to connect...");
  }
  Serial.println("Connected to WiFi!");
  DebugPrint("Setup complete\n");

  byte mac[6];
  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));
  device.setName("HATest Garden");
  //device.setSoftwareVersion("1.0.0");

  SensorBattery.setUnitOfMeasurement("V");
  SensorBattery.setDeviceClass("voltage");
  BatterySensor.Init("voltage", "V");

  waitForSync();
  myTZ.setLocation("America/New_York");

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  DebugPrint("Wakeup cause = " + String(wakeup_reason) + "\n");

  if (wakeup_reason < 1 || wakeup_reason > 5)
    SendStartup();
  SendBatteryStatus();
  DebugPrint("Setup actually complete\n");
  DebugPrint("Boot time = " + String(bootTime) + "; now = " + String(GetTimeMS()) + "\n");
}

int ComputeAverageMoisture(unsigned short* readings)
{
  int sum = 0;
  int validReadings = 0;

  int badSensor = -1;
  for (int i = 0; i < NumSensors; i++)
  {
    if (readings[i] < 3595)
    {
      validReadings++;
      sum += readings[i];
    }
    else
    {
      DebugPrint("Sensor" + String(i) + " is invalid!\n");
      badSensor = i;
    }
  }
  
#ifdef DEBUG
  //ignore bad inputs
  sum = MoistureThreshold - 1;
  validReadings = 1;
#endif

  if (validReadings == 0)
  {
    DebugPrint("All sensors are bad!\n");
    SendErrorMessage(ERROR_NO_SENSORS, "All sensors are bad");
    return 0x7FFFFFFF;
  }

  //at least 1 sensor is working, so reset the ERROR_NO_SENSORS last error time
  ClearError(ERROR_NO_SENSORS);
  
  if (badSensor != -1)
  {
    char messageBuffer[256] = {0};
    String message = "Sensor " + String(badSensor) + " is malfunctioning";
    message.toCharArray(messageBuffer, sizeof(messageBuffer));
    SendErrorMessage(ERROR_BAD_SENSOR, messageBuffer);
  }
  else
  {
    //all of our sensors are working - clear ERROR_BAD_SENSOR error
    ClearError(ERROR_BAD_SENSOR);
  }

  int averageValue = sum / validReadings;
  DebugPrint("Average value = " + String(averageValue) + "\n");
  return averageValue;
}

void DoTheThings()
{
  //take moisture readings
  unsigned short readings[MoistureReadings::MaxReadings] = {0};
  TakeMoistureReadings(readings);
  SendMoistureReadings(readings);

  int averageValue = ComputeAverageMoisture(readings);
  if (averageValue >= MoistureThreshold)
  {
    DebugPrint("Garden bed is not dry enough, going to sleep\n");
    return;
  }

  int hour = myTZ.hour();
  int minute = myTZ.minute();

  //decide what to do...
  int timeOfDay = hour * 60 + minute;
  if (timeOfDay < WaterAfterTime)
  {
    DebugPrint("It is not late enough in the day, going to sleep\n");
    return;
  }

  if (LastWateredDay == myTZ.dayOfYear())
  {
    DebugPrint("Already watered today, going to sleep\n");
    return;
  }

  //make sure there is enough water to pump
  if (digitalRead(WATER_LEVEL_PIN) == HIGH)
  {
    //not enough water!
    DebugPrint("Want to water, but the water is empty\n");
    SendErrorMessage(ERROR_OUT_OF_WATER, "Out of water");
    return;
  }

  //we have enough water, so clear the ERROR_OUT_OF_WATER error
  ClearError(ERROR_OUT_OF_WATER);
  
  //turn on the pump for a bit!
  //send message to hub saying pump got turned on
  SendPumpEvent(1);
  DebugPrint("Turning on pump\n");
  LastWateredDay = myTZ.dayOfYear();
  digitalWrite(PUMP_POWER_PIN, HIGH);
  for (int i = 0; i * 1000 < PumpTimeMS; i++)
  {
    //check to see if there is still water left
    if (digitalRead(WATER_LEVEL_PIN) == HIGH)
    {
      //out of water!!
      SendErrorMessage(ERROR_OUT_OF_WATER, "Out of water while pumping");
      break;
    }
    delay(1000);
  }
  DebugPrint("Turning off pump\n");
  digitalWrite(PUMP_POWER_PIN, LOW);
  //send message to hub saying pump got turned off
  SendPumpEvent(0);

  //update the moisture after the pumping is done
  TakeMoistureReadings(readings);
  SendMoistureReadings(readings);
  //send the battery status too since we probably drained it pretty good
  SendBatteryStatus();
}

void loop()
{
  DebugPrint("Entered loop\n");
  events(); //ezTime events()
  mqtt.loop();
  hamqtt.loop();

  DoTheThings();

  mqtt.loop();
  DebugPrint("going to sleep\n");
  delay(50);
  //update our time tracker
  CurrentTimeMS += millis() - bootTime + DeepSleepTimeUS / 1000;
  //go to sleep
  mqtt.disconnect();
  esp_sleep_enable_timer_wakeup(DeepSleepTimeUS);
  esp_deep_sleep_start();
}
