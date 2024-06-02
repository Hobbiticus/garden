#include <Arduino.h>
#include <WiFi.h>
#include <ezTime.h>

//DOIT ESP32 DEVKIT V1
//define MY_SSID, MY_WIFI_PASSWORD, RPI_ADDRESS, RPI_PORT
#include "passwd/passwd.h"
#include <ArduinoHA.h>


//#define DEBUG
//#define ENABLE_DEBUGGING
#define ENABLE_MOISTURE_SENSING

#ifdef DEBUG
const int DeepSleepTimeUS = 10 * 1000000;
const int PumpTimeMS = 10 * 1000;
#else
const int DeepSleepTimeUS = 15*60 * 1000000; //how long to go into deep sleep
const int PumpTimeMS = 3*60 * 1000;          //how long to pump for
#endif

//NOTE: with the cheap meters, higher values are dryer - need to invert them
//higher values are wetter
const int MoistureThreshold = 2000;  //TODO: set this!
const int WaterAfterTime = 8 * 60 + 0; //hours + minutes (in minutes)
const int BatteryStatusIntervalWhilePumpingMS = 30 * 1000; //how often to report battery status while the pump is operating

Timezone myTZ;

#define NODE_ID 1

#if NODE_ID == 1
#define DEVICE_NAME "Vegetable Garden"
#define DEVICE_ID "vegetable_garden"

const int NumSensors = 1;
const int VoltageCalibrationTableSize = 6;

//3 to 2 yields range 0-8.25V, battery should get to max 7.3
float R1 = 300.0;
float R2 = 200.0;
float VIN = 3.3; //TODO: calibrate this!

//node 1
//actual voltage, measured voltage
float VoltageCalibrationTable[VoltageCalibrationTableSize][2] =
{
  { 5.0, 4.72 },
  { 5.5, 5.22 },
  { 6.0, 5.73 },
  { 6.5, 6.28 },
  { 7.0, 6.90 },
  { 7.5, 7.70 }
};


#elif NODE_ID == 2
#define DEVICE_NAME "Perenial Garden"

const int NumSensors = 3;
const int VoltageCalibrationTableSize = 6;

float R1 = 300.0;
float R2 = 200.0;
float VIN = 3.3;

//node 2
//TODO: calibrate this
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

WiFiClient client;
HADevice MyHADevice;
HAMqtt hamqtt(client, MyHADevice);
HASensorNumber BatterySensor(DEVICE_ID "_battery", HABaseDeviceType::PrecisionP2);
HABinarySensor PumpStatus(DEVICE_ID "_pump_status");
HABinarySensor WaterSensor(DEVICE_ID "_water_status");
HASensor StatusSensor(DEVICE_ID "_status");
HASensor BootTimeSensor(DEVICE_ID "_boot_time");
HASensor LastPumpTime(DEVICE_ID "_last_water");
HASensorNumber MoistureSensor(DEVICE_ID "_moisture", HABaseDeviceType::PrecisionP2);


#define BATTERY_SENSE_PIN 36
const int SENSOR_PINS[5] = { 39, 34, 35, 32, 33 };
#define WATER_LEVEL_PIN 23
#define SENSOR_POWER_PIN 14
#define PUMP_POWER_PIN 13

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

bool HaveWater()
{
  return digitalRead(WATER_LEVEL_PIN) == LOW;
}

void SendBatteryStatus()
{
  float vin = ReadBatteryVoltage();
  BatterySensor.setValue(vin);
}

void SetPumpState(bool on)
{
  hamqtt.loop(); //trying to get last pump time to show up
  if (on)
  {
    digitalWrite(PUMP_POWER_PIN, HIGH);
    if (!LastPumpTime.setValue(myTZ.dateTime("Y/m/d H:i:s").c_str()))
    {
      Serial.println("Failed to set last water time, trying again...");
      hamqtt.loop();
      LastPumpTime.setValue(myTZ.dateTime("Y/m/d H:i:s").c_str());
    }
  }
  else
  {
      digitalWrite(PUMP_POWER_PIN, LOW);
  }
  //this has failed before
  while (!PumpStatus.setState(on))
  {
    Serial.println("Whoops! Failed to set pump state");
    delay(100);
    hamqtt.loop();
  }
  hamqtt.loop(); //trying to get last pump time to show up
}

void SendErrorMessage(const char* message)
{
  DebugPrint("ERROR: " + String(message) + "\n");
  StatusSensor.setValue(message);
}

void ClearError()
{
  StatusSensor.setValue("Online");
}

void TakeMoistureReadings(unsigned short* readings)
{
#ifdef ENABLE_MOISTURE_SENSING
  //turn on the power to the sensors
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  DebugPrint("Sensors are on now\n");

  //wait for the sensors to power up
  delay(3000);
  DebugPrint("Delayed!\n");

  //take readings
  for (int i = 0; i < NumSensors; i++)
  {
    readings[i] = analogRead(SENSOR_PINS[i]);
    DebugPrint("Sensor" + String(i) + " = " + String(readings[i]) + "\n");
  }

  //turn off the sensors
  digitalWrite(SENSOR_POWER_PIN, LOW);
  DebugPrint("Sensors are off\n");
#endif
}

void SendMoistureReadings(unsigned short* readings)
{
  //just take the average (if there is more than one)
  float val = 0;
  for (int i = 0; i < NumSensors; i++)
    val += readings[i];
  val /= NumSensors;
  MoistureSensor.setValue(val * 100.0f / 4095);
}

//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================


void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(SENSOR_POWER_PIN, OUTPUT);
  pinMode(PUMP_POWER_PIN, OUTPUT);
  pinMode(WATER_LEVEL_PIN, INPUT_PULLUP);
  
  digitalWrite(SENSOR_POWER_PIN, LOW);
  digitalWrite(PUMP_POWER_PIN, LOW);

  //connect to WiFi
  while (true)
  {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(MY_SSID, MY_WIFI_PASSWORD);
    Serial.println("Waiting for wifi to connect...");
    uint8_t result = WiFi.waitForConnectResult();
    Serial.printf("result = %hhu\n", result);
    if (result == WL_CONNECTED)
    {
      Serial.println("Connected to WiFi!");
      break;
    }
  }
  DebugPrint("Setup complete\n");

  byte mac[6];
  WiFi.macAddress(mac);
  MyHADevice.setUniqueId(mac, sizeof(mac));
  MyHADevice.setName(DEVICE_NAME);

  BatterySensor.setName("Voltage");
  BatterySensor.setUnitOfMeasurement("V");
  BatterySensor.setDeviceClass("voltage");
  StatusSensor.setName("Status");
  BootTimeSensor.setName("Boot Time");
  LastPumpTime.setName("Last Watered");
  PumpStatus.setName("Pump Status");
  WaterSensor.setName("Water Level");
  MoistureSensor.setName("Moisture");
  MoistureSensor.setUnitOfMeasurement("%");
  MoistureSensor.setDeviceClass("moisture");

  //apparently we have to set this BEFORE we connect to HA because it likes to publish its state when it connects
  bool haveWater = HaveWater();
  Serial.printf("Initial water state = %s\n", haveWater ? "Good" : "Low");
  WaterSensor.setCurrentState(haveWater); //we can use setCurrentState BEFORE we are connected and it will publish the correct state when we connect

  Serial.println("Starting ha integration...");
  bool result = hamqtt.begin("192.168.1.98", MQTT_USER, MQTT_PASSWORD);
  Serial.printf("begin = %c\n", result ? 'Y' : 'N');

  hamqtt.loop(); //apparently it helps to call this once BEFORE you start publishing (boot time was not publishing)

  Serial.println("Waiting for time sync...");
  int startTime = millis();
  waitForSync();
  Serial.printf("Sync took %d ms\n", (int)(millis() - startTime));
  myTZ.setLocation("America/New_York");

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  DebugPrint("Wakeup cause = " + String(wakeup_reason) + "\n");

  if (wakeup_reason < 1 || wakeup_reason > 5)
  {
    //this is a cold start (not waking from deep sleep)
    DebugPrint("Boot time = " + myTZ.dateTime("Y/m/d H:i:s") + "\n");
    BootTimeSensor.setValue(myTZ.dateTime("Y/m/d H:i:s").c_str());
  }
  DebugPrint("Setup actually complete\n");
  DebugPrint("Boot time = " + String(bootTime) + "; now = " + String(GetTimeMS()) + "\n");

  //update our status
  SendBatteryStatus();
  bool haveError = false;
  WaterSensor.setState(haveWater);
  if (!haveWater)
  {
    SendErrorMessage("Water is low");
    haveError = true;
  }
  PumpStatus.setState(false);

  if (!haveError)
  {
    ClearError();
  }
}

#ifdef ENABLE_MOISTURE_SENSING
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
    SendErrorMessage("All sensors are bad");
    return 0x7FFFFFFF;
  }

  if (badSensor != -1)
  {
    String message = "Sensor " + String(badSensor) + " is malfunctioning";
    SendErrorMessage(message.c_str());
  }

  int averageValue = sum / validReadings;
  DebugPrint("Average value = " + String(averageValue) + "\n");
  return averageValue;
}
#endif

void DoTheThings()
{
#ifdef ENABLE_MOISTURE_SENSING
  //take moisture readings
  unsigned short readings[NumSensors] = {0};
  TakeMoistureReadings(readings);
  SendMoistureReadings(readings);

  int averageValue = ComputeAverageMoisture(readings);
  if (averageValue >= MoistureThreshold)
  {
    DebugPrint("Garden bed is not dry enough, going to sleep\n");
    return;
  }
#endif

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
  if (!HaveWater())
  {
    //not enough water!
    DebugPrint("Want to water, but the water is empty\n");
    SendErrorMessage("Water is low");
    WaterSensor.setState(false);
    return;
  }

  //turn on the pump for a bit!
  //send message to hub saying pump got turned on
  DebugPrint("Turning on pump\n");
  SetPumpState(true);
  LastWateredDay = myTZ.dayOfYear();
  unsigned long waterStartTime = millis();
  unsigned long lastBatteryStatusTime = millis();
  while (millis() - waterStartTime < PumpTimeMS)
  {
    //check to see if there is still water left
    if (digitalRead(WATER_LEVEL_PIN) == HIGH)
    {
      //out of water!!
      SendErrorMessage("Water is low");
      break;
    }
    //periodically send the battery status so we can see it draining while pumping
    if (millis() - lastBatteryStatusTime > BatteryStatusIntervalWhilePumpingMS)
    {
      SendBatteryStatus();
      lastBatteryStatusTime += BatteryStatusIntervalWhilePumpingMS;
    }
    delay(1000);
  }
  DebugPrint("Turning off pump\n");
  SetPumpState(false);

  //update the moisture after the pumping is done
#ifdef ENABLE_MOISTURE_SENSING
  TakeMoistureReadings(readings);
  SendMoistureReadings(readings);
#endif
  //send the battery status too since we probably drained it pretty good
  SendBatteryStatus();
}

void loop()
{
  DebugPrint("Entered loop\n");
  events(); //ezTime events()

  hamqtt.loop();

//-----
  // SetPumpState(true);
  // delay(1000);
  // SetPumpState(false);
  // delay(1000);
  // return;
//-----

  DoTheThings();
  hamqtt.loop();

  DebugPrint("going to sleep\n");
  delay(50);
  //update our time tracker
  CurrentTimeMS += millis() - bootTime + DeepSleepTimeUS / 1000;
  //go to sleep
  esp_sleep_enable_timer_wakeup(DeepSleepTimeUS);
  esp_deep_sleep_start();
}
