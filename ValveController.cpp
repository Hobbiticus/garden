#include <Arduino.h>
#include "WIFI_Credentials.h"


#define BOOST_PIN 25
#define BUCK_PIN 26
#define OPEN_PIN 27
#define PUMP_ENABLE_PIN 12
#define MOISTURE_SENSOR_DATA_PIN 35
#define MOISTURE_SENSOR_SWITCH_PIN 32
#define FLOAT_PIN 17
//eventually...
//#define LED_PIN 15
//#define DEBUG_PIN 13

const static unsigned long TransitionTimeMS = 50; // how long to stay in opening/closing states

//For testing...
#define DO_TEST
#ifdef DO_TEST
#define LED_PIN 2
#define STATE_SWITCH_PIN 23
int CurrState = 0; // 0 = idle, 1 = charge, 2 = open, 3 = running, 4 = close
unsigned long LastBlinkTime = 0;
unsigned long LastStateChangeTime = 0;
enum State
{
  STATE_IDLE = 0,
  STATE_CHARGE,
  STATE_OPEN,
  STATE_AFTER_OPEN,
  STATE_RUNNING,
  STATE_CLOSE,
  STATE_ROLLOVER,
};
#endif

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello!");

  pinMode(BUCK_PIN, OUTPUT);
  pinMode(BOOST_PIN, OUTPUT);
  pinMode(OPEN_PIN, OUTPUT);

#ifdef DO_TEST
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(STATE_SWITCH_PIN, INPUT_PULLUP); //for testing
  LastBlinkTime = millis();
  LastStateChangeTime = millis();
#endif

  //TODO: Setup WiFi

}

void OnChangeState()
{
  digitalWrite(BOOST_PIN, CurrState == STATE_CHARGE || CurrState == STATE_OPEN || CurrState == STATE_RUNNING ? HIGH : LOW);
  digitalWrite(BUCK_PIN, CurrState == STATE_OPEN || CurrState == STATE_CLOSE ? HIGH : LOW);
  digitalWrite(OPEN_PIN, CurrState == STATE_OPEN ? HIGH : LOW);
}

void loop()
{  
#ifdef DO_TEST
  unsigned long now = millis();
  if (digitalRead(STATE_SWITCH_PIN) == LOW)
  {
    if (now - LastStateChangeTime > 1000)
    {
      CurrState += 1;
      if (CurrState >= STATE_ROLLOVER)
        CurrState = 0;
      Serial.printf("New state = %d\n", CurrState);
      LastStateChangeTime = now;
      LastBlinkTime = now;
      digitalWrite(LED_PIN, LOW);

      OnChangeState();
    }
  }

  if (CurrState == STATE_IDLE)
  {
    //idle
    //short on
    if (now - LastBlinkTime > 1000)
    {
      digitalWrite(LED_PIN, LOW);
      LastBlinkTime = now;
    }
    if (now - LastBlinkTime > 800)
    {
      digitalWrite(LED_PIN, HIGH);
    }
  }
  else if (CurrState == STATE_CHARGE)
  {
    //charging
    //half on half off
    if (now - LastBlinkTime > 1000)
    {
      digitalWrite(LED_PIN, LOW);
      LastBlinkTime = now;
    }
    if (now - LastBlinkTime > 500)
    {
      digitalWrite(LED_PIN, HIGH);
    }
  }
  else if (CurrState == STATE_OPEN)
  {
    //opening
    //really quick half on half off
    if (now - LastBlinkTime > 25)
    {
      digitalWrite(LED_PIN, LOW);
      LastBlinkTime = now;
    }
    else if (now - LastBlinkTime > 12)
    {
      digitalWrite(LED_PIN, HIGH);
    }
    if (now - LastStateChangeTime > TransitionTimeMS)
    {
      LastStateChangeTime = now;
      CurrState++;
      Serial.println("Swithing to OPEN state!");
      OnChangeState();
    }
  }
  else if (CurrState == STATE_AFTER_OPEN)
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else if (CurrState == STATE_RUNNING)
  {
    //open
    //short off
    if (now - LastBlinkTime > 1000)
    {
      digitalWrite(LED_PIN, LOW);
      LastBlinkTime = now;
    }
    if (now - LastBlinkTime > 200)
    {
      digitalWrite(LED_PIN, HIGH);
    }
  }
  else if (CurrState == STATE_CLOSE)
  {
    //closing
    //really quick half on half off
    if (now - LastBlinkTime > 25)
    {
      digitalWrite(LED_PIN, LOW);
      LastBlinkTime = now;
    }
    else if (now - LastBlinkTime > 12)
    {
      digitalWrite(LED_PIN, HIGH);
    }
    if (now - LastStateChangeTime > TransitionTimeMS)
    {
      LastStateChangeTime = now;
      CurrState = 0;
      Serial.println("Switching to IDLE state!");
      digitalWrite(LED_PIN, LOW);
      OnChangeState();
    }
  }
  delay(1);
  return;
#endif
}

