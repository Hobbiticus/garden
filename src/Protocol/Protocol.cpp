#include "Protocol.h"

#if defined(__AVR__) or defined(ESP8266) or defined(ESP32)

#include <Arduino.h>
#include <WiFi.h>

#else

#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#endif


unsigned int BaseMessage::Serialize(unsigned char* buffer, unsigned int bufferSize)
{
    buffer[2] = GetMessageType();
    buffer[3] = m_ID;
    unsigned int length = PrivateSerialize(buffer + 4, bufferSize - 4);

    *((unsigned short*)buffer) = htons(length + 2);
    return length + 4;
}

unsigned int BaseMessage::Deserialize(unsigned char* buffer, unsigned int bufferSize)
{
    m_ID = buffer[0];
    return PrivateDeserialize(buffer + 1, bufferSize - 1);
}


unsigned int StartupEvent::PrivateSerialize(unsigned char* buffer, unsigned int bufferSize)
{
    *((unsigned short*)buffer) = htons(m_VoltageHundredths);
    return 2;
}

unsigned int StartupEvent::PrivateDeserialize(unsigned char* buffer, unsigned int bufferSize)
{
    m_VoltageHundredths = ntohs(*((unsigned short*)buffer));
    return 2;
}

unsigned int SumpEvent::PrivateSerialize(unsigned char* buffer, unsigned int bufferSize)
{
    buffer[0] = m_On;
    return 1;
}

unsigned int SumpEvent::PrivateDeserialize(unsigned char* buffer, unsigned int bufferSize)
{
    m_On = buffer[0];
    return 1;
}

unsigned int BatteryStatus::PrivateSerialize(unsigned char* buffer, unsigned int bufferSize)
{
    *((unsigned short*)buffer) = htons(m_VoltageHundredths);
    return 2;
}

unsigned int BatteryStatus::PrivateDeserialize(unsigned char* buffer, unsigned int bufferSize)
{
    m_VoltageHundredths = ntohs(*((unsigned short*)buffer));
    return 2;
}

unsigned int ErrorMessage::PrivateSerialize(unsigned char* buffer, unsigned int bufferSize)
{
    buffer[0] = (unsigned char)strlen(m_Message);
    memcpy(buffer + 1, m_Message, buffer[0]);
    return 1 + buffer[0];
}

unsigned int ErrorMessage::PrivateDeserialize(unsigned char* buffer, unsigned int bufferSize)
{
    unsigned char stringLen = buffer[0];
    memcpy(m_Message, buffer + 1, stringLen);
    m_Message[stringLen] = '\0';
    return 1 + buffer[0];
}

unsigned int MoistureReadings::PrivateSerialize(unsigned char* buffer, unsigned int bufferSize)
{
    unsigned short* dst = (unsigned short*)buffer;
    for (int i = 0; i < MaxReadings; i++)
	    dst[i] = htons(m_Readings[i]);
    return sizeof(unsigned short) * MaxReadings;
}

unsigned int MoistureReadings::PrivateDeserialize(unsigned char* buffer, unsigned int bufferSize)
{
    unsigned short* src = (unsigned short*)buffer;
    for (int i = 0; i < MaxReadings; i++)
	    m_Readings[i] = ntohs(src[i]);
    return sizeof(unsigned short) * MaxReadings;
}
