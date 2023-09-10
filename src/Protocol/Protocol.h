#ifndef Protocol_h
#define Protocol_h

#define MSG_TYPE_STARTUP 0
#define MSG_TYPE_SUMP_EVENT 1
#define MSG_TYPE_BATTERY_STATUS 2
#define MSG_TYPE_ERROR 3
#define MSG_TYPE_MOISTURE_READINGS 4

struct BaseMessage
{
    unsigned char m_ID;
    
    unsigned int Serialize(unsigned char* buffer, unsigned int bufferSize);
    unsigned int Deserialize(unsigned char* buffer, unsigned int bufferSize);
    
protected:
    virtual unsigned int PrivateSerialize(unsigned char* buffer, unsigned int bufferSize) = 0;
    virtual unsigned int PrivateDeserialize(unsigned char* buffer, unsigned int bufferSize) = 0;
    virtual unsigned char GetMessageType() const = 0;
};

struct StartupEvent : public BaseMessage
{
    unsigned short m_VoltageHundredths;
    
protected:
    unsigned int PrivateSerialize(unsigned char* buffer, unsigned int bufferSize);
    unsigned int PrivateDeserialize(unsigned char* buffer, unsigned int bufferSize);
    unsigned char GetMessageType() const { return MSG_TYPE_STARTUP; }
};

struct SumpEvent : public BaseMessage
{
    unsigned char m_On;
    
protected:
    unsigned int PrivateSerialize(unsigned char* buffer, unsigned int bufferSize);
    unsigned int PrivateDeserialize(unsigned char* buffer, unsigned int bufferSize);
    unsigned char GetMessageType() const { return MSG_TYPE_SUMP_EVENT; }
};

struct BatteryStatus : public BaseMessage
{
    unsigned short m_VoltageHundredths;

protected:
    unsigned int PrivateSerialize(unsigned char* buffer, unsigned int bufferSize);
    unsigned int PrivateDeserialize(unsigned char* buffer, unsigned int bufferSize);
    unsigned char GetMessageType() const { return MSG_TYPE_BATTERY_STATUS; }
};

struct ErrorMessage : public BaseMessage
{
    char m_Message[256];

protected:
    unsigned int PrivateSerialize(unsigned char* buffer, unsigned int bufferSize);
    unsigned int PrivateDeserialize(unsigned char* buffer, unsigned int bufferSize);
    unsigned char GetMessageType() const { return MSG_TYPE_ERROR; }
};

struct MoistureReadings : public BaseMessage
{
	const static int MaxReadings = 5;
	unsigned short m_Readings[MaxReadings];

protected:
    unsigned int PrivateSerialize(unsigned char* buffer, unsigned int bufferSize);
    unsigned int PrivateDeserialize(unsigned char* buffer, unsigned int bufferSize);
    unsigned char GetMessageType() const { return MSG_TYPE_MOISTURE_READINGS; }
};

#endif

