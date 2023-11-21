#include "EspNowRelay.h"
#include <WiFi.h>
#include "../../Relay-EspNow/include/NowMessages.h"
#include <set>

static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

EventGroupHandle_t ConnectResponseEvent;
static int ConnectResponse;
static std::set<int> OpenSockets;

void xxd(const unsigned char* data, unsigned int dataLen)
{
  for (int i = 0; i < dataLen; i++)
  {
    Serial.printf("%02hhx ", data[i]);
  }
  Serial.println("");
}

int SendWithRetries(const unsigned char* data, int dataLength)
{
    for (int i = 0; i < 5; i++)
    {
        if (esp_now_send(NULL, data, dataLength) == ESP_NOW_SEND_SUCCESS)
            return dataLength;
    }
    Serial.println("Send timed out");

    return -1;
}

static bool DidInit = false;
static void EspNowInit()
{
    if (DidInit)
        return;
    WiFi.mode(WIFI_MODE_STA);
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    Serial.println("ESP-NOW Initialized");
    ConnectResponseEvent = xEventGroupCreate();

    esp_now_register_send_cb(OnDataSent);    
    esp_now_register_recv_cb(OnDataRecv);
    DidInit = true;
}


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    NowMsgHeader* header = (NowMsgHeader*)incomingData;
    Serial.printf("Received %d bytes from %02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx, type = %hhu\n", len, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], header->m_Type);
    xxd(incomingData, len);
    switch (header->m_Type)
    {
        case NOWMSG_CONNECT_RESULT:
        {
            NowMsgConnectResult* msg = (NowMsgConnectResult*)header;
            ConnectResponse = msg->m_Socket;
            Serial.printf("Got connect result: %hhu\n", msg->m_Socket);
            xEventGroupSetBits(ConnectResponseEvent, 1);
            //TODO: finish this
        }
        break;

        case NOWMSG_DATA:
        {
            NowMsgData* msg = (NowMsgData*)header;
            Serial.printf("Received data, length = %d\n", len - sizeof(NowMsgData));
            xxd((unsigned char*)(msg + 1), len - sizeof(NowMsgData));
            //TODO: handle data!
        }
        break;

        case NOWMSG_KEEPALIVE_REQ:
        {
            NowMsgSocketKeepaliveRequest* msg = (NowMsgSocketKeepaliveRequest*)header;
            Serial.printf("Got keepalive request for socket %d at %lu\n", msg->m_Socket, millis());
            NowMsgSocketKeepaliveResponse out;
            out.m_Header.m_Type = NOWMSG_KEEPALIVE_RESP;
            out.m_Socket = msg->m_Socket;
            out.m_InUse = OpenSockets.find(msg->m_Socket) != OpenSockets.end();
            Serial.printf("in use = %c\n", out.m_InUse ? 'Y' : 'N');
            SendWithRetries((const unsigned char*)&out, sizeof(out));
        }
        break;
    }
}

static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

EspNowRelay::EspNowRelay()
{
}

bool EspNowRelay::Init(unsigned char mac[6])
{
    EspNowInit();
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add turret peer");
        return false;
    }

    Serial.println("ESP-NOW Relay initialized");
    return true;
}

int EspNowRelay::Connect(unsigned int addr, unsigned short port)
{
    NowMsgConnect msg;
    msg.m_Header.m_Type = NOWMSG_CONNECT;
    
    msg.m_Address = addr;
    msg.m_Port = port;
    
    xEventGroupClearBits(ConnectResponseEvent, 1);
    SendWithRetries((unsigned char*)&msg, sizeof(msg));
    if (xEventGroupWaitBits(ConnectResponseEvent, 1, pdTRUE, pdFALSE, 10000) == 0) //how long are ticks?  assuming 1ms?
    {
        Serial.println("Timed out waiting for reply\n");
        return -1; //failed
    }

    //keep track of all open sockets
    if (ConnectResponse > 0)
        OpenSockets.insert(ConnectResponse);
    return ConnectResponse;
}

void EspNowRelay::Close(int sock)
{
    NowMsgClose msg;
    msg.m_Header.m_Type = NOWMSG_CLOSE;
    msg.m_Socket = sock;

    SendWithRetries((unsigned char*)&msg, sizeof(msg));
}

int EspNowRelay::Send(int sock, const unsigned char* data, int dataLength)
{
    unsigned char out[250];
    NowMsgData* msg = (NowMsgData*)out;
    msg->m_Header.m_Type = NOWMSG_DATA;
    msg->m_Socket = sock;

    const static int MaxDataBuffer = 250 - sizeof(NowMsgData);

    for (int pos = 0; pos < dataLength; pos += MaxDataBuffer)
    {
        int numBytes = dataLength - pos;
        if (numBytes > MaxDataBuffer)
            numBytes = MaxDataBuffer;
        memcpy(out + sizeof(NowMsgData), data + pos, numBytes);
        if (SendWithRetries(out, sizeof(NowMsgData) + numBytes) != sizeof(NowMsgData) + numBytes)
            return -1;
    }

    return dataLength;
}

int EspNowRelay::Recv(int sock, unsigned char* data, int maxDataLength)
{
    return -1; //TODO
}


