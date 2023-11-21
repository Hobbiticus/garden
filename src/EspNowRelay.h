#pragma once

#include <esp_now.h>

class EspNowRelay
{
public:
    EspNowRelay();

    bool Init(unsigned char mac[6]);

    int Connect(unsigned int addr, unsigned short port); //returns sock or negative value for error
    void Close(int sock);
    int Send(int sock, const unsigned char* data, int dataLength);
    int Recv(int sock, unsigned char* data, int maxDataLength);

private:
    static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
};

extern EspNowRelay relay;
