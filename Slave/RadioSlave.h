#ifndef RadioSlave_h
#define RadioSlave_h

#include <RF24.h>
#include <Arduino.h>

#define MAXPACKETS 3
#define PACKET1 0
#define PACKET2 1
#define PACKET3 2

#define STATE_SCANNING 0
#define STATE_PARTIAL_LOCK 1
#define STATE_FULL_LOCK 2

class RadioSlave
{
private:
    static RadioSlave* handlerInstance;

    // Radio communication parameters
    RF24 radio;
    int8_t channelList[40] = { 15, 102, 87, 62, 95, 33, 100, 78, 81, 92, 26, 39, 105, 12, 36, 96, 60, 84, 21, 48, 90, 27, 75, 9, 70, 93, 18, 102, 81, 30, 63, 108, 48, 57, 36, 99, 78, 87, 38, 25 };
    const uint8_t addressRec[4] = { 'R', 'R', 'R', '\0' };
    const uint8_t addressTransmit[4] = { 'T', 'T', 'T', '\0' };
    const uint8_t channelsToHop = 40;
    const uint8_t framesPerHop = 2;
    int8_t currentChannelIndex = 0;
    uint8_t channelHopCounter = 0;
    const uint8_t hopOnLockValue = framesPerHop - 1;
    uint8_t hopOnScanValue = 0;
    uint8_t hopOnScanCounter = 0;
    uint8_t failedCounter = 0;
    const uint8_t failedBeforeScanning = 50;

    // Frame timing parameters
    uint8_t frameRate = 0;
    uint32_t microsPerFrame = 0;
    uint32_t halfMicrosPerFrame = 0;
    uint32_t frameTimeEnd = 0;
    bool isOverFlowFrame = false;
    uint8_t secondCounter = 0;
    uint8_t receivedPacketCount = 0;
    uint8_t sentPacketCount = 0;
    uint16_t receivedPerSecond = 0;
    uint16_t sentPerSecond = 0;
    bool isSecondTick = false;

    // Packet data parameters
    uint8_t numberOfSendPackets = 0;
    uint8_t numberOfReceivePackets = 0;
    uint8_t* receivePackets[MAXPACKETS];
    uint8_t* sendPackets[MAXPACKETS];
    bool receivePacketsAvailable[MAXPACKETS];
    uint8_t byteAddCounter[MAXPACKETS];
    uint8_t byteReceiveCounter[MAXPACKETS];
    uint8_t packetSize = 0;

    // Radio interrupt handling
    int16_t totalAdjustedDrift = 0;
    uint32_t syncDelay = 0;
    uint32_t minOverflowProtection;
    uint32_t maxOverflowProtection;
    uint8_t partialLockCounter = 0;
    volatile uint8_t radioState = STATE_SCANNING;
    volatile bool isSyncFrame;
    volatile uint32_t interruptTimeStamp = 0;
    volatile uint32_t lastInterruptTimeStamp = 0;

    TaskHandle_t taskHandle = nullptr;

    void ClearSendPackets();
    void ClearReceivePackets();
    void UpdateScanning(bool isSuccess);
    void UpdateSecondCounter();
    void SetNextFrameEnd(uint32_t newTime);
    void AdvanceFrame();
    bool IsFrameReady();
    void AdjustChannelIndex(int8_t amount);
    bool UpdateHop();
    static void StaticIRQHandler();
    void IRQHandler();
    void YieldTask();

    // RTOS Task Function
    static void TaskFunction(void* pvParameters);

public:
    void Init(_SPI* spiPort, uint8_t pinCE, uint8_t pinCS, uint8_t pinIRQ, int8_t powerLevel, uint8_t packetSize, uint8_t numberOfSendPackets, uint8_t numberOfReceivePackets, uint8_t frameRate);
    void WaitAndSend();
    void Receive();
    void ProcessRadioEvent();
    bool IsNewPacket(uint8_t packetId) { return receivePacketsAvailable[packetId]; }
    uint16_t GetReceivedPacketsPerSecond() { return receivedPerSecond; }
    int8_t GetCurrentChannel() { return channelList[currentChannelIndex]; }
    bool IsSecondTick() { return isSecondTick; }
    template <typename T> void AddNextPacketValue(uint8_t packetId, T data);
    template <typename T> T GetNextPacketValue(uint8_t packetId);
};

template <typename T>
void RadioSlave::AddNextPacketValue(uint8_t packetId, T data)
{
    size_t dataLength = sizeof(T);

    if (packetId >= MAXPACKETS || byteAddCounter[packetId] + dataLength > packetSize)
    {
        return;
    }

    memcpy(&sendPackets[packetId][byteAddCounter[packetId]], &data, dataLength);
    byteAddCounter[packetId] += dataLength;
}

template <typename T>
T RadioSlave::GetNextPacketValue(uint8_t packetId)
{
    size_t dataLength = sizeof(T);

    if (packetId >= MAXPACKETS || byteReceiveCounter[packetId] + dataLength > packetSize)
    {
        return 0;
    }

    T value;
    memcpy(&value, &receivePackets[packetId][byteReceiveCounter[packetId]], dataLength);
    byteReceiveCounter[packetId] += dataLength;
    return value;
}

#endif
