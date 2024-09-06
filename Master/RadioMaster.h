#ifndef RadioMaster_h
#define RadioMaster_h

#include <RF24.h>
#define MAXPACKETS 3
#define PACKET1 0
#define PACKET2 1
#define PACKET3 2

class RadioMaster
{

private:
// Radio communication parameters
  RF24 radio;
  // New member variables for IDs and seed
  uint8_t channels_Gen[40]; // For dynamically generated channels
  uint8_t masterID[6];
  uint8_t slaveID[6];
  const uint8_t channelsToHop = 40;
  const uint8_t framesPerHop = 2;
  int8_t currentChannelIndex = 0;
  uint8_t channelHopCounter = 0;

//Frame Timing Stuff
  uint8_t frameRate = 0;
  uint32_t microsPerFrame = 0;
  uint32_t frameTimeEnd = 0;
  bool isOverFlowFrame = false;
  uint8_t secondCounter = 0;
  uint8_t recievedPacketCount = 0;
  uint16_t receivedPerSecond = 0;
  bool isSecondTick = false;

//Packet Data
  uint8_t numberOfSendPackets = 0;
  uint8_t numberOfReceivePackets = 0;
  uint8_t* recievePackets[MAXPACKETS];
  uint8_t* sendPackets[MAXPACKETS];
  bool receivePacketsAvailable[MAXPACKETS];
  uint8_t byteAddCounter[MAXPACKETS];
  uint8_t byteReceiveCounter[MAXPACKETS];
  uint8_t packetSize = 0;

  void ClearSendPackets();
  void ClearReceivePackets();
  void UpdateRecording();
  void AdvanceFrame();
  bool IsFrameReady();
  // Helper function to generate channels based on seed
  void GenerateChannels(uint8_t lowerBound, uint8_t upperBound, uint32_t seed);

public:
  void Init(_SPI* spiPort, uint8_t pinCE, uint8_t PinCS, int8_t powerLevel, uint8_t packetSize, uint8_t numberOfSendPackets, uint8_t numberOfReceivePackets, uint8_t frameRate);
  void WaitAndSend();
  void Receive();
  bool IsNewPacket(uint8_t packetId) {return receivePacketsAvailable[packetId]; }
  int16_t GetRecievedPacketsPerSecond() {return receivedPerSecond; }
  int8_t GetCurrentChannel() { return channels_Gen[currentChannelIndex]; }
  bool IsSecondTick() {return isSecondTick; }
  template <typename T> void AddNextPacketValue(uint8_t packetId, T data);
  template <typename T> T GetNextPacketValue(uint8_t packetId);

  // New functions for seed and IDs
  void setChannelSeed(uint8_t lowerBound, uint8_t upperBound, uint32_t seed);
  void setMasterID(const char* masterID);
  void setSlaveID(const char* slaveID);
};


template <typename T>
void RadioMaster::AddNextPacketValue(uint8_t packetId, T data) 
{
    size_t dataLength = sizeof(T);
    if (packetId >= MAXPACKETS) 
    {
        return;
    }

    if (byteAddCounter[packetId] + dataLength > packetSize) 
    {
      return;
    }

    memcpy(&sendPackets[packetId][byteAddCounter[packetId]], &data, dataLength);

    byteAddCounter[packetId] += dataLength;
}

template <typename T>
T RadioMaster::GetNextPacketValue(uint8_t packetId) 
{

    size_t dataLength = sizeof(T);

    if (packetId >= MAXPACKETS) {
        return 0;
    }

    if (byteReceiveCounter[packetId] + dataLength > packetSize) 
    {
        return 0;
    }

    T value;
    memcpy(&value, &recievePackets[packetId][byteReceiveCounter[packetId]], dataLength);
    byteReceiveCounter[packetId] += dataLength;
    return value;
}

#endif
