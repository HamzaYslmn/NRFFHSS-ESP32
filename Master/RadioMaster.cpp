#include "RadioMaster.h"

void RadioMaster::Init(_SPI* spiPort, uint8_t pinCE, uint8_t PinCS, int8_t powerLevel, uint8_t packetSize, uint8_t numberOfSendPackets, uint8_t numberOfReceivePackets, uint8_t frameRate)
{
  //Packets
  this->numberOfSendPackets = (numberOfSendPackets < 0) ? 0 : ((numberOfSendPackets > 3) ? 3 : numberOfSendPackets);
  this->numberOfReceivePackets = (numberOfReceivePackets < 0) ? 0 : ((numberOfReceivePackets > 3) ? 3 : numberOfReceivePackets);
  this->packetSize = (packetSize < 1) ? 1 : ((packetSize > 32) ? 32 : packetSize);
  powerLevel = (powerLevel < 0) ? 0 : ((powerLevel > 3) ? 3: powerLevel);

  for (int i = 0; i < numberOfSendPackets; ++i) 
  {
    sendPackets[i] = new uint8_t[packetSize]();
  }

  for (int i = 0; i < numberOfReceivePackets; ++i) 
  {
    recievePackets[i] = new uint8_t[packetSize]();
  }

  ClearSendPackets();
  ClearReceivePackets();

  //Radio
  spiPort->begin();
  radio.begin(spiPort, pinCE, PinCS);
  radio.stopListening();
  radio.powerDown();
  radio.setPALevel(powerLevel);
  radio.setAddressWidth(3);
  radio.openReadingPipe(1, address[1]);  // Slave address
  radio.openWritingPipe(address[0]);     // Master address
  radio.setDataRate(RF24_1MBPS);
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setPayloadSize(this->packetSize);
  radio.setChannel(channels_Gen[currentChannelIndex]);
  radio.maskIRQ(true, true, false);
  radio.powerUp();
  radio.startListening();

  //Frame Timing
  this->frameRate = (frameRate < 10) ? 10 : ((frameRate > 120) ? 120 : frameRate);  //Clamp between 10 and 120
  microsPerFrame = 1000000 / frameRate;
}

void RadioMaster::SetAddresses(const char* masterID, const char* slaveID)
{
    strncpy((char*)address[0], masterID, 5);  // Master address
    strncpy((char*)address[1], slaveID, 5);   // Slave address
}

void RadioMaster::GenerateChannels(uint8_t lowerBound, uint8_t upperBound, uint32_t seed)
{
    randomSeed(seed);
    uint8_t availableNumbers[upperBound - lowerBound + 1];
    for (uint8_t i = 0; i <= (upperBound - lowerBound); i++) 
    {
        availableNumbers[i] = lowerBound + i;
    }

    for (int i = (upperBound - lowerBound); i > 0; i--) 
    {
        int j = random(0, i + 1);
        uint8_t temp = availableNumbers[i];
        availableNumbers[i] = availableNumbers[j];
        availableNumbers[j] = temp;
    }

    channels_Gen[0] = 125;  // Reserve the first channel as a fixed one (optional)
    for (int i = 1; i < 40; i++) 
    {
        channels_Gen[i] = availableNumbers[i - 1];
    }
}

void RadioMaster::ClearSendPackets()
{
  for(int i = 0; i < numberOfSendPackets; i++)
  {
    memset(sendPackets[i], 0, packetSize);
    byteAddCounter[i] = 1;
  }
}

void RadioMaster::ClearReceivePackets()
{
  for(int i = 0; i < numberOfReceivePackets; i++)
  {
    receivePacketsAvailable[i] = false;
    memset(recievePackets[i], 0, packetSize);
    byteReceiveCounter[i] = 1;
  }
}

void RadioMaster::AdvanceFrame()
{
  uint32_t newTime = frameTimeEnd + microsPerFrame;
  isOverFlowFrame = (newTime < frameTimeEnd);
  frameTimeEnd = newTime;
}

bool RadioMaster::IsFrameReady()
{ 
  uint32_t currentTimeStamp = micros();
  uint32_t localFrameTimeEnd = frameTimeEnd;
   
  if(isOverFlowFrame)
  {
    currentTimeStamp -= 0x80000000;
    localFrameTimeEnd -= 0x80000000;
  }

  if (currentTimeStamp >= localFrameTimeEnd)
	{
    AdvanceFrame();
    return true;
  }
    return false;
}

void RadioMaster::UpdateRecording()
{
  secondCounter++;
  isSecondTick = false;
  if(secondCounter >= frameRate)
  {
    secondCounter = 0;
    receivedPerSecond = recievedPacketCount;
    recievedPacketCount = 0;
    isSecondTick = true;
  }
}

void RadioMaster::WaitAndSend()
{
  while(!IsFrameReady()) {vTaskDelay(1);}

  radio.stopListening();
  
  for(int i = 0; i < numberOfSendPackets; i++)
  {
    sendPackets[i][0] = i;
    sendPackets[i][0] |= ((channelHopCounter << 5) & 0xE0);
    radio.write(sendPackets[i], packetSize);
  }  

  channelHopCounter++;
  if(channelHopCounter >= framesPerHop)
  { 
    channelHopCounter = 0;
    currentChannelIndex++;
    if(currentChannelIndex >= channelsToHop) { currentChannelIndex = 0; }
    radio.setChannel(channels_Gen[currentChannelIndex]);
  }

  radio.startListening();
  ClearSendPackets();
}

void RadioMaster::Receive()
{
  ClearReceivePackets();

  for(int i = 0; i < 3; i++)  //Always check 3 times to clear the input buffers
  {
    if (radio.available())
    {       
      recievedPacketCount++;
      uint8_t currentPacket[packetSize];
      radio.read(currentPacket, packetSize);
      uint8_t firstByte = currentPacket[0];
      uint8_t packetId = firstByte & 0x03;
      memcpy(recievePackets[packetId], currentPacket, packetSize);
      receivePacketsAvailable[packetId] = true;
    }
  }

  UpdateRecording();
}
