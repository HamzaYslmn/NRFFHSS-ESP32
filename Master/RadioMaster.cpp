#include "RadioMaster.h"

void RadioMaster::Init(_SPI* spiPort, uint8_t pinCE, uint8_t pinCS, int8_t powerLevel, uint8_t packetSize, uint8_t numberOfSendPackets, uint8_t numberOfReceivePackets, uint8_t frameRate)
{
    this->numberOfSendPackets = constrain(numberOfSendPackets, 0, MAXPACKETS);
    this->numberOfReceivePackets = constrain(numberOfReceivePackets, 0, MAXPACKETS);
    this->packetSize = constrain(packetSize, 1, 32);
    powerLevel = constrain(powerLevel, 0, 3);

    for (int i = 0; i < this->numberOfSendPackets; ++i)
    {
        sendPackets[i] = new uint8_t[this->packetSize]();
    }

    for (int i = 0; i < this->numberOfReceivePackets; ++i)
    {
        receivePackets[i] = new uint8_t[this->packetSize]();
    }

    ClearSendPackets();
    ClearReceivePackets();

    // Initialize radio
    spiPort->begin();
    radio.begin(spiPort, pinCE, pinCS);
    radio.setPALevel(powerLevel);
    radio.setAddressWidth(3);
    radio.setDataRate(RF24_1MBPS);
    radio.setAutoAck(false);
    radio.setRetries(0, 0);
    radio.setPayloadSize(this->packetSize);
    radio.openReadingPipe(1, slaveID);  // Use slaveID
    radio.openWritingPipe(masterID);    // Use masterID
    radio.setChannel(channels_Gen[currentChannelIndex]);
    radio.startListening();

    // Frame timing
    this->frameRate = constrain(frameRate, 10, 120);
    microsPerFrame = 1000000 / this->frameRate;
}

void RadioMaster::ClearSendPackets()
{
    for (int i = 0; i < numberOfSendPackets; i++)
    {
        memset(sendPackets[i], 0, packetSize);
        byteAddCounter[i] = 1;
    }
}

void RadioMaster::ClearReceivePackets()
{
    for (int i = 0; i < numberOfReceivePackets; i++)
    {
        receivePacketsAvailable[i] = false;
        memset(receivePackets[i], 0, packetSize);
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

    if (isOverFlowFrame)
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
    if (secondCounter >= frameRate)
    {
        secondCounter = 0;
        receivedPerSecond = receivedPacketCount;
        receivedPacketCount = 0;
        isSecondTick = true;
    }
}

void RadioMaster::WaitAndSend()
{
    while (!IsFrameReady())
    {
        YieldTask();
    }

    radio.stopListening();

    for (int i = 0; i < numberOfSendPackets; i++)
    {
        sendPackets[i][0] = i | ((channelHopCounter << 5) & 0xE0);
        radio.write(sendPackets[i], packetSize);
    }

    channelHopCounter = (channelHopCounter + 1) % framesPerHop;
    if (channelHopCounter == 0)
    {
        currentChannelIndex = (currentChannelIndex + 1) % channelsToHop;
        radio.setChannel(channels_Gen[currentChannelIndex]);
    }

    radio.startListening();
    ClearSendPackets();
}

void RadioMaster::Receive()
{
    ClearReceivePackets();

    for (int i = 0; i < 3; i++)  // Check multiple times to clear input buffers
    {
        if (radio.available())
        {
            receivedPacketCount++;
            uint8_t currentPacket[packetSize];
            radio.read(currentPacket, packetSize);
            uint8_t firstByte = currentPacket[0];
            uint8_t packetId = firstByte & 0x03;
            memcpy(receivePackets[packetId], currentPacket, packetSize);
            receivePacketsAvailable[packetId] = true;
        }
    }

    UpdateRecording();
}

void RadioMaster::YieldTask()
{
    vTaskDelay(1);  // Yield to prevent watchdog timer errors
}

void RadioMaster::setChannelSeed(uint8_t lowerBound, uint8_t upperBound, uint32_t seed)
{
    GenerateChannels(lowerBound, upperBound, seed);
}

void RadioMaster::setMasterID(const char* masterID)
{
    memcpy(this->masterID, masterID, 6);
}

void RadioMaster::setSlaveID(const char* slaveID)
{
    memcpy(this->slaveID, slaveID, 6);
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
    channels_Gen[0] = 125;
    for (int i = 1; i < 40; i++)
    {
        channels_Gen[i] = availableNumbers[i - 1];
    }
}
