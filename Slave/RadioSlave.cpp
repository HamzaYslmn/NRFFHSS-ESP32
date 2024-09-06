#include "RadioSlave.h"

RadioSlave* RadioSlave::handlerInstance = nullptr;

void IRAM_ATTR RadioSlave::StaticIRQHandler()
{
    if (handlerInstance != nullptr)
    {
        handlerInstance->IRQHandler();
    }
}

void RadioSlave::IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the task that the interrupt has occurred
    vTaskNotifyGiveFromISR(taskHandle, &xHigherPriorityTaskWoken);

    // Yield to the higher priority task if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void RadioSlave::Init(_SPI* spiPort, uint8_t pinCE, uint8_t pinCS, uint8_t pinIRQ, int8_t powerLevel, uint8_t packetSize, uint8_t numberOfSendPackets, uint8_t numberOfReceivePackets, uint8_t frameRate)
{
    handlerInstance = this;
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

    // Attach interrupt for radio
    attachInterrupt(digitalPinToInterrupt(pinIRQ), StaticIRQHandler, FALLING);

    // Create the task that will handle processing
    xTaskCreatePinnedToCore(
        RadioSlave::TaskFunction, // Function to run
        "RadioTask",              // Name of the task
        2048,                     // Stack size
        this,                     // Parameters to pass to the task (your class object in this case)
        5,                        // Priority
        &taskHandle,              // Task handle
        1                          // Core to pin to (1 in this case)
    );

    // Frame timing
    this->frameRate = constrain(frameRate, 10, 120);
    microsPerFrame = 1000000 / this->frameRate;
    halfMicrosPerFrame = microsPerFrame / 2;
    minOverflowProtection = microsPerFrame * 3;
    maxOverflowProtection = 0xFFFFFFFF - (microsPerFrame * 3);
    syncDelay = microsPerFrame / 8;
}

void RadioSlave::TaskFunction(void* pvParameters)
{
    RadioSlave* instance = (RadioSlave*)pvParameters;

    while (1)
    {
        // Wait for the notification from the ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Process the interrupt event here
        instance->ProcessRadioEvent();
    }
}

void RadioSlave::ProcessRadioEvent()
{
    uint32_t localInterruptTimeStamp = micros() + syncDelay;

    if (localInterruptTimeStamp - lastInterruptTimeStamp >= halfMicrosPerFrame)
    {
        isSyncFrame = true;
        lastInterruptTimeStamp = localInterruptTimeStamp;
    }

    // Additional processing if needed...
}

void RadioSlave::ClearSendPackets()
{
    for (int i = 0; i < numberOfSendPackets; i++)
    {
        memset(sendPackets[i], 0, packetSize);
        byteAddCounter[i] = 1;
    }
}

void RadioSlave::ClearReceivePackets()
{
    for (int i = 0; i < numberOfReceivePackets; i++)
    {
        receivePacketsAvailable[i] = false;
        memset(receivePackets[i], 0, packetSize);
        byteReceiveCounter[i] = 1;
    }
}

void RadioSlave::SetNextFrameEnd(uint32_t newTime)
{
    isOverFlowFrame = (newTime < frameTimeEnd);
    frameTimeEnd = newTime;
}

void RadioSlave::AdvanceFrame()
{
    uint32_t localInterruptTimeStamp = interruptTimeStamp;
    bool localIsSyncFrame = isSyncFrame;
    isSyncFrame = false;

    if (localIsSyncFrame)
    {
        if (localInterruptTimeStamp > maxOverflowProtection || localInterruptTimeStamp < minOverflowProtection)
        {
            SetNextFrameEnd(frameTimeEnd + microsPerFrame);
            return;
        }

        uint32_t futureLocalInterruptTimeStamp = localInterruptTimeStamp + microsPerFrame;
        int32_t diffA = localInterruptTimeStamp - frameTimeEnd;
        int32_t diffB = futureLocalInterruptTimeStamp - frameTimeEnd;
        int32_t drift = (abs(diffA) < abs(diffB)) ? diffA : diffB;

        SetNextFrameEnd(frameTimeEnd + microsPerFrame + drift);
        totalAdjustedDrift += (drift < 0) ? -1 : 1;
    }
    else
    {
        SetNextFrameEnd(frameTimeEnd + microsPerFrame);
    }
}

bool RadioSlave::IsFrameReady()
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

void RadioSlave::UpdateScanning(bool isSuccess)
{
    if (isSuccess)
    {
        if (radioState == STATE_SCANNING)
        {
            AdjustChannelIndex(2);
            radio.startListening();
            radioState = STATE_PARTIAL_LOCK;
            partialLockCounter = 0;
        }
        else if (radioState == STATE_PARTIAL_LOCK)
        {
            partialLockCounter++;
            if (partialLockCounter > 10)
            {
                radioState = STATE_SCANNING;
            }
            else if (isSuccess)
            {
                radioState = STATE_FULL_LOCK;
            }
        }
    }
    else
    {
        failedCounter++;
    }

    if (failedCounter >= failedBeforeScanning)
    {
        failedCounter = 0;
        radioState = STATE_SCANNING;
    }
}

void RadioSlave::UpdateSecondCounter()
{
    secondCounter++;
    isSecondTick = false;
    if (secondCounter >= frameRate)
    {
        secondCounter = 0;
        receivedPerSecond = receivedPacketCount;
        receivedPacketCount = 0;
        sentPerSecond = sentPacketCount;
        sentPacketCount = 0;
        isSecondTick = true;
    }
}

void RadioSlave::AdjustChannelIndex(int8_t amount)
{
    currentChannelIndex = (currentChannelIndex + amount + channelsToHop) % channelsToHop;

    hopOnScanCounter++;
    if (hopOnScanCounter >= channelsToHop)
    {
        hopOnScanCounter = 0;
        hopOnScanValue = (hopOnScanValue + 1) % framesPerHop;
    }

    radio.stopListening();
    radio.setChannel(channels_Gen[currentChannelIndex]);
}

bool RadioSlave::UpdateHop()
{
    channelHopCounter = (channelHopCounter + 1) % framesPerHop;
    if (radioState == STATE_SCANNING && channelHopCounter == hopOnScanValue)
    {
        AdjustChannelIndex(-1);
        return true;
    }
    else if (radioState == STATE_FULL_LOCK && channelHopCounter == hopOnLockValue)
    {
        AdjustChannelIndex(1);
        return true;
    }
    return false;
}

void RadioSlave::WaitAndSend()
{
    while (!IsFrameReady())
    {
        YieldTask();
    }

    bool hasStoppedListening = UpdateHop();
    if (radioState == STATE_FULL_LOCK)
    {
        if (!hasStoppedListening)
        {
            radio.stopListening();
        }

        for (int i = 0; i < numberOfSendPackets; i++)
        {
            sendPackets[i][0] = i;
            radio.write(sendPackets[i], packetSize);
        }
    }

    radio.startListening();
    ClearSendPackets();
}

void RadioSlave::Receive()
{
    bool isSuccess = false;
    ClearReceivePackets();

    for (int i = 0; i < 3; i++)  // Check multiple times to clear input buffers
    {
        if (radio.available())
        {
            isSuccess = true;
            receivedPacketCount++;
            uint8_t currentPacket[packetSize];
            radio.read(currentPacket, packetSize);
            uint8_t firstByte = currentPacket[0];
            uint8_t packetId = firstByte & 0x03;
            memcpy(receivePackets[packetId], currentPacket, packetSize);
            receivePacketsAvailable[packetId] = true;

            uint8_t txChannelHopCounter = (firstByte & 0xE0) >> 5;
            channelHopCounter = txChannelHopCounter;
        }
    }

    UpdateScanning(isSuccess);
    UpdateSecondCounter();
}

void RadioSlave::YieldTask()
{
    vTaskDelay(1);  // Yield to prevent watchdog timer errors
}

void RadioSlave::setChannelSeed(uint8_t lowerBound, uint8_t upperBound, uint32_t seed)
{
    GenerateChannels(lowerBound, upperBound, seed);
}

void RadioSlave::setMasterID(const char* masterID)
{
    memcpy(this->masterID, masterID, 6);
}

void RadioSlave::setSlaveID(const char* slaveID)
{
    memcpy(this->slaveID, slaveID, 6);
}

void RadioSlave::GenerateChannels(uint8_t lowerBound, uint8_t upperBound, uint32_t seed)
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