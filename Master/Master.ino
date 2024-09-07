#include <Arduino.h>
#include <SPI.h>
#include "RadioMaster.h"

// In this example, we are hopping between 40 different channels. We change channel once every 2 frames. 
// At 50HZ, we are changing channels every 40ms.
// The Master is sending 1 packet per Frame - It should be receiving up to 50 packets per Second.
// The Slave is sending 2 packets per frame - It should be receiving up to 100 packets per Second.
// Each packet can take up to 31 bytes of useable data. Adding and retrieving data is done sequentially 
// with an example below on how to do this. Make sure you set 115200 Baud Rate in your Serial Monitor.

#define CE_PIN 5                      // CE Pin connected to the NRF
#define CS_PIN 17                     // CS Pin connected to the NRF
#define POWER_LEVEL 0                 // 0 lowest Power, 3 highest Power (Use separate 3.3v power supply for NRF above 0)
#define PACKET_SIZE 32                // Max 32 Bytes. Must match the slave packet size. How many bytes you are maximum packing into each packet. Useable size is 1 less than this as first byte is PacketID and Hopping information
#define NUMBER_OF_SENDPACKETS 2       // Max of 3 Packets. How many packets per frame the Master will send. The Slave needs to have the same amount of receive packets
#define NUMBER_OF_RECEIVE_PACKETS 2   // Max of 3 Packets. How many packets per frame the Master will receive. The Slave needs to have the same amount of send packets
#define FRAME_RATE 50                 // Locked frame rate of the microcontroller. Must match the Slaves Framerate

RadioMaster radio;
int16_t slaveRecPerSecond;
int16_t lastSlaveRecPerSecond = 0;
int16_t lastNumber16Bit = 0;
uint8_t lastNumberU8Bit = 0;
float lastNumberFloat = 0.0;
uint32_t lastNumberU32Bit = 0;

void setup() {
    Serial.begin(115200);
    
    // Set custom addresses for Master and Slave
    radio.SetAddresses("UST01", "ALT01");
    
    // Generate the channels with lower bound, upper bound, and a seed value
    radio.GenerateChannels(76, 124, 12345);

    // Init must be called first with the following defined Parameters
    radio.Init(&SPI, CE_PIN, CS_PIN, POWER_LEVEL, PACKET_SIZE, NUMBER_OF_SENDPACKETS, NUMBER_OF_RECEIVE_PACKETS, FRAME_RATE);

    // Create the Master task on Core 1, Wifi/BT runs on Core 0
    xTaskCreatePinnedToCore(masterTask, "MasterTask", 4096, NULL, 1, NULL, 1);
}

void masterTask(void *pvParameters) {
    Serial.println("Master Task On: " + String(xPortGetCoreID()) + " | " + String(ESP.getCpuFreqMHz()) + "MHz");
    while (1) {
        radio.WaitAndSend();
        radio.Receive();
        
        ProcessReceived();
        AddSendData();

        if (radio.IsSecondTick()) {
            // Print out received data in a human-readable format
            String dataString = "---- Master Received Data ----\n";
            dataString += "Slave/Master Rec. Per Second: " + String(lastSlaveRecPerSecond) + " | " + String(radio.GetRecievedPacketsPerSecond()) + "\n";
            dataString += "Received 16-bit value: " + String(lastNumber16Bit) + "\n";
            dataString += "Received 8-bit value: " + String(lastNumberU8Bit) + "\n";
            dataString += "Received Float: " + String(lastNumberFloat, 2) + "\n";
            dataString += "Received 32-bit value: " + String(lastNumberU32Bit) + "\n";
            dataString += "-----------------------------\n";

            // Print the entire string at once
            Serial.print(dataString);

        }

        vTaskDelay(1);  // Yield to allow other tasks to run
    }
}

// Functions Below to show how to add and retrieve data from each packet
void AddSendData() {
    // Data can be sent using PACKET1, PACKET2, or PACKET3
    // The number of sent and received packets in use per frame is set as one of the definitions and passed into the Init Function
    // Data is added sequentially into each packet. The amount of data we can add into the packet is 1 less than Define PACKET_SIZE which is passed into the Init Function
    // Eg for 4 x int16_t values we need 8 bytes + 1. So PACKET_SIZE should be 9 bytes
    // Both Master and Slave need to have the same PACKET_SIZE. Not all bytes need to be used in each packet

    int16_t masterRecPerSecond = radio.GetRecievedPacketsPerSecond();
    uint32_t masterMicros = micros();
    uint16_t value2 = 5343;
    uint8_t value3 = 143; 

    radio.AddNextPacketValue(PACKET1, masterRecPerSecond);
    radio.AddNextPacketValue(PACKET1, masterMicros);
    radio.AddNextPacketValue(PACKET1, value2);
    radio.AddNextPacketValue(PACKET1, value3);
}

void ProcessReceived() {
    // Must call IsNewPacket before processing
    // When retrieving the data from the Packet, we must do it in the same order as it was added on the slave
    // On receiving, the template also Requires a typecast to the type we are retrieving eg <int16_t>

    if (radio.IsNewPacket(PACKET1)) {  // Call to see if there's new values for Packet1
        lastSlaveRecPerSecond = radio.GetNextPacketValue<int16_t>(PACKET1);
        lastNumber16Bit = radio.GetNextPacketValue<int16_t>(PACKET1);
        lastNumberU8Bit = radio.GetNextPacketValue<uint8_t>(PACKET1);  // Corrected to uint8_t
    }

    if (radio.IsNewPacket(PACKET2)) {  // Call to see if there's new values for Packet2
        lastNumberFloat = radio.GetNextPacketValue<float>(PACKET2);
        lastNumberU32Bit = radio.GetNextPacketValue<uint32_t>(PACKET2);
    }
}

void loop() {vTaskDelete(NULL);}