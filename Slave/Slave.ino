#include <Arduino.h>
#include <SPI.h>
#include "RadioSlave.h"

#define CE_PIN 5                    // CE Pin connected to the NRF
#define CS_PIN 17                   // CS Pin connected to the NRF
#define IRQ_PIN 4                   // Slave Requires the IRQ Pin connected to the NRF. Arduino Uno/Nano can be Pin 2 or 3
#define POWER_LEVEL 0               // 0 lowest Power, 3 highest Power (Use separate 3.3v power supply for NRF above 0)
#define PACKET_SIZE 32              // Max 32 Bytes. Must match the Master's Packet Size. How many bytes you are maximum packing into each packet. Useable size is 1 less than this as first byte is PacketID and Hopping information
#define NUMBER_OF_SENDPACKETS 2     // Max of 3 Packets. How many packets per frame the slave will send. The master needs to have the same amount of receive packets
#define NUMBER_OF_RECEIVE_PACKETS 1 // Max of 3 Packets. How many packets per frame the slave will receive. The Master needs to have the same amount of send packets
#define FRAME_RATE 50               // Locked frame rate of the microcontroller. Must match the Master's Framerate

RadioSlave radio;
int16_t masterRecPerSecond;
uint32_t masterMicros;
uint16_t value2;
uint8_t value3;

void setup() {
    Serial.begin(115200);
    // Set custom addresses for Master and Slave
    radio.SetAddresses("UST01", "ALT01");
    
    // Generate the channels with lower bound, upper bound, and a seed value
    radio.GenerateChannels(76, 124, 12345);

    // Init must be called first with the following defined Parameters
    radio.Init(&SPI, CE_PIN, CS_PIN, IRQ_PIN, POWER_LEVEL, PACKET_SIZE, NUMBER_OF_SENDPACKETS, NUMBER_OF_RECEIVE_PACKETS, FRAME_RATE);

    // Create the Slave task
    xTaskCreate(slaveTask, "SlaveTask", 4096, NULL, 1, NULL);
}

void loop() {
    // FreeRTOS handles the loop function with tasks, so nothing is needed here.
}

void slaveTask(void *pvParameters) {
    while (1) {
        radio.WaitAndSend();    // Must be called at the start of every frame. Is blocking until the frame time is up
        radio.Receive();        // Call this second on every Frame
        
        ProcessReceived();      // Method below to process received data
        AddSendData();          // Method below to add Send data

        if (radio.IsSecondTick()) {
            // Print out received data in a human-readable format
            String dataString = "---- Slave Received Data ----\n";
            dataString += "Master Rec. Per Second: " + String(masterRecPerSecond) + "\n";
            dataString += "Received Microseconds: " + String(masterMicros) + "\n";
            dataString += "Received 16-bit value: " + String(value2) + "\n";
            dataString += "Received 8-bit value: " + String(value3) + "\n";
            dataString += "----------------------------\n";
            
            // Print the entire string at once
            Serial.print(dataString);
        }

        vTaskDelay(1);  // Yield to allow other tasks to run
    }
}

void AddSendData() {
    int16_t slaveRecPerSecond = radio.GetRecievedPacketsPerSecond();  // Get the number of Packets we are receiving per second
    int16_t number16Bit = 23145;      // Useless variable we will send
    uint8_t numberU8Bit = 50;         // Useless variable we will send
    float numberFloat = 302.234f;     // Useless variable we will send
    uint32_t numberU32Bit = 2342521;  // Useless variable we will send

    // Add data to Packet 1. We can add 1 less byte than packet byte size
    radio.AddNextPacketValue(PACKET1, slaveRecPerSecond);
    radio.AddNextPacketValue(PACKET1, number16Bit);
    radio.AddNextPacketValue(PACKET1, numberU8Bit);

    // Add data to Packet 2. We can add 1 less byte than packet byte size
    radio.AddNextPacketValue(PACKET2, numberFloat);
    radio.AddNextPacketValue(PACKET2, numberU32Bit);
}

void ProcessReceived() {
    // Must call IsNewPacket before processing
    // Packet contents must be processed in the order they were sent from the Master
    // On receiving, the template also Requires a typecast to the type we are retrieving eg <int16_t>

    if (radio.IsNewPacket(PACKET1)) {  // Call to see if there's new values for Packet1
        masterRecPerSecond = radio.GetNextPacketValue<int16_t>(PACKET1);
        masterMicros = radio.GetNextPacketValue<uint32_t>(PACKET1);
        value2 = radio.GetNextPacketValue<uint16_t>(PACKET1);
        value3 = radio.GetNextPacketValue<uint8_t>(PACKET1);
    }
}
