// Lin_Interface.h
//
// Provides a Hardware LIN Interface
// inherited from the "HardwareSerial"
//
// * Frame sending [Break, Sync, PID, Data, Checksum]
// * Frame request [Break, Sync, PID] and wait for data from slave + verify checksum
//
// * WriteBreak for initiation of a Lin-Frame
// * Convert FID to PID
// * Calculation of Checksum (to be send, or to be verified)

#pragma once

#include <Arduino.h>

class Lin_Interface : private HardwareSerial
{
public:
    //inherit constructor from HardwareSerial (Parameter: int uart_nr)
    using HardwareSerial::HardwareSerial;

    int verboseMode = -1;
    unsigned long baud = 19200;
    int8_t rxPin = -1;
    int8_t txPin = -1;

    // 8 Data Bytes + ChkSum + some space for receiving complete frames
    uint8_t LinMessage[8 + 1 + 4] = {0};
    void writeCmdWakeup();
    void writeCmdSleep();

    bool readFrame(const uint8_t FrameID, const uint8_t expectedDatalen = 0);

    void writeFrame(const uint8_t FrameID, const uint8_t datalen);
    void writeFrameClassic(const uint8_t FrameID, const uint8_t datalen);
    void writeFrameClassicNoChecksum(const uint8_t FrameID, const uint8_t datalen);

protected:
    uint32_t m_bitCycles;
    void startTransmission(const uint8_t ProtectedID);
    size_t writeBreak();
    uint8_t getProtectedID(const uint8_t FrameID);
    uint8_t getChecksum(const uint8_t ProtectedID, const uint8_t datalen);
    bool isChecksumValid(const uint8_t Checksum, const uint8_t ProtectedID, const size_t bytes_received);
};
