// LinTransportLayer.hpp
//
// Provides transport layer for multi frame messages as Diagnostic Transport Layer (DTL)
//
// LIN Specification 2.2A
// Source https://www.lin-cia.org/fileadmin/microsites/lin-cia.org/resources/documents/LIN_2.2A.pdf

#pragma once

#ifdef UNIT_TEST
    // #include "../test/mock_HardwareSerial.h"
    // using HardwareSerial = mock_HardwareSerial;
#else
    #include <Arduino.h>
#endif

#include <optional>
#include <vector>

#include "LinFrameTransfer.hpp"
#include "LinPDU.hpp"

class LinTransportLayer : protected LinFrameTransfer{
public:
    using LinFrameTransfer::LinFrameTransfer;

    std::optional<std::vector<uint8_t>> writePDU(uint8_t &NAD, const std::vector<uint8_t>& payload, const uint8_t newNAD = 0);

protected:
    inline std::vector<PDU> framesetFromPayload(const uint8_t NAD, const std::vector<uint8_t> &payload);
    inline void fillSingleFrame(PDU &frame, const uint8_t NAD, const std::vector<uint8_t> &payload);
    inline void fillFirstFrame(PDU &frame, const uint8_t NAD, const std::vector<uint8_t> &payload, int &bytesWritten);
    inline void fillConsecutiveFrame(PDU &frame, const uint8_t NAD, const uint8_t sequenceNumber, const std::vector<uint8_t> &payload, int &bytesWritten);

private:
    inline std::optional<std::vector<uint8_t>> readPduResponse(uint8_t &NAD, const uint8_t newNAD = 0);
    inline bool readSingleFrame(PDU &frame, std::vector<uint8_t> &payload);
    inline bool readFirstFrame(PDU &frame, std::vector<uint8_t> &payload, size_t &announcedBytes);
    inline bool readConsecutiveFrame(PDU &frame, std::vector<uint8_t> &payload, const size_t &announcedBytes, int frameCounter);
};
