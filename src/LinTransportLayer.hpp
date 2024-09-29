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
#include "LinFrameTransfer.hpp"

class LinTransportLayer : protected LinFrameTransfer{
public:
    using LinFrameTransfer::LinFrameTransfer;

    using LinFrameTransfer::begin;
    using LinFrameTransfer::end;

    std::optional<std::vector<uint8_t>> writePDU(uint8_t* NAD, const std::vector<uint8_t>& payload, const uint8_t newNAD = 0);

#ifdef UNIT_TEST
public:
    using LinFrameTransfer::mock_loopback;
    using LinFrameTransfer::mock_Input;
    using LinFrameTransfer::txBuffer;
#endif

protected:
    // Packet Data Unit (PDU)
    union PDU {
    // 4.2.3.2 NAD = Node Address
        enum NAD : uint8_t {
    // 0     (0x00) = Reserved for go to sleep command, see Section 2.6.3
            SLEEP = 0x00,
    // 1-125 (0x01-0x7D) = Slave Node Adress (NAD)
    // 126   (0x7E) = Functional node address (functional NAD), only used for diagnostics (using the transport layer)
            FUNCTIONAL = 0x7E,
    // 127   (0x7F) = Slave node address broadcast (broadcast NAD)
            BROADCAST = 0x7F  // = wildcard
    // 128-255   (0x80-0xFF) = Free usage
        };

    // 3.2.1.3 PCI
    // 4.2.3.3 PCI = Protocol Control Information (= Length of Message)
        enum PCI : uint8_t {
            MASK_PCI_TYPE = 0xF0,
            MASK_PCI_LEN = 0x0F, // SF and FF
            MASK_PCI_SN = 0x0F, // CF

            SF_Type = 0x00,
            FF_Type = 0x10,
            CF_Type = 0x20,
            Sleep_Type = 0xFF
        };

        // Single Frame, payload fits into the single PDU
        struct SingleFrame {
            uint8_t NAD {0x00};
            uint8_t PCI_LEN {SF_Type}; // B7..B4 = type (SF=0, FF=1, CF=2)
                                       // B3..B0 = LEN of data bytes
            uint8_t data[6] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        };

        // First Frame, when payload does not fit into a single PDU, followed by Consecutive Frames
        struct FirstFrame {
            uint8_t NAD {0x00};
            uint8_t PCI_LEN256 {FF_Type}; // B7..B4 = type (SF=0, FF=1, CF=2)
                                          // B3..B0 = LEN/256 of databytes
            uint8_t LEN {0xFF}; // LEN & 0xFF of data bytes
            uint8_t data[5] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        };

        // Consecutive Frames follows on FF, when payload does not fit into a single PDU
        struct ConsecutiveFrame {
            uint8_t NAD {0x00};
            uint8_t PCI_SN {CF_Type}; // B7..B4 = type (SF=0, FF=1, CF=2)
                                      // B3..B0 = Frame Counter % 0x0F (wraps around)
            uint8_t data[6] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        };
    };

    std::vector<std::vector<uint8_t>> framesetFromPayload(const uint8_t *NAD, const std::vector<uint8_t> &payload);
    inline void fillSingleFrame(LinTransportLayer::PDU::SingleFrame *singleFrame, const uint8_t *NAD, const std::vector<uint8_t> &payload);
    inline void fillFirstFrame(LinTransportLayer::PDU::FirstFrame *firstFrame, const uint8_t *NAD, const std::vector<uint8_t> &payload, int &bytesWritten);
    inline void fillConsecutiveFrame(LinTransportLayer::PDU::ConsecutiveFrame *consecutiveFrame, const uint8_t *NAD, const uint8_t sequenceNumber, const std::vector<uint8_t> &payload, int &bytesWritten);

private:
    std::optional<std::vector<uint8_t>> readPduResponse(uint8_t *NAD, const uint8_t newNAD = 0);
    void readSingleFrame(PDU::SingleFrame* singleFrame, std::vector<uint8_t> &payload);
    bool readFirstFrame(PDU::FirstFrame* firstFrame, std::vector<uint8_t> &payload, size_t &announcedBytes);
    bool readConsecutiveFrame(PDU::ConsecutiveFrame* consecutiveFrame, std::vector<uint8_t> &payload, size_t &announcedBytes, int frameCounter);
};