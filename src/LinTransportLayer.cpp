// LinTransportLayer.cpp
//
// Provides transport layer for multi frame messages as Diagnostic Transport Layer (DTL)
//
// LIN Specification 2.2A
// Source https://www.lin-cia.org/fileadmin/microsites/lin-cia.org/resources/documents/LIN_2.2A.pdf

#include "LinTransportLayer.hpp"

#ifdef UNIT_TEST
    #include "../test/mock_Arduino.h"
    #include "../test/mock_millis.h"
    #include "../test/mock_delay.h"
#else
    #include <Arduino.h>
#endif

#include <optional>
#include <vector>
#include <unordered_map>

#include "LinPDU.hpp"

// LIN2.2A Spec Table 3.2
constexpr auto timeout_DtlSlaveResponse_per_frame = 50; // ms - Spec has higher timeout: ~1 second

// ------------------------------------

/// @brief 
/// @param NAD 
/// @param payload 
/// @param newNAD in case of SID: CONDITIONAL_CHANGE of NAD node will answer by using new AND
/// @return 
std::optional<std::vector<uint8_t>> LinTransportLayer::writePDU(uint8_t &NAD, const std::vector<uint8_t>& payload, uint8_t newNAD)
{
    // prepare frameset
    std::vector<PDU> frameSet = framesetFromPayload(NAD, payload);
    
    // write full frameset
    for (const PDU& frame : frameSet)
    {
        writeFrame(FRAME_ID::MASTER_REQUEST, frame.asVector());
    }

    // read response
    // special case: CONDITINAL_CHANGE of NAD will answer with newNAD
    return readPduResponse(NAD, newNAD);
}

std::vector<PDU> LinTransportLayer::framesetFromPayload(const uint8_t NAD, const std::vector<uint8_t>& payload)
{
    // verify max Len 
    // if (payload.size() >= 4096) { return {}; }

    // Single Frame
    if (payload.size() <= sizeof(PDU::dataLenSingle))
    {
        std::vector<PDU> frameset(1);

        fillSingleFrame(frameset[0], NAD, payload);

        return std::move(frameset);
    }

    // Mulit Frame: FirstFrame + n * ConsecutiveFrame
    auto ceilDiv = [](size_t nominator, size_t denominator) {
        return (nominator + denominator - 1) / denominator;
    };

    auto data_in_CF = payload.size() - PDU::dataLenFirst;
    auto CF_count = ceilDiv(data_in_CF, PDU::dataLenConsecutive);
    std::vector<PDU> frameset(1 + CF_count);

    auto bytesWritten = 0;

    fillFirstFrame(frameset[0], NAD, payload, bytesWritten);
    for (uint8_t i = 1; i<=CF_count; ++i)
    {
        fillConsecutiveFrame(frameset[i], NAD, i, payload, bytesWritten);
    }

    return std::move(frameset);
}

void LinTransportLayer::fillSingleFrame(
    PDU &frame,
    const uint8_t NAD,
    const std::vector<uint8_t> &payload
){
    frame.setNAD(NAD);
    frame.singleFrame.setDataAndLen(payload);
}

void LinTransportLayer::fillFirstFrame(
    PDU &frame,
    const uint8_t NAD,
    const std::vector<uint8_t> &payload,
    int &bytesWritten
){
    frame.setNAD(NAD);
    frame.firstFrame.setLen(payload.size());
    bytesWritten += frame.firstFrame.setData(payload);
}

void LinTransportLayer::fillConsecutiveFrame(
    PDU &frame,
    const uint8_t NAD,
    const uint8_t sequenceNumber,
    const std::vector<uint8_t> &payload,
    int &bytesWritten
){
    frame.setNAD(NAD);
    frame.consecutiveFrame.setSequenceNumber(sequenceNumber);
    bytesWritten += frame.consecutiveFrame.setData(payload, bytesWritten);
}

/// @brief Start a PDU SlaveRequest
/// @param NAD Node Adress (via pointer), wildcard will be replaced by received NAD
/// @return payload
std::optional<std::vector<uint8_t>> LinTransportLayer::readPduResponse(uint8_t &NAD, const uint8_t newNAD)
{
    uint8_t acceptedNAD = NAD;
    uint8_t frameCounter = 0;
    size_t announcedBytes;
    std::vector<uint8_t> payload {};

    auto timeout = millis() + timeout_DtlSlaveResponse_per_frame;
    while (millis() < timeout)
    {
        // read first frame and process
        auto rxFrame = readFrame(FRAME_ID::SLAVE_REQUEST, 8);
        
        if (!rxFrame) {
            debugStream.println("Failed to read initial PDU");
            continue;
        }

        if (rxFrame.value().size() != 8)
        {
            debugStream.println("Invalid frame size for PDU");
            continue;
        }

        PDU& frame = *reinterpret_cast<PDU*>(rxFrame.value().data());

        if (0 == frameCounter)
        {
            /// NAD will be replaced...
            if ((PDU::NAD_Type::BROADCAST == acceptedNAD) || // on Wildcard
                (frame.getNAD() == newNAD))             // on NAD Change my config requect
            {
                // Boradcast or Cmd "Conditional Change of NAD" was successfull
                acceptedNAD = frame.getNAD();
            }

            if (acceptedNAD != frame.getNAD())
            {
                // unexpected NAD: ignore Frame
                continue;
            }

            if (PDU::PCI_Type::SINGLE == frame.getType()) {
                if (!readSingleFrame(frame, payload)) {
                    // STRICT: when announcedBytes is greater than 6 bytes, frame shall be ignored
                    acceptedNAD = NAD; // revert in case of wildcard
                    continue;
                }
                break; // success
            }

            if (PDU::PCI_Type::FIRST == frame.getType()) {
                if (!readFirstFrame(frame, payload, announcedBytes)) {
                    // STRICT: when announcedBytes is less than 7 bytes, frame shall be ignored
                    acceptedNAD = NAD; // revert in case of wildcard
                    continue;
                }
                frameCounter++;
                timeout = millis() + timeout_DtlSlaveResponse_per_frame;
                continue;
            }

            // STRICT: unexpected frame type shall be ignored
            acceptedNAD = NAD; // revert in case of wildcard

        } else {
            // frameCounter == isConsecutiveFrame
            // sequence of CF started -> error handling now changed
            if (acceptedNAD != frame.getNAD()) {
                // STRICT: mismatch with received NAT of FirstFrame --> abort
                return {};
            }

            if (PDU::PCI_Type::CONSECUTIVE != frame.getType()) {
                // STRICT: unexpected frame type --> abort
                return {};
            }

            if (!readConsecutiveFrame(frame, payload, announcedBytes, frameCounter)) {
                // STRICT: unexpected sequence number received --> abort
                return {};
            }

            frameCounter++;
            timeout += timeout_DtlSlaveResponse_per_frame;
            if (announcedBytes == payload.size()) {
                break; // success
            }
        }
    }

    if (payload.empty()) {
        // payload must not be empty!
        return {};
    }

    // success
    // may return new NAT
    if ((PDU::NAD_Type::BROADCAST == NAD) || (0 != newNAD)) {
        NAD = acceptedNAD;
    }
    return payload;
}

bool LinTransportLayer::readSingleFrame(PDU &frame, std::vector<uint8_t> &payload)
{
    size_t announcedBytes = frame.singleFrame.getLen();
    if (announcedBytes > PDU::dataLenSingle) {
        // STRICT: when announcedBytes is greater than 6 bytes, frame shall be ignored
        return false;
    }

    payload = frame.singleFrame.getData();
    return true;
}

bool LinTransportLayer::readFirstFrame(PDU &frame, std::vector<uint8_t> &payload, size_t &announcedBytes)
{
    announcedBytes = frame.firstFrame.getLen();
    if (announcedBytes <= PDU::dataLenSingle) {
        // STRICT: when announcedBytes is less than 7 bytes, frame shall be ignored
        return false;
    }

    payload.reserve(announcedBytes);
    if (payload.capacity() != announcedBytes) {
        // STRICT: reception of segmented message shall not start, wenn buffer payload is to small.
        return {};
    }

    auto data = frame.firstFrame.getData();
    payload.insert(payload.end(), data.begin(), data.end());
    return true;
}

bool LinTransportLayer::readConsecutiveFrame(PDU &frame, std::vector<uint8_t> &payload, const size_t &announcedBytes, int frameCounter)
{
    if (!frame.consecutiveFrame.verifySequenceNumber(frameCounter)) {
        return false;
    }
    auto bytesToReceive = announcedBytes - payload.size();
    auto data = frame.consecutiveFrame.getData(bytesToReceive);
    payload.insert(payload.end(), data.begin(), data.end());

    return true;
}
