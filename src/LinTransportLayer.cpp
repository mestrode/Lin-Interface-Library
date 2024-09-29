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

// LIN2.2A Spec Table 3.2
constexpr auto timeout_DtlSlaveResponse_per_frame = 50; // ms - Spec has higher timeout: ~1 second

// ------------------------------------

/// @brief 
/// @param NAD 
/// @param payload 
/// @param newNAD in case of SID: CONDITIONAL_CHANGE of NAD node will answer by using new AND
/// @return 
std::optional<std::vector<uint8_t>> LinTransportLayer::writePDU(uint8_t* NAD, const std::vector<uint8_t>& payload, uint8_t newNAD)
{
    // prepare frameset
    std::vector<std::vector<uint8_t>> frameSet = framesetFromPayload(NAD, payload);
    
    // write full frameset
    for (std::vector<uint8_t>& frame : frameSet)
    {
        writeFrame(FRAME_ID::MASTER_REQUEST, frame);
    }

    // read response
    // special case: CONDITINAL_CHANGE of NAD will answer with newNAD
    return readPduResponse(NAD, newNAD);
}

std::vector<std::vector<uint8_t>> LinTransportLayer::framesetFromPayload(const uint8_t* NAD, const std::vector<uint8_t>& payload)
{
    // verify max Len 
    // if (payload.size() >= 4096) { return {}; }

    // Single Frame
    if (payload.size() <= sizeof(PDU::SingleFrame::data))
    {
        std::vector<std::vector<uint8_t>> frameset(1, std::vector<uint8_t>(sizeof(PDU::SingleFrame)));

        PDU::SingleFrame* singleFrame = reinterpret_cast<PDU::SingleFrame*>(frameset[0].data());
        fillSingleFrame(singleFrame, NAD, payload);

        return frameset;
    }

    // Multi Frame = First Frame + n * Consecutive Frame
    static_assert(sizeof(PDU::FirstFrame) == sizeof(PDU::ConsecutiveFrame),
        "Size of PDU::FirstFrame does not match PDU::Consecutive");

    auto data_in_CF = payload.size() - sizeof(PDU::FirstFrame::data);
    auto CF_count = (data_in_CF + sizeof(PDU::ConsecutiveFrame::data) - 1) / sizeof(PDU::ConsecutiveFrame::data); // ceiling division
    std::vector<std::vector<uint8_t>> frameset(1 + CF_count, std::vector<uint8_t>(sizeof(PDU::ConsecutiveFrame)));

    auto bytesWritten = 0;

    // First Frame
    PDU::FirstFrame* firstFrame = reinterpret_cast<PDU::FirstFrame*>(frameset[0].data());
    fillFirstFrame(firstFrame, NAD, payload, bytesWritten);

    // a series of Consecutive Frames; Last one maybe contain Fill Bytes
    // Start with 1; FirstFrame was 0
    for (uint8_t i = 1; i<=CF_count; ++i)
    {
        PDU::ConsecutiveFrame* consecutiveFrame = reinterpret_cast<PDU::ConsecutiveFrame*>(frameset[i].data());
        fillConsecutiveFrame(consecutiveFrame, NAD, i, payload, bytesWritten);
    }

    return frameset;
}

void LinTransportLayer::fillSingleFrame(
    LinTransportLayer::PDU::SingleFrame* singleFrame,
    const uint8_t* NAD,
    const std::vector<uint8_t> &payload
){
    // NAD
    singleFrame->NAD = *NAD;

    std::size_t len = payload.size();
    // High Nibble: SingleFrame
    // Low Nibble : LEN (= max 6)
    singleFrame->PCI_LEN = PDU::PCI::SF_Type | static_cast<uint8_t>(len & 0xFF);

    // Data [0..n]
    std::copy(
        payload.begin(),
        payload.begin() + payload.size(),
        singleFrame->data
    );

    // Fill Bytes
    if (payload.size() < 6)
    {
        std::fill(
            singleFrame->data + payload.size(),
            singleFrame->data + 6,
            0xFF
        );
    }
}

void LinTransportLayer::fillFirstFrame(
    LinTransportLayer::PDU::FirstFrame* firstFrame,
    const uint8_t* NAD,
    const std::vector<uint8_t> &payload,
    int &bytesWritten)
{
    // NAD
    firstFrame->NAD = *NAD;

    std::size_t len = payload.size();
    // High Nibble: FirstFrame
    // Low Nibble: LEN / 256
    firstFrame->PCI_LEN256 = PDU::PCI::FF_Type | static_cast<uint8_t>(len >> 8);
    firstFrame->LEN = static_cast<uint8_t>(len & 0xFF);

    // Data [0..n]
    std::copy(
        payload.begin(),
        payload.begin() + sizeof(PDU::FirstFrame::data),
        firstFrame->data
    );
    bytesWritten += sizeof(PDU::FirstFrame::data);

    // First Frame never contains fill bytes
}

void LinTransportLayer::fillConsecutiveFrame(
    LinTransportLayer::PDU::ConsecutiveFrame* consecutiveFrame,
    const uint8_t* NAD,
    const uint8_t sequenceNumber,
    const std::vector<uint8_t> &payload, int &bytesWritten
){
    // NAD
    consecutiveFrame->NAD = *NAD;
    // High Nibble: Consguetive Frame
    // Low Nibble: Framecounter % 0x0F
    consecutiveFrame->PCI_SN = PDU::PCI::CF_Type | (sequenceNumber & PDU::MASK_PCI_SN);
    // Data [0..n]
    auto bytesToCopy = std::min(sizeof(PDU::ConsecutiveFrame::data), payload.size() - bytesWritten);
    std::copy(
        payload.begin() + bytesWritten,
        payload.begin() + bytesWritten + bytesToCopy,
        consecutiveFrame->data
    );
    bytesWritten += bytesToCopy;

    // Fill Bytes occure only in the last frame
    if (bytesToCopy < sizeof(PDU::ConsecutiveFrame::data))
    {
        std::fill(
            std::begin(consecutiveFrame->data) + bytesToCopy,
            std::end(consecutiveFrame->data),
            0xFF
        );
    }
}

/// @brief Start a PDU SlaveRequest
/// @param NAD Node Adress (via pointer), wildcard will be replaced by received NAD
/// @return payload
std::optional<std::vector<uint8_t>> LinTransportLayer::readPduResponse(uint8_t* NAD, const uint8_t newNAD)
{
    uint8_t acceptedNAD = *NAD;
    constexpr auto isSingleFrame_or_isFirstFrame = 0;
    uint8_t frameCounter = isSingleFrame_or_isFirstFrame;
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

        static_assert(offsetof(PDU::SingleFrame, PCI_LEN) == offsetof(PDU::FirstFrame, PCI_LEN256),
            "Data position of SingleFrame::PCI_LEN does not match FirstFrame::PCI_LEN256");
        static_assert(offsetof(PDU::SingleFrame, PCI_LEN) == offsetof(PDU::ConsecutiveFrame, PCI_SN),
            "Data position of SingleFrame::PCI_LEN does not match PDU::ConsecutiveFrame::PCI_SN");

        // treat as SingleFrame to get FrameType
        PDU::SingleFrame* singleFrame = reinterpret_cast<PDU::SingleFrame*>(rxFrame->data());
        uint8_t rxNAD = singleFrame->NAD;
        uint8_t frameType = singleFrame->PCI_LEN & PDU::MASK_PCI_TYPE;

        if (frameCounter == isSingleFrame_or_isFirstFrame)
        {
            /// NAD will be replaced...
            if ((frameCounter == isSingleFrame_or_isFirstFrame) &&
                ((acceptedNAD == PDU::NAD::BROADCAST) || // on Wildcard
                (newNAD == rxNAD)))                     // on NAD Change my config requect
            {
                // Boradcast or Cmd "Conditional Change of NAD" was successfull
                acceptedNAD = rxNAD;
            }

            if (acceptedNAD != rxNAD)
            {
                // unexpected NAD: ignore Frame
                continue;
            }

            // test for type: single frame
            if (PDU::PCI::SF_Type == frameType) {
                PDU::SingleFrame* singleFrame = reinterpret_cast<PDU::SingleFrame *>(rxFrame.value().data());
                readSingleFrame(singleFrame, payload);
                break; // success
            }

            if (PDU::PCI::FF_Type == frameType)
            {
                PDU::FirstFrame* firstFrame = reinterpret_cast<PDU::FirstFrame*>(rxFrame.value().data());
                if (!readFirstFrame(firstFrame, payload, announcedBytes))
                {
                    // STRICT: when announcedBytes is less than 7 bytes, frame shall be ignored
                    acceptedNAD = *NAD; // revert in case of wildcard
                    continue;
                }

                frameCounter++;
                timeout = millis() + timeout_DtlSlaveResponse_per_frame;
                continue;
            }

            // STRICT: unexpected frame type shall be ignored
            acceptedNAD = *NAD; // revert in case of wildcard

        } else {
            // frameCounter == isConsecutiveFrame
            // sequence of CF started -> error handling now changed
            if (acceptedNAD != rxNAD)
            {
                // STRICT: mismatch with received NAT of FirstFrame --> abort
                return {};
            }

            if (PDU::PCI::CF_Type != frameType) {
                // STRICT: unexpected frame type --> abort
                return {};
            }

            PDU::ConsecutiveFrame* consecutiveFrame = reinterpret_cast<PDU::ConsecutiveFrame*>(rxFrame.value().data());
            if (!readConsecutiveFrame(consecutiveFrame, payload, announcedBytes, frameCounter))
            {
                // STRICT: unexpected sequence number received --> abort
                return {};
            }

            frameCounter++;
            timeout += timeout_DtlSlaveResponse_per_frame;
            if (announcedBytes == payload.size())
            {
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
    if ((*NAD == PDU::NAD::BROADCAST) || (newNAD != 0)) {
        *NAD = acceptedNAD;
    }
    return payload;
}

void LinTransportLayer::readSingleFrame(PDU::SingleFrame* singleFrame, std::vector<uint8_t> &payload)
{
    size_t announcedBytes = (singleFrame->PCI_LEN & PDU::MASK_PCI_LEN);
    // STRICT: when announcedBytes is greater than 6 bytes, frame shall be ignored
    // however, single frame is limited to 6 data bytes...

    payload.insert(
        payload.end(),
        singleFrame->data,
        singleFrame->data + announcedBytes
    );
}

bool LinTransportLayer::readFirstFrame(PDU::FirstFrame* firstFrame, std::vector<uint8_t> &payload, size_t &announcedBytes)
{
    announcedBytes = ((firstFrame->PCI_LEN256 & PDU::MASK_PCI_LEN) << 8) | firstFrame->LEN;
    if (announcedBytes < 7) {
        // STRICT: when announcedBytes is less than 7 bytes, frame shall be ignored
        return false;
    }

    // STRICT: reception of segmented message shall not start, wenn buffer payload is to small.
    payload.reserve(announcedBytes);
    payload.insert(
        payload.end(),
        firstFrame->data,
        firstFrame->data + sizeof(PDU::FirstFrame::data)
    );
    // since FF_DataLen is smaller than SF_DataLen, FF will never include FillBytes

    return true;
}

bool LinTransportLayer::readConsecutiveFrame(PDU::ConsecutiveFrame* consecutiveFrame, std::vector<uint8_t> &payload, size_t &announcedBytes, int frameCounter)
{
    auto expectedSequenceNumber = frameCounter & PDU::MASK_PCI_SN;
    auto rxSequenceNumber = consecutiveFrame->PCI_SN & PDU::MASK_PCI_SN;
    if (expectedSequenceNumber != rxSequenceNumber) {
        // STRICT: unexpected sequence number received --> abort
        return false;
    }
    auto bytesToReceive = announcedBytes - payload.size();
    auto bytesInFrame = std::min(bytesToReceive, sizeof(PDU::ConsecutiveFrame::data));
    payload.insert(
        payload.end(),
        consecutiveFrame->data,
        consecutiveFrame->data + bytesInFrame
    );
    // fillbytes are not verified (0xFF) and ignored

    return true;
}
