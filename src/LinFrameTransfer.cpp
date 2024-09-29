// LinFrameTransfer.cpp
//
// Provides a Hardware LIN Interface for frame transmission
//
// LIN Specification 2.2A
// Source https://www.lin-cia.org/fileadmin/microsites/lin-cia.org/resources/documents/LIN_2.2A.pdf
// modification required but possible for legacy support

#include "LinFrameTransfer.hpp"

#ifdef UNIT_TEST
    #include "../test/mock_Arduino.h"
    #include "../test/mock_millis.h"
    #include "../test/mock_delay.h"
#else
    #include <Arduino.h>
#endif

#include <optional>
#include <vector>

enum class debugLevel {
    none = 0,
    error,
    verbose
};

constexpr debugLevel debug = debugLevel::verbose;

constexpr auto timeout_ReadFrame = 50; // ms

class FrameReader {
public:
    enum class State {
        WaitForBreak,
        WaitForSync,
        WaitForPID,
        WaitForData,
        WaitForChkSum,
        FrameComplete
    };

    using ChecksumFunction = uint8_t(*)(const uint8_t protectedID, const std::vector<uint8_t>& data);

private:
    State state;
    uint8_t protectedID;
    uint8_t len; // max. value does also protect of overflow
    std::vector<uint8_t> rxData;
    ChecksumFunction getChecksum;
    Stream& debugStream;

public:
    FrameReader(
        const uint8_t PID,
        uint8_t expectedDataLength, // max. value does also protect of overflow
        ChecksumFunction checksumFunc,
        Stream& debugStream
    ):
        protectedID(PID),
        len(expectedDataLength),
        state(State::WaitForBreak),
        getChecksum(checksumFunc),
        debugStream(debugStream)
    {
        rxData.reserve(expectedDataLength);
    }

    void reset()
    {
        if constexpr (debug >= debugLevel::verbose) {
            debugStream.println("FrameReader: Reset");
        }

        state = State::WaitForBreak;
        rxData.clear();
    }

    bool hasHead()
    {
        return state >= State::WaitForData;
    }

    bool isFinish()
    {
        return state == State::FrameComplete;
    }

    std::vector<uint8_t> getData()
    {
        return rxData;
    }

    void processByte(const uint8_t newByte)
    {
        switch (state) {
        case State::WaitForBreak:
            if (newByte == LinFrameTransfer::BREAK_FIELD) {
                state = State::WaitForSync;
            }
            break;

        case State::WaitForSync:
            if (newByte == LinFrameTransfer::SYNC_FIELD)
            {
                state = State::WaitForPID;
            } else {
                reset();
            }
            break;

        case State::WaitForPID:
            if (newByte == protectedID)
            {
                state = State::WaitForData;
            } else {
                reset();
            }
            break;

        case State::WaitForData:
            rxData.push_back(newByte);
            if (rxData.size() >= len) // does also protect of overflow
            {
                state = State::WaitForChkSum;
            }
            break;
        
        case State::WaitForChkSum:
            if (!getChecksum) {
                if constexpr (debug >= debugLevel::error) {
                    debugStream.println("FrameReader: Missing checksum function");
                }
                reset();
            }
            uint8_t expectedChecksum = getChecksum(protectedID, rxData);
            bool checksum_isValid = (newByte == expectedChecksum);
            if (checksum_isValid)
            {
                // success: frame completely received
                state = State::FrameComplete;
                if constexpr (debug >= debugLevel::verbose) {
                    printRawFrame(protectedID, rxData, newByte, expectedChecksum);
                    debugStream.println("FrameReader: Frame valid");
                }
            } else {
                // checksum missmatch
                if constexpr (debug >= debugLevel::error) {
                    printRawFrame(protectedID, rxData, newByte, expectedChecksum);
                }
                reset();
            }
        }
    }

    void printRawFrame(const uint8_t protectedID, std::vector<uint8_t>& data, uint8_t rxChecksum, uint8_t expectedChecksum)
    {
        debugStream.print(" --- FID ");
        debugStream.print(protectedID & LinFrameTransfer::FRAME_ID_MASK, HEX);
        debugStream.print("h        = 55|");
        debugStream.print(protectedID, HEX);
        debugStream.print("|");

        for (uint8_t byte: data)
        {
            debugStream.print(byte, HEX);
            debugStream.print(".");
        }
        debugStream.print("\b|");
        debugStream.print(rxChecksum, HEX);

        if (rxChecksum != expectedChecksum)
        {
            debugStream.print(" Checksum mismatch, expected ");
            debugStream.print(expectedChecksum, HEX);
        }

        debugStream.println();
    }
};

// ------------------------------------

/// @brief write a LIN2.0 frame to the lin-bus. no request for any node response on the bus.
/// @details write LIN Frame (Break, Synk, PID, Data, Checksum) to the Bus, and hope some node will recognize this
/// - Checksum Calculations regarding LIN 2.x
/// - The data of this frame is 'expectedDataLength' long and incuded in the LinFrameTransfer::LinMessage[] array
/// - use writeReadback_verify or writeReadback_throw to control readback and error handling 
/// @param FrameID ID of frame (will be converted to protected ID)
/// @param expectedDataLength count of data within the LinMessage array (containing only the data) should be transmitted
bool LinFrameTransfer::writeFrame(const uint8_t frameID, const std::vector<uint8_t>& data)
{
    if (data.size() == 0) {
        return writeEmptyFrame(frameID);
    }

    const uint8_t protectedID { getProtectedID(frameID) };

    // TX Full Frame
    writeFrameHead(protectedID);
    for (const uint8_t& byte : data) {
        driver.write(byte);
    }

    uint8_t chksum = getChecksumLin2x(protectedID, data);
    driver.write(chksum);

    // ensure request is avaliable for receiver
    driver.flush();

    // Do readback written bytes and verify
    if constexpr (writeReadback_verify) {
        // RX copy of our TX (Full Frame)
        auto result = receiveFrameExtractData(protectedID, data.size());

        // verify if frame was received
        if (!result) {
            // failed, caused by timeout (debug was printed)
            return false;
        }

        // verify rx data == tx data
        if (result.value() != data) {
            if constexpr (debug >= debugLevel::error) {
                debugStream.print(" writeFrame, readback failed");
            }
            return false;
        }
    }

    // discard written bytes
    if constexpr (writeReadback_throw)
    {
        // remove bytes from buffer (head + data + checksum)
        int frameBytes = 3 + data.size() + 1;
        for (auto i=0; i<frameBytes; ++i) {
            driver.read();
        }
    }

    return true;
}

bool LinFrameTransfer::writeEmptyFrame(const uint8_t frameID)
{
    const uint8_t protectedID { getProtectedID(frameID) };

    // TX Frame Head
    writeFrameHead(protectedID);
    // no data
    // no checksum

    // ensure request is avaliable for receiver
    driver.flush();

    // Do readback written bytes and verify
    if constexpr (writeReadback_verify)
    {
        auto result = receiveFrameHead(protectedID);

        // verify if frame was received
        if (!result) {
            // failed, caused by timeout (debug was printed)
            return false;
        }
    }

    // discard written bytes
    if constexpr (writeReadback_throw)
    {
        // remove bytes from buffer (head + data + checksum)
        int frameBytes = 3;
        for (auto i=0; i<frameBytes; ++i) {
            driver.read();
        }
    }

    return true;
}

/// @brief reads data from a lin node by requesting a specific FrameID
/// @details Request data and read response from bus device
/// - Receives precisely the expected number of byts
/// Verify Checksum according to LIN 2.0 rules
/// @param FrameID FrameID (will be converted to ProtectedID)
/// @param expectedDataLength Length of data bytes [0..8] (default=8), only success if matched
/// @returns rx data on success, otherwise std::nullopt
std::optional<std::vector<uint8_t>> LinFrameTransfer::readFrame(const uint8_t frameID, uint8_t expectedDataLength)
{
    const uint8_t protectedID { getProtectedID(frameID) };

    // TX only Frame Head
    writeFrameHead(protectedID);

    // ensure request is avaliable for receiver
    driver.flush();

    // RX loopback of our TX AND response from receiver
    auto result = receiveFrameExtractData(protectedID, expectedDataLength);

    return result;
}

void LinFrameTransfer::writeFrameHead(uint8_t protectedID)
{
    writeBreak();
    driver.write(SYNC_FIELD);
    driver.write(protectedID);
}

/// @brief Send a Break for introduction of a Frame
/// @returns if the 0x00 has been send
size_t LinFrameTransfer::writeBreak()
{
    // Goal: Brake Length (dominant + delimiter) = min 14 Tbit (see 2.8.1)
    // This is done by sending a Byte (0x00) + Stop Bit by using half baud rate

    driver.flush();
    // configure to half baudrate --> a t_bit will be doubled
    driver.updateBaudRate(baud >> 1);
    // write 0x00, including Stop-Bit (=1),
    // qualifies when writing in slow motion like a Break in normal speed
    size_t result = driver.write(BREAK_FIELD);
    // ensure this was send
    driver.flush();
    // restore normal speed
    driver.updateBaudRate(baud);
    return result;
}

/// get Protected ID by calculating parity bits and combine with Frame ID
/// @param frameID (0x00-0x3F) to be converted (avaliable parity bits will be overwritten)
/// @return Protected ID
constexpr uint8_t LinFrameTransfer::getProtectedID(const uint8_t frameID)
{
    // calc Parity Bit 0
    uint8_t p0 = bitRead(frameID, 0) ^ bitRead(frameID, 1) ^ bitRead(frameID, 2) ^ bitRead(frameID, 4);
    // calc Parity Bit 1
    uint8_t p1 = ~(bitRead(frameID, 1) ^ bitRead(frameID, 3) ^ bitRead(frameID, 4) ^ bitRead(frameID, 5));
    // combine bits to protected ID
    // 0..5 id is limited between 0x00..0x3F
    // 6    parity bit 0
    // 7    parity bit 1
    return ((p1 << 7) | (p0 << 6) | (frameID & FRAME_ID_MASK));
}

/// @brief reads a full frame from bus.
/// discard all data, until break, sync, PID, data, chksum valid is OR timeout occurs
/// @param protectedID expected ProtectedID; only success if matched
/// @param expectedDataLength expected lenght of data; only success if matched
/// @return vector of received data (may 0 byte) OR fail
std::optional<std::vector<uint8_t>> LinFrameTransfer::receiveFrameExtractData(uint8_t protectedID, size_t expectedDataLength)
{
    FrameReader frameReader(protectedID, expectedDataLength, getChecksumLin2x, debugStream);

    auto timeout_stop = millis() + timeout_ReadFrame;
    while ((millis() < timeout_stop) && (!frameReader.isFinish()))
    {
        // ensure timeout is checked, while no data are avaliable
        if (!driver.available())
        {
            continue;
        }

        // get byte, verify and use (or may discard)
        uint8_t newByte = driver.read();
        frameReader.processByte(newByte);
    }

    if (!frameReader.isFinish())
    {
        // rx of valid frame failed!
        if constexpr (debug >= debugLevel::error) {
            debugStream.print("timeout: no valid frame received\n");
        }
        return {};
    }

    return frameReader.getData();
}

/// @brief reads a frame head - no frame response - from bus.
/// discard all data, until break, sync, PID OR timeout occurs
/// @param protectedID expected ProtectedID; only success if matched
/// @return success
bool LinFrameTransfer::receiveFrameHead(uint8_t protectedID)
{
    FrameReader frameReader(protectedID, 0, nullptr, debugStream);

    auto timeout_stop = millis() + timeout_ReadFrame;
    while ((millis() < timeout_stop) && (!frameReader.hasHead()))
    {
        // ensure timeout is checked, while no data are avaliable
        if (!driver.available())
        {
            continue;
        }

        // get byte, verify and use (or may discard)
        uint8_t newByte = driver.read();
        frameReader.processByte(newByte);
    }

    if (!frameReader.hasHead())
    {
        // rx of valid frame failed!
        if constexpr (debug >= debugLevel::error) {
            debugStream.print("timeout: no valid frame head received\n");
        }
        return false;
    }

    return true;
}

/// @brief calculates Classic Checksum (Lin2.x) WITH ProtectedID except for FID >= Master Request
/// @param protectedID expected ProtectedID
/// @param data vector of n Data bytes
/// @return calculated checksum
uint8_t LinFrameTransfer::getChecksumLin2x(const uint8_t protectedID, const std::vector<uint8_t>& data)
{
    // REMARK: since FrameID 0x3E and 0x3F shall not be used (see 2.3.3.5)
    // we do not distinct here and use classic checksum (incorrect)
    if ((protectedID & FRAME_ID_MASK) >= FRAME_ID::MASTER_REQUEST)
    {
        // Classic Checksum (see 2.3.1.5)
        // FID 0x3C Master Request
        // FID 0x3D Slave Request
        return getChecksumEnhanced(0x00, data);
    }

    // Enhanced Checksum
    // FID 0x00..0x3B
    // FID 0x3E reserved, incorrect according to 2.3.1.5
    // FID 0x3F reserved, incorrect according to 2.3.1.5
    return getChecksumEnhanced(protectedID, data);
}

/// @brief calculates Classic Checksum (Lin1.3) WITH ProtectedID in ALL cases
/// @param protectedID expected ProtectedID
/// @param data vector of n Data bytes
/// @return calculated checksum
uint8_t LinFrameTransfer::getChecksumLin13(const uint8_t protectedID, const std::vector<uint8_t>& data)
{
    // Enhanced Checksum
    return getChecksumEnhanced(protectedID, data);
}

/// @brief calculates Classic Checksum (Lin1.x except Lin1.3) excluding ProtectedID
/// @param data vector of n Data bytes
/// @return calculated checksum
uint8_t LinFrameTransfer::getChecksumClassic(const std::vector<uint8_t>& data)
{
    // Classic Checksum
    return getChecksumEnhanced(0x00, data);
}

/// @brief Checksum calculation for LIN Frame WITH ProtectedID
/// @details
/// EnhancedChecksum considers ProtectedID
///     LIN 2.0 only for FrameID between 0x00..0x3B
///     LIN 2.0 uses for 0x3C and above ClassicChecksum for legacy (auto detected)
/// ClassicChecksum
///     LIN 1.x in general (use 'ProtectedID' = 0x00 to ensure that)
/// see LIN Specification  for details
///     2.8.3 Example of Checksum Calculation
/// @param protectedID initial Byte, set to 0x00, when calc Checksum for classic LIN Frame
/// @param data vector of n Data bytes
/// @returns calculated checksum
uint8_t LinFrameTransfer::getChecksumEnhanced(const uint8_t protectedID, const std::vector<uint8_t>& data)
{
    uint16_t sum { protectedID };

    // sum up all bytes (including carryover to the high byte)
    for (const uint8_t& byte : data)
    {
        sum += byte;
    }

    // extract low byte and add high byte (sum of carry over)
    sum = (sum & 0xFF) + (sum >> 8);
    // in case of additional carry over: add them too
    sum += (sum >> 8);

    // invert and cast result
    uint8_t result = static_cast<uint8_t>(~sum);


    // original method according to 2.3.1.5

    // // sum up all bytes (including carryover to the high byte)
    // for (const uint8_t& byte : data)
    // {
    //     sum += byte;
    //     sum = sum >= 256 ? sum-255 : sum;
    // }

    // // invert and cast result
    // uint8_t result = static_cast<uint8_t>(~sum);

    return result;
}
