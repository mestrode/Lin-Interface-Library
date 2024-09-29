// LinFrameTransfer.hpp
//
// Provides a Hardware LIN Interface for frame transmission
//
// LIN Specification 2.2A
// Source https://www.lin-cia.org/fileadmin/microsites/lin-cia.org/resources/documents/LIN_2.2A.pdf
// modification required but possible for legacy support

#pragma once

#ifdef UNIT_TEST
    #include "../test/mock_HardwareSerial.h"
    using HardwareSerial = mock_HardwareSerial;
    #include "../test/mock_Stream.h"
    using Stream = mock_Stream;
#else
    #include <Arduino.h>
#endif

#include <optional>
#include <vector>

class LinFrameTransfer : protected HardwareSerial {
public:
    // Do readback written bytes and verify
    static constexpr bool writeReadback_verify = true;
    // Do readback written bytes and ignore
    static constexpr bool writeReadback_throw = false;

    static constexpr uint8_t BREAK_FIELD = 0x00;
    static constexpr uint8_t SYNC_FIELD = 0x55;
    static constexpr uint8_t FRAME_ID_MASK = 0b0011'1111;

    enum FRAME_ID : const uint8_t {
    //    0-50 (0x00-0x3B) are used for normal Signal/data carrying frames.
    //    60 (0x3C) and 61 (0x3D) are used to carry diagnostic and configuration data.
        MASTER_REQUEST = 0x3C,
        SLAVE_REQUEST = 0x3D
    //    62 (0x3E) and 63 (0x3F) are reserved for future protocol enhancements.
    };


    LinFrameTransfer(uint8_t uart_nr, Stream &debug, int verbose = -1):
        HardwareSerial(uart_nr),
        debugStream(debug),
        verboseLevel(verbose)
    {}

    int verboseLevel;
    Stream &debugStream;

    unsigned long baud = 19200;
    int8_t rxPin = -1;
    int8_t txPin = -1;

    void begin();
    using HardwareSerial::end;

    bool writeFrame(const uint8_t frameID, const std::vector<uint8_t>& data);
    bool writeEmptyFrame(const uint8_t frameID);

    using HardwareSerial::available;
    std::optional<std::vector<uint8_t>> readFrame(const uint8_t frameID, uint8_t expectedDataLength = 8);

#ifdef UNIT_TEST
public:
    using HardwareSerial::mock_loopback;
    using HardwareSerial::mock_Input;
    using HardwareSerial::txBuffer;
#endif

protected:
    using HardwareSerial::updateBaudRate;
    using HardwareSerial::read;
    using HardwareSerial::flush;
    using HardwareSerial::write;

    inline void writeFrameHead(const uint8_t protectedID);
    inline size_t writeBreak();
    inline constexpr uint8_t getProtectedID(const uint8_t frameID);

    std::optional<std::vector<uint8_t>> receiveFrameExtractData(uint8_t protectedID, size_t expectedDataLength);
    bool receiveFrameHead(uint8_t protectedID);

    inline static uint8_t getChecksumLin2x(uint8_t protectedID, const std::vector<uint8_t>& data);
    inline static uint8_t getChecksumLin13(const uint8_t protectedID, const std::vector<uint8_t>& data);
    inline static uint8_t getChecksumClassic(const std::vector<uint8_t>& data);
    static uint8_t getChecksumEnhanced(const uint8_t protectedID, const std::vector<uint8_t>& data);
};
