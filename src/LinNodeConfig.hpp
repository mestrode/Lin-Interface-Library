// LinNodeConfiguration.hpp
//
// Node configuration and identification
//
// Lin 2.2A specification
// https://www.lin-cia.org/fileadmin/microsites/lin-cia.org/resources/documents/LIN_2.2A.pdf

#pragma once

#ifdef UNIT_TEST
    #include "../test/mock_HardwareSerial.h"
    using HardwareSerial = mock_HardwareSerial;
#else
    #include <Arduino.h>
#endif

#include <optional>
#include <vector>
#include <unordered_map>

#include "LinTransportLayer.hpp"

class LinNodeConfig : protected LinTransportLayer{
public:
    using LinTransportLayer::LinTransportLayer;

    void requestWakeup();
    void requestGoToSleep();

    std::optional<std::vector<uint8_t>> readById(uint8_t &NAD, uint16_t supplierId, uint16_t functionId, uint8_t id);
    bool readProductId(uint8_t &NAD, uint16_t &supplierId, uint16_t &functionId, uint8_t &variantId);
    std::optional<uint32_t> readSerialNumber(uint8_t &NAD, uint16_t supplierId, uint16_t functionId);

    bool assignNAD(uint8_t &NAD, uint16_t supplierId, uint16_t functionId, uint8_t newNAD);
    bool conditionalChangeNAD(uint8_t &NAD, uint8_t id, uint8_t byte, uint8_t invert, uint8_t mask, uint8_t newNAD);

    // not implemented:
    // std::optional<std::vector<uint8_t>> LinNodeConfig::dataDump(uint8_t* NAD, std::vector<uint8_t>)

    bool saveConfig(uint8_t &NAD);
    
    bool assignFrameIdRange(uint8_t &NAD, uint8_t startIndex, uint8_t PID0, uint8_t PID1, uint8_t PID2, uint8_t PID3);

protected:
    // 3.2.1.4 SID
    // 4.2.3.5 SID = Service Identifier
    // == first byte of payload within a PDU
    enum class ServiceIdentifier: uint8_t {
    // 0x00 - 0xAF = diagnostic
    // 0xB0 - 0xB7 = node configuration
        ASSIGN_NAD = 0xB0,                      // Assign NAD (Optional)
        ASSIGN_FRAME_ID = 0xB1,                 // Assign Frame Identifier (obsolete, see Lin 2.0)
        READ_BY_ID = 0xB2,                      // Read by Identifier (Mandatory)
        CONDITIONAL_CHANGE = 0xB3,              // Conditional Change NAD (Optional)
        DATA_DUMP = 0xB4,                       // Data Dump (Optional)
        RESERVED = 0xB5,                        // Assign NAD via SNPD (reserved for Node Position detection)
        SAVE_CONFIG = 0xB6,                     // Save Configuration (Optional)
        ASSIGN_FRAME_IDENTIFIER_RANGE = 0xB7    // Assign frame identifier range (Mandatory)
    // 0xB8 - 0xFE = diagnostic
    };

    // for READ_BY_ID = Read by Identifier
    enum class CMD_Identifier {
        PRODUCT_ID = 0,
        SERIAL_NUMBER = 1
        // 2-31 Reserved
        // 32-63 User defined
        // 64-255 Reserved
    };

    // DTL standard payload
    static constexpr uint8_t NEGATIVE_RESPONSE = 0x7F;

    // Negative Response Codes NRC
    enum NegativeResponseCode: uint8_t {
        GENERAL_REJECT = 0x10,
        SERVICE_NOT_SUPPORTED = 0x11,
        SUBFUNCTION_NOT_SUPPORTED = 0x12,
        INCORRECT_MSG_LENGTH_OR_INVALID_FORMAT = 0x13,
        RESPONSE_TOO_LONG = 0x14,
        BUSY_REPEAT_REQUEST = 0x21,
        CONDITIONS_NOT_CORRECT = 0x22,
        REQUEST_OUT_OF_RANGE = 0x31,
        SECURITY_ACCESS_DENIED = 0x33,
        INVALID_KEY = 0x35
    };

    const std::unordered_map<NegativeResponseCode, const char*> codeMap = {
        {NegativeResponseCode::GENERAL_REJECT, "NRC_GENERAL_REJECT"},
        {NegativeResponseCode::SERVICE_NOT_SUPPORTED, "NRC_SERVICE_NOT_SUPPORTED"},
        {NegativeResponseCode::SUBFUNCTION_NOT_SUPPORTED, "NRC_SUBFUNCTION_NOT_SUPPORTED"},
        {NegativeResponseCode::INCORRECT_MSG_LENGTH_OR_INVALID_FORMAT, "NRC_INCORRECT_MSG_LENGTH_OR_INVALID_FORMAT"},
        {NegativeResponseCode::RESPONSE_TOO_LONG, "NRC_RESPONSE_TOO_LONG"},
        {NegativeResponseCode::BUSY_REPEAT_REQUEST, "NRC_BUSY_REPEAT_REQUEST"},
        {NegativeResponseCode::CONDITIONS_NOT_CORRECT, "NRC_CONDITIONS_NOT_CORRECT"},
        {NegativeResponseCode::REQUEST_OUT_OF_RANGE, "NRC_REQUEST_OUT_OF_RANGE"},
        {NegativeResponseCode::SECURITY_ACCESS_DENIED, "NRC_SECURITY_ACCESS_DENIED"},
        {NegativeResponseCode::INVALID_KEY, "NRC_INVALID_KEY"}
    };

    bool checkPayload_isValid(const uint8_t SID, const std::optional<std::vector<uint8_t>> &payload);
    inline constexpr uint8_t getRSID(const uint8_t SID);
    const char* get_NegativeResponseCode_String(NegativeResponseCode code);
};
