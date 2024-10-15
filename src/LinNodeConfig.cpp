// LinNodeConfiguration.cpp
//
// Node configuration and identification
//
// Lin 2.2A specification
// https://www.lin-cia.org/fileadmin/microsites/lin-cia.org/resources/documents/LIN_2.2A.pdf

#include "LinNodeConfig.hpp"

#ifdef UNIT_TEST
    #include "../test/mock_Arduino.h"
    #include "../test/mock_millis.h"
    #include "../test/mock_delay.h"
#else
    #include <Arduino.h>
#endif

#include <optional>
#include <vector>

#include "LinFrameTransfer.hpp"
#include "LinPDU.hpp"

/// @brief send wakeup command by sending a bus dominant for 1.6ms (at 9600 Baud)
void LinNodeConfig::requestWakeup()
{
    // Lin Protocol Specification 2.1 Chapter 2.6.2 Wakeup
    // any node in a sleeping LIN cluster may request a wake up
    // hold bus line down for 250µs - 5ms (see 2.8.1)
    // 9600 Baud = 104µs per bit ==> half speed = 208µs per bit ==> 8 bits = 1.664ms
    // (slaves should respond on dominand signal longer 150µs and listen awake after max. 100ms)

    // ensure the tx buffer is empty
    driver.flush();
    // configure to half baudrate --> a t_bit will be doubled
    driver.updateBaudRate(baud >> 1);
    // write 0x00, including Stop-Bit (=1)
    // qualifies when writing in slow motion for a wake-up request
    driver.write(LinFrameTransfer::BREAK_FIELD);
    // ensure this is send
    driver.flush();
    // restore normal speed
    driver.updateBaudRate(baud);

    // give the bus some time to wake up (100-150ms)
    constexpr auto delay_after_Wakeup = 100; // 100-150ms, after 250ms slaves may request a second call
    delay(delay_after_Wakeup);
}

/// @brief Request bus cluster to go to sleep
void LinNodeConfig::requestGoToSleep()
{
    // https://www.lin-cia.org/fileadmin/microsites/lin-cia.org/resources/documents/LIN_2.2A.pdf
    // 2.6.3 Go To Sleep
    // Request from master to all nodes to go to sleep

    PDU cmdSleep = PDU::getSleepCmd();
    writeFrame(FRAME_ID::MASTER_REQUEST, cmdSleep.asVector());
}

std::optional<std::vector<uint8_t>> LinNodeConfig::readById(uint8_t &NAD, uint16_t supplierId, uint16_t functionId, uint8_t id)
{
    uint8_t SID = static_cast<uint8_t>(ServiceIdentifier::READ_BY_ID);
    std::vector<uint8_t> payload = {
        SID,
        id,
        (uint8_t)lowByte(supplierId),
        (uint8_t)highByte(supplierId),
        (uint8_t)lowByte(functionId),
        (uint8_t)highByte(functionId)
    };
    auto raw = writePDU(NAD, payload);

    if (!checkPayload_isValid(SID, raw))
    {
        return {};
    }

    std::vector<uint8_t> result;
    // leave out: data()[0]
    result.push_back(raw.value().data()[1]);
    result.push_back(raw.value().data()[2]);
    result.push_back(raw.value().data()[3]);
    result.push_back(raw.value().data()[4]);
    result.push_back(raw.value().data()[5]);

    return result;
}

/// @brief get SupplierID, FuncionID and VariantID from specific node (mandatory Function for all Nodes)
/// @details see LIN Spec 2.2A 4.2.1 LIN PRODUCT IDENTIFICATION
/// @param NAD Node Adress (may wildcard)
/// @param supplierId Supplier ID (may wildcard)
/// @param functionId Function ID (may wildcard)
/// @param variantId Variant of identical nodes
/// @return success
bool LinNodeConfig::readProductId(uint8_t &NAD, uint16_t &supplierId, uint16_t &functionId, uint8_t &variantId)
{
    uint8_t SID = static_cast<uint8_t>(ServiceIdentifier::READ_BY_ID);
    std::vector<uint8_t> payload = {
        SID,
        (uint8_t)CMD_Identifier::PRODUCT_ID,
        (uint8_t)lowByte(supplierId),
        (uint8_t)highByte(supplierId),
        (uint8_t)lowByte(functionId),
        (uint8_t)highByte(functionId)
    };
    auto raw = writePDU(NAD, payload);

    if (!checkPayload_isValid(SID, raw))
    {
        return false;
    }

    struct responseSid0ProductId
    {
        uint8_t RSID;
        uint8_t supplierId_LSB;
        uint8_t supplierId_MSB;
        uint8_t functionId_LSB;
        uint8_t functionId_MSB;
        uint8_t variantId;
    };

    responseSid0ProductId& re = *reinterpret_cast<responseSid0ProductId*>(raw.value().data());

    supplierId = re.supplierId_MSB << 8 | re.supplierId_LSB;
    functionId = re.functionId_MSB << 8 | re.functionId_LSB;
    variantId = re.variantId;

    return true;
}

/// @brief get Serial Number from specific node (optinal Function of Node)
/// @details see LIN Spec 2.2A 4.2.1 LIN PRODUCT IDENTIFICATION
/// @param NAD
/// @param supplierId
/// @param functionId
/// @param serialNumber
/// @return Serial number or fail
std::optional<uint32_t> LinNodeConfig::readSerialNumber(uint8_t &NAD, uint16_t supplierId, uint16_t functionId)
{
    uint8_t SID = static_cast<uint8_t>(ServiceIdentifier::READ_BY_ID);
    std::vector<uint8_t> payload = {
        SID,
        (uint8_t)CMD_Identifier::PRODUCT_ID,
        (uint8_t)lowByte(supplierId),
        (uint8_t)highByte(supplierId),
        (uint8_t)lowByte(functionId),
        (uint8_t)highByte(functionId)
    };
    auto raw = writePDU(NAD, payload);

    if (!checkPayload_isValid(SID, raw))
    {
        return {};
    }

    struct responseSid0ProductId
    {
        uint8_t RSID;
        uint8_t serialNumber_LSB;
        uint8_t serialNumber_LSB2;
        uint8_t serialNumber_LSB3;
        uint8_t serialNumber_MSB;
    };

    responseSid0ProductId& re = *reinterpret_cast<responseSid0ProductId*>(raw.value().data());

    /* no double check neccessary
        if (re.RSID != getRSID(SID))
            return false;
    */

    uint32_t serialNumber = re.serialNumber_MSB << 24 | re.serialNumber_LSB3 << 16 | re.serialNumber_LSB2 << 8 | re.serialNumber_LSB;

    return serialNumber;
}


/// @brief unconditional chance of NAD
/// @details LIN SPEC 2.2A 4.2.5.1 Assign NAD
/// @param NAD old NAD (or wildcard)
/// @param supplierId wildcard = 0x7FFF
/// @param functionId wildcard = 0x3FFF
/// @param newNAD new NAD
/// @return success
bool LinNodeConfig::assignNAD(uint8_t &NAD, uint16_t supplierId, uint16_t functionId, uint8_t newNAD)
{
    uint8_t SID = static_cast<uint8_t>(ServiceIdentifier::ASSIGN_NAD);
    std::vector<uint8_t> payload = {
        SID,
        (uint8_t)lowByte(supplierId),
        (uint8_t)highByte(supplierId),
        (uint8_t)lowByte(functionId),
        (uint8_t)highByte(functionId),
        (uint8_t)newNAD
    };
    auto raw = writePDU(NAD, payload);
    // Response on initial NAD

    if (!checkPayload_isValid(SID, raw))
    {
        return false;
    }

    // expected: all verified within writePDU() and checkPayload_isValid()
    // initial NAD (not the new one)
    // PCI = Single Frame, Length = 1
    // RSID is valid

    return true;
}

/// @brief conditional change of NAD (NEEDs FIX)
/// @details LIN SPEC 2.2A 4.2.5.2 Conditional change NAD
/// @param NAD old NAD
/// @param id 1. Get the identifier specified in Table 4.20 and selected by Id.
/// @param byte 2. Extract the data byte selected by Byte (Byte = 1 corresponds to the first byte, D1)
/// @param invert 3. Do a bitwise XOR with Invert
/// @param mask 4. Do a bitwise AND with Mask
/// @param newNAD 5. if the final result is zero change the NAD to newNAD
/// @return success
bool LinNodeConfig::conditionalChangeNAD(uint8_t &NAD, uint8_t id, uint8_t byte, uint8_t invert, uint8_t mask, uint8_t newNAD)
{
    uint8_t SID = static_cast<uint8_t>(ServiceIdentifier::CONDITIONAL_CHANGE);
    std::vector<uint8_t> payload = {
        SID,
        id,
        byte,
        mask,
        invert,
        newNAD
    };
    auto raw = writePDU(NAD, payload, newNAD);

// TODO: response will use the new NAD, not the initial one!
// FIX: response will use the new NAD, not the initial one!


    if (!checkPayload_isValid(SID, raw))
    {
        return false;
    }

    // expected: NOT verified within writePDU() and checkPayload_isValid()
    // new NAD (not the old one!!!)
    // PCI = Single Frame, Length = 1
    // RSID is valid

    return true;
}

// LIN SPEC 2.2A 4.2.5.3 Data dump
// - not implemented -
// std::vector<uint8_t> LinNodeConfig::dataDump(uint8_t* NAD, std::vector<uint8_t>)

/// @brief Saves the configuration of the LIN node.
/// @details LIN SPEC 2.2A 4.2.5.4 Save Configuration
/// Request the node to save its current configuration. The implementation follows the
/// LIN specification and ensures that the node retains its settings after a power cycle.
/// @param NAD Node Address (NAD) of the target
/// @return success
bool LinNodeConfig::saveConfig(uint8_t &NAD)
{
    uint8_t SID = static_cast<uint8_t>(ServiceIdentifier::SAVE_CONFIG);
    std::vector<uint8_t> payload = {
        SID
    };
    auto raw = writePDU(NAD, payload);

    if (!checkPayload_isValid(SID, raw))
    {
        return false;
    }

    return true;
}

/// @brief Assigns a range of frame IDs to a LIN node
/// @details LIN 2.2A Spec: 4.2.5.5 Assign frame ID range
/// @param NAD Node Address (NAD) of the target slave node
/// @param startIndex index specifying the start of the frame ID range
/// @param PID0 first protected identifier (PID) in the frame ID range
/// @param PID1 second protected identifier (PID) in the frame ID range
/// @param PID2 third protected identifier (PID) in the frame ID range
/// @param PID3 fourth protected identifier (PID) in the frame ID range
/// @return success
bool LinNodeConfig::assignFrameIdRange(uint8_t &NAD, uint8_t startIndex, uint8_t PID0, uint8_t PID1, uint8_t PID2, uint8_t PID3)
{
    uint8_t SID = static_cast<uint8_t>(ServiceIdentifier::ASSIGN_FRAME_IDENTIFIER_RANGE);
    std::vector<uint8_t> payload = {
        SID,
        startIndex,
        PID0,
        PID1,
        PID2,
        PID3
    };
    auto raw = writePDU(NAD, payload);

    if (!checkPayload_isValid(SID, raw))
    {
        return false;
    }

    // no double check of RSID neccessary
    return true;
}

/// @brief Check payload for expected SID or ErrorCode
/// @param SID expected SID for positive response
/// @param payload data to investigate
/// @return data are valid, expected SID was recognizes, no error code
bool LinNodeConfig::checkPayload_isValid(const uint8_t SID, const std::optional<std::vector<uint8_t>> &payload)
{
    uint8_t expectedRSID = getRSID(SID);

    if (!payload) {
        return false;
    }

    if ((!payload.value().empty()) && (expectedRSID == payload.value().front())) {
        // looks like valid response
        return true;
    }

    // now we are having trouble.

    struct Error {
        uint8_t magicnumber;
        uint8_t SID;
        NegativeResponseCode errorCode;
    };

    if (payload.value().size() < sizeof(Error)) {
        return false;
    }
    const Error* err = reinterpret_cast<const Error*>(payload.value().data());

    if (NEGATIVE_RESPONSE != err->magicnumber) {
        // unexpected: payload[0] is not equal to neither RSID nor 0x7F
        debugStream.print("writePDU failed: unexpected RSID");
        return {};
    }

    auto rxSID = err->SID;
    auto errorcode = err->errorCode;

    debugStream.print("writePDU failed: SID=0x");
    debugStream.print(rxSID, HEX);
    debugStream.print(" Error Code=0x");
    debugStream.print((uint8_t)(errorcode), HEX);
    debugStream.print(" = ");
    debugStream.print(get_NegativeResponseCode_String(errorcode));
    debugStream.println();

    return false;
}

/// @brief convert SID to RSID
/// @details 4.2.3.5 RSID = Response Service Identifier
/// @param SID input
/// @return RSID
constexpr uint8_t LinNodeConfig::getRSID(const uint8_t SID)
{
    // RSID = SID + 0x40
    constexpr uint8_t SID_TO_RSID_MASK = 0x40;
    return SID + SID_TO_RSID_MASK;
}

/// @brief get error string out of code
/// @param code Error code
/// @return short string, describes error
const char* LinNodeConfig::get_NegativeResponseCode_String(NegativeResponseCode code)
{
    auto it = codeMap.find(code);
    return (it != codeMap.end()) ? it->second : "Unknown NegativeResponseCode";
}
