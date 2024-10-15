#include <unity.h>
#include "LinNodeConfig.hpp"
#include "mock_HardwareSerial.h"
#include "mock_DebugStream.hpp"

mock_DebugStream debugStream;

mock_HardwareSerial* linDriver;
LinNodeConfig* linNodeConfig;

constexpr uint8_t upperByte(uint16_t input)
{
    return static_cast<uint8_t>(input >> 8);
}

constexpr uint8_t lowerByte(uint16_t input)
{
    return static_cast<uint8_t>(input & 0xFF);
}

void setUp()
{
    linDriver = new mock_HardwareSerial(0);
    linDriver->mock_loopback = true;
    linDriver->begin(19200, SERIAL_8N1);

    linNodeConfig = new LinNodeConfig(*linDriver, debugStream, 1);
}

void tearDown()
{
    delete linNodeConfig;

    linDriver->end();
    delete linDriver;
}

void test_lin_wakeup() {
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    std::vector<uint8_t> bus_transmitted = {
        0x00
    };

    linNodeConfig->requestWakeup();

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

void test_lin_sleep() {
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    std::vector<uint8_t> bus_transmitted = {
        0x00, 0x55, 0x3c,
        0x00,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0x00
    };

    linNodeConfig->requestGoToSleep();

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

/// @brief Node Identification: Read by identifier (Spec LIN 2.2A, Page 78, Chap. 4.2.6.1)
void test_lin_getID() {
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    constexpr uint8_t request_NAD = 0x7F; // Wildcard
    constexpr uint16_t request_SupplierId = 0x7FFF; // Wildcard
    constexpr uint16_t request_FunctionId = 0x3FFF; // Wildcard

    std::vector<uint8_t> bus_transmitted = {
    // Master
        0x00, 0x55, 0x3C, // Frame: Master Request
        request_NAD, // NAD
        0x06, // PCI: Single frame, 6 Byte
        0xB2, // SID: Read by ID
        0x00, // Identifier: Lin Product Identification
        lowerByte(request_SupplierId), upperByte(request_SupplierId), // Supplier ID LSB, MSB
        lowerByte(request_FunctionId), upperByte(request_FunctionId), // Function ID LSB, MSB
        0x09, // Frame: Checksum
    // Slave
        0x00, 0x55, 0x7d // Request for Slave Response
    };

    // Slave Response
    constexpr uint8_t response_NAD = 0x0A;
    constexpr uint16_t response_supplierId = 0x2E06;
    constexpr uint16_t response_functionId = 0x1080;
    constexpr uint8_t response_variant = 0x56;
    std::vector<uint8_t> response {
        response_NAD, // NAD
        0x06, // PCI: Single Frame, 6 Bytes
        0xF2, // RSID
        lowerByte(response_supplierId), upperByte(response_supplierId), // supplier ID, LSB MSB
        lowerByte(response_functionId), upperByte(response_functionId), // function ID, LSB MSB
        response_variant, // variant
        0xE1 // Frame Checksum
    };
    linDriver->mock_Input(response);

    uint8_t NAD = request_NAD;
    uint16_t supplierId = request_SupplierId;
    uint16_t functionId = request_FunctionId;
    uint8_t variant = 0;
    bool result = linNodeConfig->readProductId(NAD, supplierId, functionId, variant);

    TEST_ASSERT_TRUE(result);

    TEST_ASSERT_EQUAL(response_NAD, NAD);
    TEST_ASSERT_EQUAL(response_supplierId, supplierId);
    TEST_ASSERT_EQUAL(response_functionId, functionId);
    TEST_ASSERT_EQUAL(response_variant, variant);

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

/// @brief Node Identification: Read by identifier (Spec LIN 2.2A, Page 78, Chap. 4.2.6.1)
void test_lin_serialNumber()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    constexpr uint8_t request_NAD = 0x7F; // wildcard
    constexpr uint16_t request_SupplierId = 0x7FFF; // wildcard
    constexpr uint16_t request_FunctionId = 0x3FFF; // wildcard

    std::vector<uint8_t> bus_transmitted = {
    // Master
        0x00, 0x55, 0x3C, // Frame Head: Master Request
        request_NAD, // NAD Wildcard
        0x06, // PID: Single Frame, 6 Byte
        0xB2, // SID: Read by identifier
        0x00, // Identifier: Serial number
        lowerByte(request_SupplierId), upperByte(request_SupplierId), // Supplier ID LSB, MSB
        lowerByte(request_FunctionId), upperByte(request_FunctionId), // Function ID LSB, MSB
        0x09, // Frame: Checksum

        0x00, 0x55, 0x7D // Frame Head: Slave Response
    };

    // Slave Response
    constexpr uint8_t response_NAD = 0x0A;
    constexpr uint32_t response_SN = 0x76543210;
    std::vector<uint8_t> response {
        response_NAD, // <-- according to spec initial NAD will be used
        0x05, // PCI: Single Frame, 5 Byte
        0xF2, // RSID
        0x10, 0x32, 0x54, 0x76, // Srial Number LSB...MSB
        0xFF, // unused
        0xF0 // Frame: Checksum
    };
    linDriver->mock_Input(response);
    
    uint8_t NAD = request_NAD;
    uint16_t supplierId = request_SupplierId;
    uint16_t functionId = request_FunctionId;
    auto result = linNodeConfig->readSerialNumber(NAD, supplierId, functionId);

    TEST_ASSERT_EQUAL(response_NAD, NAD);  // <-- answer will follow on old NAD

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(response_SN, result.value());

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

/// @brief Node Configuration Service: Assign NAD (Spec LIN 2.2A, Page 74, Chap. 4.2.5.1)
void test_lin_assignNAD_ok()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    constexpr uint8_t request_NAD = 0x7F; // wildcard
    constexpr uint16_t request_SupplierId = 0x7FFF; // wildcard
    constexpr uint16_t request_FunctionId = 0x3FFF; // wildcard
    constexpr uint8_t request_NAD_new = 0x0B;

    std::vector<uint8_t> bus_transmitted = {
    // Master
        0x00, 0x55, 0x3C, // Frame Head: Master Request
        request_NAD, // NAD Wildcard
        0x06, // PCI: 6 Byte
        0xB0, // SID: Assign NAD
        lowerByte(request_SupplierId), upperByte(request_SupplierId), // Supplier ID LSB, MSB
        lowerByte(request_FunctionId), upperByte(request_FunctionId), // Function ID LSB, MSB
        request_NAD_new, // New NAD
        0x00, // Frame Checksum
    // Slave
        0x00, 0x55, 0x7D // Frame Head: Slave Response
    };

    // Slave Response
    std::vector<uint8_t> response {
        request_NAD, // initial NAD <-- according to spec initial NAD will be used
        0x01,  // PCI: Single Frame, 1 Byte
        0xF0, // RSID
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 5x unused
        0x8E // Frame Checksum
    };
    linDriver->mock_Input(response);

    uint8_t NAD = request_NAD;
    bool result = linNodeConfig->assignNAD(NAD, request_SupplierId, request_FunctionId, request_NAD_new);

    TEST_ASSERT_TRUE(result);

    TEST_ASSERT_EQUAL(request_NAD, NAD);  // <-- answer will follow on old NAD

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

/// @brief Node Configuration Service: Conditional change NAD (Spec LIN 2.2A, Page 75, Chap. 4.2.5.2)
void test_lin_conditionalChangeNAD_ok()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    constexpr uint8_t request_NAD = 0x1A;
    constexpr uint8_t request_id = 0x01;
    constexpr uint8_t request_byte = 0x03;
    constexpr uint8_t request_invert = 0xFF;
    constexpr uint8_t request_mask = 0x01;
    constexpr uint8_t request_NAD_new = 0x1B;

    std::vector<uint8_t> bus_transmitted = {
    // Master
        0x00, 0x55, 0x3C, // Frame Head: Master Request
        request_NAD, // NAD (=master request)
        0x06, // PCI: Single Frame, 6 Byte
        0xB3, // SID: Conditional change NAD
        request_id, // Id
        request_byte, // Byte
        request_mask, // Mask
        request_invert, // Invert
        request_NAD_new, // new NAD
        0x0C, // Frame Checksum
    //Slave
        0x00, 0x55, 0x7D // Frame Head: Slave Response
    };

    // Slave Response
    std::vector<uint8_t> response {
        request_NAD_new, // NAD <-- according to spec: new NAD will be used
        0x01,  // PCI: Single Frame, 1 Byte
        0xF3, // RSID
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 5x unused
        0xEF // Frame Checksum
    };
    linDriver->mock_Input(response);

    uint8_t NAD = request_NAD;
    bool result = linNodeConfig->conditionalChangeNAD(NAD, request_id, request_byte, request_invert, request_mask, request_NAD_new);

    TEST_ASSERT_TRUE(result);

    TEST_ASSERT_EQUAL(request_NAD_new, NAD);  // <-- answer will follow on new NAD

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

/// @brief Node Configuration Service: Save Configuration (Spec LIN 2.2A, Page 76, Chap. 4.2.5.4)
void test_lin_saveConfig_ok()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    constexpr uint8_t request_NAD = 0x66; // wildcard

    std::vector<uint8_t> bus_transmitted = {
    // Master
        0x00, 0x55, 0x3C, // Frame Head: Master Request
        request_NAD, // NAD
        0x01, // PCI: Single Frame, 1 Byte
        0xB6, // SID: Save Config
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 5x unused
        0xE1, // Frame Checksum
    //Slave
        0x00, 0x55, 0x7D, // Frame Head: Slave Response
    };

    // Slave Response
    std::vector<uint8_t> response {
        request_NAD, // NAD
        0x01,  // Single Frame, 1 Byte
        0xF6, // RSID = Save Config
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 5x unused
        0xA1 // Frame Checksum
    };
    linDriver->mock_Input(response);

    uint8_t NAD = request_NAD;
    bool result = linNodeConfig->saveConfig(NAD);

    TEST_ASSERT_TRUE(result);

    TEST_ASSERT_EQUAL(request_NAD, NAD);  // <-- answer will follow on old NAD

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

/// @brief Node Configuration Service: Assign frame ID range (Spec LIN 2.2A, Page 77, Chap. 4.2.5.5)
void test_lin_AssignFrameIdRange_ok()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    constexpr uint8_t request_NAD = 0x66; // wildcard
    constexpr uint8_t request_start = 1;
    constexpr uint8_t request_PID0 = 0x80;
    constexpr uint8_t request_PID1 = 0xC1;
    constexpr uint8_t request_PID2 = 0x42;
    constexpr uint8_t request_PID3 = 0x00; // unused

    std::vector<uint8_t> bus_transmitted = {
    // Master
        0x00, 0x55, 0x3C, // Frame Head: Master Request
        request_NAD, // NAD
        0x06, // PCI: Single Frame, 6 Byte
        0xB7, // SID: Assign frame ID Range
        request_start, // start
        request_PID0, // PID(index)
        request_PID1, // PID(index+1)
        request_PID2, // PID(index+2)
        request_PID3, // PID(index+2)
        0x56, // chksum
    // Slave
        0x00, 0x55, 0x7D   // Frame Head: Slave Request
    };

    // Slave Response
    std::vector<uint8_t> response {
        request_NAD, // NAD
        0x01,  // PID: Single Frame, 1 Byte
        0xF7, // RSID: Assign Frame ID Range
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 5x unused
        0xA0 // Frame Checksum
    };
    linDriver->mock_Input(response);

    uint8_t NAD = request_NAD;
    bool result = linNodeConfig->assignFrameIdRange(NAD, request_start, request_PID0, request_PID1, request_PID2, request_PID3);

    TEST_ASSERT_TRUE(result);

    TEST_ASSERT_EQUAL(request_NAD, NAD);  // <-- answer will follow on old NAD

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

int main() {
    UNITY_BEGIN();
    
    RUN_TEST(test_lin_wakeup);
    RUN_TEST(test_lin_sleep);
    RUN_TEST(test_lin_getID);
    RUN_TEST(test_lin_serialNumber);
    RUN_TEST(test_lin_assignNAD_ok);
    RUN_TEST(test_lin_conditionalChangeNAD_ok);
    RUN_TEST(test_lin_saveConfig_ok);
    RUN_TEST(test_lin_AssignFrameIdRange_ok);
   
    return UNITY_END();
}
