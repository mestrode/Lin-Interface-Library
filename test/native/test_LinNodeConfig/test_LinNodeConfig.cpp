#include <unity.h>
#include "LinNodeConfig.hpp"
#include "mock_DebugStream.hpp"


mock_DebugStream debugStream;
LinNodeConfig* linNodeConfig;

// overload the + operator on std::vector
template <typename T>
std::vector<T> operator+(const std::vector<T>& v1, const std::vector<T>& v2) {
    // copy of the first vector
    std::vector<T> result(v1);
    // append on the second
    result.insert(result.end(), v2.begin(), v2.end());
    return result;
}

void setUp()
{
    linNodeConfig = new LinNodeConfig(0, debugStream, 1);
    linNodeConfig->mock_loopback = true;
    linNodeConfig->begin();
}

void tearDown()
{
    linNodeConfig->end();
    delete linNodeConfig;
}

void test_lin_wakeup() {
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    std::vector<uint8_t> bus_transmitted = {
        0x00
    };

    linNodeConfig->requestWakeup();

    std::vector<uint8_t>* txBuffer = &linNodeConfig->txBuffer;

    TEST_ASSERT_EQUAL(bus_transmitted.size(), txBuffer->size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), txBuffer->data(), bus_transmitted.size());
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

    std::vector<uint8_t>* txBuffer = &linNodeConfig->txBuffer;

    TEST_ASSERT_EQUAL(bus_transmitted.size(), txBuffer->size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), txBuffer->data(), bus_transmitted.size());
}

void test_lin_getID() {
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t NAD = {0x7F};
    uint16_t supplierId = 0x7FFF;
    uint16_t functionId = 0x3FFF;
    uint8_t variant = 0x00;

    std::vector<uint8_t> bus_transmitted = {
        0x00, 0x55, 0x3c,
        0x7F, // NAD Wildcard
        0x06, // 6 Byte
        0xB2, // SID
        0x00, // Cmd Identifier
        0xFF, 0x7F, // Supplier ID wildcard
        0xFF, 0x3F, // Function ID wildcard
        0x09, // chksum

        0x00, 0x55, 0x7d
    };

    // Slave Response
    struct Response {
        std::vector<uint8_t> head {
            0x0A, // NAD
            0x06  // Single Frame, 6 Bytes
        };
        std::vector<uint8_t> payload {
            0xF2, // RSID
            0x06, 0x2E, // supplier ID, LSB MSB
            0x80, 0x10, // function ID, LSB MSB
            0x56 // variant
        };
        uint8_t checksum {
            0xe1 // Frame Checksum
        };
    } response;
    linNodeConfig->mock_Input(response.head);
    linNodeConfig->mock_Input(response.payload);
    // no fillbytes required
    linNodeConfig->mock_Input(response.checksum);

    bool result = linNodeConfig->readProductId(&NAD, &supplierId, &functionId, &variant);

    TEST_ASSERT_EQUAL(0x0A, NAD);
    TEST_ASSERT_EQUAL(0x2E06, supplierId);
    TEST_ASSERT_EQUAL(0x1080, functionId);
    TEST_ASSERT_EQUAL(0x56, variant);

    std::vector<uint8_t>* txBuffer = &linNodeConfig->txBuffer;

    TEST_ASSERT_EQUAL(bus_transmitted.size(), txBuffer->size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), txBuffer->data(), bus_transmitted.size());
}

void test_lin_assignNAD_ok()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t NAD = 0x7F; // wildcard
    uint16_t supplierId = 0x7FFF; // wildcard
    uint16_t functionId = 0x3FFF; // wildcard
    static const uint8_t oldNAD = 0x0A;
    static const uint8_t newNAD = 0x0B;

    std::vector<uint8_t> bus_transmitted = {
    // Master
        0x00, 0x55, 0x3c,
        0x7F, // NAD Wildcard
        0x06, // 6 Byte
        0xB0, // SID = assign NAD
        0xFF, 0x7F, // Supplier ID LSB, MSB
        0xFF, 0x3F, // Function ID LSB, MSB
        newNAD,
        0x00, // chksum

        0x00, 0x55, 0x7d,
    };

    // Slave Response
    struct Response {
        std::vector<uint8_t> head {
            oldNAD, // oldNAD <-- according to spec initial NAD will be used
            0x01  // Single Frame, 1 Bytes
        };
        std::vector<uint8_t> payload {
            0xF0 // RSID
        };
        std::vector<uint8_t> payload_fillbytes {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF // unused
        };
        uint8_t checksum {
            0x04 // Frame Checksum
        };
    } response;
    linNodeConfig->mock_Input(response.head);
    linNodeConfig->mock_Input(response.payload);
    linNodeConfig->mock_Input(response.payload_fillbytes);
    linNodeConfig->mock_Input(response.checksum);

    bool result = linNodeConfig->assignNAD(&NAD, &supplierId, &functionId, newNAD);

    TEST_ASSERT_EQUAL(oldNAD, NAD);  // <-- answer will follow on oldNAD

    std::vector<uint8_t>* txBuffer = &linNodeConfig->txBuffer;

    TEST_ASSERT_EQUAL(bus_transmitted.size(), txBuffer->size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), txBuffer->data(), bus_transmitted.size());
}

void test_lin_conditionalChangeNAD_ok()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t NAD = 0x1A;
    uint8_t id = 0x01;
    uint8_t byte = 0x03;
    uint8_t invert = 0xFF;
    uint8_t mask = 0x01;
    static const uint8_t oldNAD = 0x1A;
    static const uint8_t newNAD = 0x1B;

    std::vector<uint8_t> bus_transmitted = {
        0x00, 0x55, 0x3c,
        NAD, // NAD (=master request)
        0x06, // 6 Byte
        0xB3, // SID = Conditional change NAD
        0x01, // Id
        0x03, // Byte
        0x01, // Mask
        0xFF, // Invert
        newNAD, // new NAD
        0x0C, // chksum

        0x00, 0x55, 0x7d
    };

    // Slave Response
    struct Response {
        std::vector<uint8_t> head {
            newNAD, // NAD <-- according to spec newNAD will be used
            0x01  // Single Frame, 1 Bytes
        };
        std::vector<uint8_t> payload {
            0xF3 // RSID
        };
        std::vector<uint8_t> payload_fillbytes {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF // unused
        };
        uint8_t checksum {
            0xEF // Frame Checksum
        };
    } response;
    linNodeConfig->mock_Input(response.head);
    linNodeConfig->mock_Input(response.payload);
    linNodeConfig->mock_Input(response.payload_fillbytes);
    linNodeConfig->mock_Input(response.checksum);

    bool result = linNodeConfig->conditionalChangeNAD(&NAD, id, byte, invert, mask, newNAD);

    TEST_ASSERT_EQUAL(newNAD, NAD);  // <-- answer will follow on newNAD

    std::vector<uint8_t>* txBuffer = &linNodeConfig->txBuffer;

    TEST_ASSERT_EQUAL(bus_transmitted.size(), txBuffer->size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), txBuffer->data(), bus_transmitted.size());
}

void test_lin_saveConfig_ok()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t NAD = 0x7F; // wildcard
    constexpr uint8_t NAD_response = 0x66;

    std::vector<uint8_t> bus_transmitted = {
    // Master
        0x00, 0x55, 0x3c,
        NAD, // NAD
        0x01, // 6 Byte
        0xB6, // SID = Save Config
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // unused
        0xC8, // chksum

        0x00, 0x55, 0x7d,
    };

    // Slave Response
    struct Response {
        std::vector<uint8_t> head {
            NAD_response, // NAD
            0x01  // Single Frame, 1 Bytes
        };
        std::vector<uint8_t> payload {
            0xF6 // RSID = Save Config
        };
        std::vector<uint8_t> payload_fillbytes {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF // unused
        };
        uint8_t checksum {
            0xA1 // Frame Checksum
        };
    } response;
    linNodeConfig->mock_Input(response.head);
    linNodeConfig->mock_Input(response.payload);
    linNodeConfig->mock_Input(response.payload_fillbytes);
    linNodeConfig->mock_Input(response.checksum);

    bool result = linNodeConfig->saveConfig(&NAD);

    TEST_ASSERT_EQUAL(NAD_response, NAD);  // <-- answer will follow on oldNAD

    std::vector<uint8_t>* txBuffer = &linNodeConfig->txBuffer;

    TEST_ASSERT_EQUAL(bus_transmitted.size(), txBuffer->size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), txBuffer->data(), bus_transmitted.size());
}

void test_lin_AssignFrameIdRange_ok()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t NAD = 0x7F; // wildcard
    const uint8_t start = 1;
    const uint8_t PID0 = 0x80;
    const uint8_t PID1 = 0xC1;
    const uint8_t PID2 = 0x42;
    const uint8_t PID3 = 0x00;
    constexpr uint8_t NAD_response = 0x66;

    std::vector<uint8_t> bus_transmitted = {
    // Master
        0x00, 0x55, 0x3c,
        NAD, // NAD
        0x06, // 6 Byte
        0xB7, // SID = Assign FrameId Range
        start, // start
        PID0,
        PID1,
        PID2, 
        PID3, // unused
        0x3D, // chksum

        0x00, 0x55, 0x7d,
    };

    // Slave Response
    struct Response {
        std::vector<uint8_t> head {
            NAD_response, // NAD
            0x01  // Single Frame, 1 Bytes
        };
        std::vector<uint8_t> payload {
            0xF7 // RSID = Assign Frame ID Range
        };
        std::vector<uint8_t> payload_fillbytes {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF // unused
        };
        uint8_t checksum {
            0xA0 // Frame Checksum
        };
    } response;
    linNodeConfig->mock_Input(response.head);
    linNodeConfig->mock_Input(response.payload);
    linNodeConfig->mock_Input(response.payload_fillbytes);
    linNodeConfig->mock_Input(response.checksum);

    bool result = linNodeConfig->assignFrameIdRange(&NAD, start, PID0, PID1, PID2, PID3);

    TEST_ASSERT_EQUAL(NAD_response, NAD);  // <-- answer will follow on oldNAD

    std::vector<uint8_t>* txBuffer = &linNodeConfig->txBuffer;

    TEST_ASSERT_EQUAL(bus_transmitted.size(), txBuffer->size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), txBuffer->data(), bus_transmitted.size());
}

int main() {
    UNITY_BEGIN();
    
    RUN_TEST(test_lin_wakeup);
    RUN_TEST(test_lin_sleep);
    RUN_TEST(test_lin_getID);
    RUN_TEST(test_lin_assignNAD_ok);
    RUN_TEST(test_lin_conditionalChangeNAD_ok);
    RUN_TEST(test_lin_saveConfig_ok);
    RUN_TEST(test_lin_AssignFrameIdRange_ok);
   
    return UNITY_END();
}