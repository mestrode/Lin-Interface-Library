#include <unity.h>
#include "LinTransportLayer.hpp"
#include "mock_HardwareSerial.h"
#include "mock_DebugStream.hpp"

mock_DebugStream debugStream;

mock_HardwareSerial* linDriver;
LinTransportLayer* linTransportLayer;

void setUp()
{
    linDriver = new mock_HardwareSerial(0);
    linDriver->mock_loopback = true;
    linDriver->begin(19200, SERIAL_8N1);
    linTransportLayer = new LinTransportLayer(*linDriver, debugStream, 2);
}

void tearDown()
{
    delete linTransportLayer;
    linDriver->end();
    delete linDriver;
}

void test_write_DTL_MasterRequest_SF_SF()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    // Master Request
    constexpr uint8_t NAD_value = 0x0A;
    uint8_t NAD = NAD_value;
    struct Request
    {
        std::vector<uint8_t> payload = {
            0x22, // SID
            0x06, 0x2E // real payload
        };
    } request;

    // Slave Response
    struct Response {
        std::vector<uint8_t> head {
            NAD_value, // NAD
            0x06  // Single Frame, 6 Bytes
        };
        std::vector<uint8_t> payload {
            0x62, 0x06, 0x2E, 0x80, 0x00, 0x00 // payload
        };
        uint8_t checksum {
            0xD8 // Frame Checksum
        };
    } response;

    linDriver->mock_Input(response.head);
    linDriver->mock_Input(response.payload);
    // no fillbytes required
    linDriver->mock_Input(response.checksum);

    auto result = linTransportLayer->writePDU(NAD, request.payload);

    TEST_ASSERT_TRUE(result.has_value());
    TEST_ASSERT_EQUAL(NAD_value, NAD_value);

    TEST_ASSERT_EQUAL(response.payload.size(), result.value().size());
    TEST_ASSERT_EQUAL_MEMORY(response.payload.data(), result.value().data(), response.payload.size());


    std::vector<uint8_t> expectation = {
    // Master Request
        0x00, 0x55, 0x3C, // Frame Head = Master Request
        NAD_value, // NAD
        0x03, // Single Frame, 3 Bytes
        0x22, 0x06, 0x2E, // payload
        0xff, 0xff, 0xff, // filing bytes
        0x9c, // Frame Checksum

    // Slave Response
        0x00, 0x55, 0x7D // Frame Head = Slave Response
        // body send by mock
    };

    TEST_ASSERT_EQUAL(expectation.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(expectation.data(), linDriver->txBuffer.data(), expectation.size());
}


void test_write_DTL_MasterRequest_SF_SF2()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    // Master Request
    constexpr uint8_t NAD_request = 0x0A;
    uint8_t NAD = NAD_request;
    std::vector<uint8_t> payload = {
        0x22, // SID
        0x06, 0x2E, 0x87, 0x47, 0x41 // real payload
    };

    // Slave Response
    constexpr uint8_t NAD_slave = NAD_request;
    std::vector<uint8_t> mock_DTL_response_head {
        NAD_slave, // NAD
        0x05  // Type | LEN
    }; // Single Frame, 6 Bytes
    std::vector<uint8_t> mock_DTL_response_payload {
        0x62, 0x06, 0x2E, 0x80, 0x00 // payload
    };
    std::vector<uint8_t> mock_DTL_response_payload_fillbytes {
        0xFF // fill bytes
    };
    uint8_t mock_DTL_response_checksum {
        0xD9
    }; // Frame Checksum
    linDriver->mock_Input(mock_DTL_response_head);
    linDriver->mock_Input(mock_DTL_response_payload);
    linDriver->mock_Input(mock_DTL_response_payload_fillbytes);
    linDriver->mock_Input(mock_DTL_response_checksum);

    auto result = linTransportLayer->writePDU(NAD, payload);

    TEST_ASSERT_TRUE(result.has_value());
    TEST_ASSERT_EQUAL(NAD_slave, NAD);

    TEST_ASSERT_EQUAL(mock_DTL_response_payload.size(), result.value().size());
    TEST_ASSERT_EQUAL_MEMORY(mock_DTL_response_payload.data(), result.value().data(), mock_DTL_response_payload.size());


    std::vector<uint8_t> expectation = {
    // Master Request
        0x00, 0x55, 0x3C, // Frame Head = Master Request
        NAD_request, // NAD
        0x06, // Single Frame, 3 Bytes
        0x22, 0x06, 0x2E, 0x87, 0x47, 0x41, // payload
        // no filing bytes
        137, // Frame Checksum

    // Slave Response
        0x00, 0x55, 0x7D // Frame Head = Slave Response
        // body send by mock
    };
    TEST_ASSERT_EQUAL(expectation.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(expectation.data(), linDriver->txBuffer.data(), expectation.size());
}

void test_write_DTL_MasterRequest_SF_MultiFrame()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    // Master Request
    constexpr uint8_t NAD_request = 0x7F; // Wildcard
    uint8_t NAD = NAD_request;
    std::vector<uint8_t> payload = {
        0x22, // SID
        0x06, 0x5E // real payload
    };

    // Slave Response
    constexpr uint8_t NAD_slave = 0x0A;
    std::vector<uint8_t> mock_DTL_response {
    // FF
        NAD_slave, // NAD
        0x10, 0x14,  // Type | LEN
        0x62, 0x06, 0x5E, 0x96, 0x54, // payload
        0x20, // Frame Checksum
    // 1st CC
        NAD_slave, // NAD
        0x21,  // Type | SN
        0x62, 0x06, 0x5E, 0x44, 0x55, 0x78, // payload
        0xFB, // Frame Checksum
    // 2nd CC
        NAD_slave, // NAD
        0x22,  // Type | SN
        0x54, 0x10, 0x01, 0x00, 0xFF, 0xEE, // payload
        0x7F, // Frame Checksum
    // 3rd CC
        NAD_slave, // NAD
        0x23,  // Type | SN
        0x12, 0x99, 0x21, // payload
        0xFF, 0xFF, 0xFF, // fill bytes
        0x06 // Frame Checksum
    };
    linDriver->mock_Input(mock_DTL_response);

    auto result = linTransportLayer->writePDU(NAD, payload);

    std::vector<uint8_t> mock_payload {
        0x62, 0x06, 0x5E, 0x96, 0x54,
        0x62, 0x06, 0x5E, 0x44, 0x55, 0x78,
        0x54, 0x10, 0x01, 0x00, 0xFF, 0xEE,
        0x12, 0x99, 0x21
    };

    TEST_ASSERT_TRUE(result.has_value());
    TEST_ASSERT_EQUAL(NAD_slave, NAD);

    TEST_ASSERT_EQUAL(mock_payload.size(), result.value().size());
    TEST_ASSERT_EQUAL_MEMORY(mock_payload.data(), result.value().data(), mock_payload.size());


    std::vector<uint8_t> expectation = {
    // Master Request
        0x00, 0x55, 0x3C, // Frame Head = Master Request
        NAD_request, // NAD
        0x03, // Single Frame, 3 Bytes
        0x22, 0x06, 0x5E, // payload
        0xFF, 0xFF, 0xFF, // filing bytes
        0xF6, // Frame Checksum

    // Slave Response
        0x00, 0x55, 0x7D, // Frame Head = Slave Response
        // body send by mock
        0x00, 0x55, 0x7D, // Frame Head = Slave Response
        // body send by mock
        0x00, 0x55, 0x7D, // Frame Head = Slave Response
        // body send by mock
        0x00, 0x55, 0x7D // Frame Head = Slave Response
        // body send by mock
    };

    TEST_ASSERT_EQUAL(expectation.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(expectation.data(), linDriver->txBuffer.data(), expectation.size());
}

void test_write_DTL_MasterRequest_MultiFrame_SF()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;
    std::cout << "fillbytes: yes + yes" << std::endl;

    // Master Request
    constexpr uint8_t NAD_request = 0x7F; // Wildcard
    uint8_t NAD = NAD_request;
    std::vector<uint8_t> payload = {
        0x62, 0x06, 0x5E, 0x33, 0x43,
        0x38, 0x39, 0x35, 0x39, 0x35, 0x33,
        0x37, 0x20, 0x20
    };

    // Slave Response
    constexpr uint8_t NAD_slave = 0x0A;
    std::vector<uint8_t> mock_DTL_response_head {
        NAD_slave, // NAD
        0x01  // Single Frame, 1 Bytes
    }; 
    std::vector<uint8_t> mock_DTL_response_payload {
        0xA2 // payload = RSID
    };
    std::vector<uint8_t> mock_DTL_response_payload_fillbytes {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF // fill bytes
    };
    uint8_t mock_DTL_response_checksum {
        0x52 // Frame Checksum
    }; 
    linDriver->mock_Input(mock_DTL_response_head);
    linDriver->mock_Input(mock_DTL_response_payload);
    linDriver->mock_Input(mock_DTL_response_payload_fillbytes);
    linDriver->mock_Input(mock_DTL_response_checksum);

    auto result = linTransportLayer->writePDU(NAD, payload);

    std::vector<uint8_t> mock_payload {
        0x12, 0x34, 0x56, 0x78, 0x9A // payload
    };

    TEST_ASSERT_TRUE(result.has_value());
    TEST_ASSERT_EQUAL(NAD_slave, NAD);

    TEST_ASSERT_EQUAL(mock_DTL_response_payload.size(), result.value().size());
    TEST_ASSERT_EQUAL_MEMORY(mock_DTL_response_payload.data(), result.value().data(), mock_DTL_response_payload.size());

    std::vector<uint8_t> expectation = {
    // Master Request
        0x00, 0x55, 0x3C, // Frame Head
        NAD_request,
        0x10, 0x0E, // First Frame, over all len = e Bytes
        0x62, 0x06, 0x5E, 0x33, 0x43, // payload
        0x25, // Frame Checksum

        0x00, 0x55, 0x3C, // Frame Head
        NAD_request,
        0x21, // Consegutive Frame, count = 1
        0x38, 0x39, 0x35, 0x39, 0x35, 0x33, // payload
        0x17, // Frame Checksum

        0x00, 0x55, 0x3C, // Frame Head
        NAD_request,
        0x22, // Consegutive Frame, count = 2
        0x37, 0x20, 0x20, // payload
        0xFF, 0xFF, 0xFF, // fill bytes
        0xe6, // Frame Checksum

    // Slave Response
        0x00, 0x55, 0x7D, // Frame Head 
        // body send by mock
    };

    TEST_ASSERT_EQUAL(expectation.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(expectation.data(), linDriver->txBuffer.data(), expectation.size());
}


void test_write_DTL_MasterRequest_MultiFrame_MultiFrame()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;
    std::cout << "fillbytes: no + no" << std::endl;

    constexpr uint8_t NAD_request = 0x7F; // wildcard
    uint8_t NAD = NAD_request;

    // Master Request
    struct Request {
        std::vector<uint8_t> payload = {
            0x99, 0x98, 0x97, 0x96, 0x95,
            0x89, 0x88, 0x87, 0x86, 0x85, 0x84,
            0x79, 0x78, 0x77, 0x76, 0x75, 0x74
        };
    } request;

    // Slave Response
    constexpr const uint8_t NAD_slave = 0x0A;
    struct Response {
        std::vector<uint8_t> mock_DTLs {
        // FF
            NAD_slave, // NAD
            0x10, 0x11,  // Type | LEN
            0xD9, 0xA2, 0xA3, 0xA4, 0xA5, // payload
            0x6A, // Frame Checksum
        // 1st CC
            NAD_slave, // NAD
            0x21,  // Type | cnt =1
            0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, // payload
            0x9B, // Frame Checksum
        // 2nd CC
            NAD_slave, // NAD
            0x22,  // Type | cnt = 2
            0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, // payload
            // no fill bytes
            0x3A // Frame Checksum
        };
    } response;

    std::vector<uint8_t> payload {
        0xD9, 0xA2, 0xA3, 0xA4, 0xA5,
        0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6,
        0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6
    };

    linDriver->mock_Input(response.mock_DTLs);

    auto result = linTransportLayer->writePDU(NAD, request.payload);

    TEST_ASSERT_TRUE(result.has_value());
    TEST_ASSERT_EQUAL(NAD_slave, NAD);

    TEST_ASSERT_EQUAL(payload.size(), result.value().size());
    TEST_ASSERT_EQUAL_MEMORY(payload.data(), result.value().data(), payload.size());

    std::vector<uint8_t> expectation = {
    // Master Request
        0x00, 0x55, 0x3C, // Frame Head
        NAD_request, // NAD
        0x10, 0x11, // First Frame, over all len = 17 Bytes
        0x99, 0x98, 0x97, 0x96, 0x95, // payload
        0x69, // Frame Checksum

        0x00, 0x55, 0x3C, // Frame Head
        NAD_request, // NAD
        0x21, // Consegutive Frame, count = 1
        0x89, 0x88, 0x87, 0x86, 0x85, 0x84, // payload
        0x35, // Frame Checksum

        0x00, 0x55, 0x3C, // Frame Head
        NAD_request, // NAD
        0x22, // Consegutive Frame, count = 2
        0x79, 0x78, 0x77, 0x76, 0x75, 0x74, // payload
        // no fill bytes
        0x94, // Frame Checksum

    // Slave Response
        0x00, 0x55, 0x7D, // Frame Head 
        // body send by mock
        0x00, 0x55, 0x7D, // Frame Head 
        // body send by mock
        0x00, 0x55, 0x7D // Frame Head 
        // body send by mock
    };

    TEST_ASSERT_EQUAL(expectation.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(expectation.data(), linDriver->txBuffer.data(), expectation.size());
}

int main() {
    UNITY_BEGIN();
    
    RUN_TEST(test_write_DTL_MasterRequest_SF_SF);
    RUN_TEST(test_write_DTL_MasterRequest_SF_SF2);
    RUN_TEST(test_write_DTL_MasterRequest_SF_MultiFrame);
    RUN_TEST(test_write_DTL_MasterRequest_MultiFrame_SF);
    RUN_TEST(test_write_DTL_MasterRequest_MultiFrame_MultiFrame);
   
    return UNITY_END();
}