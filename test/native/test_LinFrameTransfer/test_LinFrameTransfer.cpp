#include <unity.h>
#include "LinFrameTransfer.hpp"
#include "mock_HardwareSerial.h"
#include "mock_DebugStream.hpp"

mock_DebugStream debugStream;

mock_HardwareSerial* linDriver;
LinFrameTransfer* linFrameTransfer;

void setUp()
{
    linDriver = new mock_HardwareSerial(0);
    linDriver->mock_loopback = true;
    linDriver->begin(19200, SERIAL_8N1);

    linFrameTransfer = new LinFrameTransfer(*linDriver, debugStream, 2);
}

void tearDown()
{
    delete linFrameTransfer;

    linDriver->end();
    delete linDriver;
}

void test_lin_writeFrame_Ok()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    // Test was planned according to: 2.8.3 Example of Checksum Calculation
    // but example PID = 0x4A is invalid.

    uint8_t FrameID = 0x10;
    std::vector<uint8_t> request = {
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        0x07,
        0x08
    };

    std::vector<uint8_t> bus_transmitted = {
    // Master
        0x00, // break
        0x55, // sync
        0x50, // PID = FID + 0x40
        0x01, // Data 1
        0x02, // Data 2
        0x03, // Data 3
        0x04, // Data 4
        0x05, // Data 5
        0x06, // Data 6
        0x07, // Data 7
        0x08, // Data 8
        0x8B  // Checksum
    };

    bool result = linFrameTransfer->writeFrame(FrameID, request);

    TEST_ASSERT_TRUE(result); // success

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

void test_lin_writeFrame_Write_Failed()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t FrameID = 0x10;
    std::vector<uint8_t> request = {
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        0x07,
        0x08
    };

    std::vector<uint8_t> bus_transmitted = {
        0x00, // break
        0x55, // sync
        0x50, // PID
        0x01, // Data 1 --> was send without bit error
        0x02, // Data 2
        0x03, // Data 3
        0x04, // Data 4
        0x05, // Data 5
        0x06, // Data 6
        0x07, // Data 7
        0x08, // Data 8
        0x8B  // Checksum
    };

    std::vector<uint8_t> bus_received = {
    // Master
        0x00, // break
        0x55, // sync
        0x50, // PID
        0x11, // Data 1 --> single bit error caused on bus
        0x02, // Data 2
        0x03, // Data 3
        0x04, // Data 4
        0x05, // Data 5
        0x06, // Data 6
        0x07, // Data 7
        0x08, // Data 8
        0x8B  // Checksum
    };

    linDriver->mock_loopback = false;
    linDriver->mock_Input(bus_received);

    bool result = linFrameTransfer->writeFrame(FrameID, request);

    TEST_ASSERT_FALSE(result); // Fail

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

void test_lin_writeFrame_Empty()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t FrameID = 0x10;
    std::vector<uint8_t> request = {}; // Empty data

    std::vector<uint8_t> bus_transmitted = {
        0x00, // break
        0x55, // sync
        0x50  // PID = FID + 0x40
        // No data, no checksum
    };

    bool result = linFrameTransfer->writeFrame(FrameID, request);

    TEST_ASSERT_TRUE(result); // success for empty frame

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

void test_lin_writeFrame_MaxData()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t FrameID = 0x10;
    std::vector<uint8_t> request = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10
    }; // Maximum frame size = 16 bytes

    std::vector<uint8_t> bus_transmitted = {
        0x00, // break
        0x55, // sync
        0x50, // PID = FID + 0x40
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, // Data
        0x27  // Checksum
    };

// TODO: doublecheck checksum for 16 byte frames. consider 16 byte frames as non compliant

    bool result = linFrameTransfer->writeFrame(FrameID, request);

    TEST_ASSERT_TRUE(result); // success for maximum data frame

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

void test_lin_writeFrame_RepeatTransmission()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t FrameID = 0x10;
    std::vector<uint8_t> request = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
    };

    std::vector<uint8_t> bus_transmitted = {
        0x00, // break
        0x55, // sync
        0x50, // PID
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // Data
        0x8B  // Checksum
    };

    for (int i = 0; i < 3; i++) {
        bool result = linFrameTransfer->writeFrame(FrameID, request);

        TEST_ASSERT_TRUE(result); // success for each transmission

        TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
        TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());

        linDriver->txBuffer.clear();
    }
}

void test_lin_readFrame_Ok()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t FrameID = 0x44;

    int requested_bytes = 8;

    std::vector<uint8_t> bus_transmitted = {
        0x00, // break
        0x55, // sync
        0xC4  // PID
    };

    struct Bus_Received
    {
        std::vector<uint8_t> data = {
            0x01, // Data 1
            0x02, // Data 2
            0x03, // Data 3
            0x04, // Data 4
            0x05, // Data 5
            0x06, // Data 6
            0x07, // Data 7
            0x08  // Data 8
        };
        uint8_t checksum = 0x17;
    } bus_received;

    linDriver->mock_Input(bus_received.data);
    linDriver->mock_Input(bus_received.checksum);

    auto result = linFrameTransfer->readFrame(FrameID, requested_bytes);

    TEST_ASSERT_TRUE(result.has_value());
    TEST_ASSERT_EQUAL(bus_received.data.size(), result.value().size());
    TEST_ASSERT_EQUAL_MEMORY(bus_received.data.data(), result.value().data(), bus_received.data.size());

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

void test_lin_readFrame_Checksum_Failed()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t FrameID = 0x44;
    struct Request
    {
    } request;

    std::vector<uint8_t> bus_transmitted = {
        0x00, // break
        0x55, // sync
        0xC4  // PID
    };

    struct Response
    {
        std::vector<uint8_t> data = {
            0x01, // Data 1
            0x02, // Data 2
            0x03, // Data 3
            0x04, // Data 4
            0x05, // Data 5
            0x06, // Data 6
            0x07, // Data 7
            0x08  // Data 8
        };
        uint8_t checksum = 0x00;
    } response;

    linDriver->mock_Input(response.data);
    linDriver->mock_Input(response.checksum);

    auto result = linFrameTransfer->readFrame(FrameID, response.data.size());

    TEST_ASSERT_FALSE(result.has_value());

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

void test_lin_readFrame_FrameShort()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t FrameID = 0x44;
    int requested_bytes = 8;

    std::vector<uint8_t> bus_transmitted = {
        0x00, // break
        0x55, // sync
        0xC4  // PID
    };

    struct Response
    {
        std::vector<uint8_t> data = {
            0x01, // Data 1
            0x02, // Data 2
            0x03, // Data 3
            0x04, // Data 4
            0x05, // Data 5
            0x06, // Data 6
            0x07 // Data 7
            // <-- BYTE IS MISSING
        };
        uint8_t checksum = 0x1F;
    } response;

    linDriver->mock_Input(response.data);
    linDriver->mock_Input(response.checksum);

    auto result = linFrameTransfer->readFrame(FrameID, requested_bytes);

    TEST_ASSERT_FALSE(result.has_value());

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

void test_lin_readFrame_BusTimeout()
{
    std::cout << "\n\n\nRunning test: " << __FUNCTION__ << std::endl;

    uint8_t FrameID = 0x44;
    int requested_bytes = 8;

    std::vector<uint8_t> bus_transmitted = {
        0x00, // break
        0x55, // sync
        0xC4  // PID
    };

    // Simulating no data input (bus timeout)
    linDriver->mock_Input({}); // No data response

    auto result = linFrameTransfer->readFrame(FrameID, requested_bytes);

    TEST_ASSERT_FALSE(result.has_value()); // timeout, no response

    TEST_ASSERT_EQUAL(bus_transmitted.size(), linDriver->txBuffer.size());
    TEST_ASSERT_EQUAL_MEMORY(bus_transmitted.data(), linDriver->txBuffer.data(), bus_transmitted.size());
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_lin_writeFrame_Ok);
    RUN_TEST(test_lin_writeFrame_Write_Failed);
    RUN_TEST(test_lin_writeFrame_Empty);
    RUN_TEST(test_lin_writeFrame_MaxData);
    RUN_TEST(test_lin_writeFrame_RepeatTransmission);

    RUN_TEST(test_lin_readFrame_Ok);
    RUN_TEST(test_lin_readFrame_Checksum_Failed);
    RUN_TEST(test_lin_readFrame_FrameShort);
    RUN_TEST(test_lin_readFrame_BusTimeout);


    return UNITY_END();
}
