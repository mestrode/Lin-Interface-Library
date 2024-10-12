#ifndef MOCK_HARDWARE_SERIAL_H
#define MOCK_HARDWARE_SERIAL_H

#include "mock_Stream.h"

#include "unity.h"

#include <stdint.h>
#include <iostream>
#include <queue>

enum SerialConfig {
    SERIAL_5N1 = 0x8000010,
    SERIAL_6N1 = 0x8000014,
    SERIAL_7N1 = 0x8000018,
    SERIAL_8N1 = 0x800001c,
    SERIAL_5N2 = 0x8000030,
    SERIAL_6N2 = 0x8000034,
    SERIAL_7N2 = 0x8000038,
    SERIAL_8N2 = 0x800003c,
    SERIAL_5E1 = 0x8000012,
    SERIAL_6E1 = 0x8000016,
    SERIAL_7E1 = 0x800001a,
    SERIAL_8E1 = 0x800001e,
    SERIAL_5E2 = 0x8000032,
    SERIAL_6E2 = 0x8000036,
    SERIAL_7E2 = 0x800003a,
    SERIAL_8E2 = 0x800003e,
    SERIAL_5O1 = 0x8000013,
    SERIAL_6O1 = 0x8000017,
    SERIAL_7O1 = 0x800001b,
    SERIAL_8O1 = 0x800001f,
    SERIAL_5O2 = 0x8000033,
    SERIAL_6O2 = 0x8000037,
    SERIAL_7O2 = 0x800003b,
    SERIAL_8O2 = 0x800003f
};

class mock_HardwareSerial : public mock_Stream {
public:
    bool mock_loopback = false;

    mock_HardwareSerial(uint8_t uart_nr) : mock_Stream() {
        std::cout << "mock_HardwareSerial() created with UART number: " << (int)uart_nr << std::endl;
    }

    void begin(unsigned long baud, uint32_t config = 0, int8_t rxPin = -1, int8_t txPin = -1, bool invert = false, unsigned long timeout_ms = 20000UL) {
        mock_baud = baud;
        std::cout << "HardwareSerial::begin(..) called: " << mock_baud << " Baud" << std::endl;
        TEST_ASSERT_FALSE_MESSAGE(begin_used, "double call of HardwareSerial::begin()");
        begin_used = true;
        flush_done = true;
    }

    void end() {
        std::cout << "HardwareSerial::end() called" << std::endl;
        std::cout << "TX " << txCnt << "\t\tBytes" << std::endl;
        std::cout << "RX\t" << rxCnt << "\tBytes" << std::endl;
        TEST_ASSERT_TRUE_MESSAGE(begin_used, "missing call of HardwareSerial::begin()");
        TEST_ASSERT_TRUE_MESSAGE(flush_done, "expect HardwareSerial::flush() before HardwareSerial::end()");
        begin_used = false;
    }

    int available() override {
        auto len = 0;
        if (mock_loopback)
        {
            len += loopbackBuffer.size();
        }
        len += rxBuffer.size();
        return len;
    }

    int read() override {
        TEST_ASSERT_TRUE_MESSAGE(begin_used, "missing call of HardwareSerial::begin()");

        // priorize loopback
        if (mock_loopback && !loopbackBuffer.empty()) {
            int byte = loopbackBuffer.front();
            loopbackBuffer.pop();
            rxCnt++;
            std::cout << "\t#" << rxCnt << "\t\t\t< 0x" << std::hex << byte << std::dec  << "\t(loopback)" << std::endl;
            return byte;
        }

        // rx data from mock
        if (rxBuffer.empty()) {
            std::cout << "--> HardwareSerial::read(): no Data available" << std::endl;
            return -1;
        }
        int byte = rxBuffer.front();
        rxBuffer.pop();
        rxCnt++;
        std::cout << "\t#" << rxCnt << "\t\t\t< 0x" << std::hex << byte << std::dec << std::endl;
        return byte;
    }

    size_t write(uint8_t byte) override {
        TEST_ASSERT_TRUE_MESSAGE(begin_used, "missing call of HardwareSerial::begin()");

        if (mock_loopback)
        {
            loopbackBuffer.push(byte);
        }
        txCnt++;
        std::cout << "#" << txCnt << "\t\t\t0x" << std::hex << (int)byte << std::dec << " >"<< std::endl;
        flush_done = false;
        return mock_Stream::write(byte); // Call the base class write method to handle output
    }

    void flush() override {
        TEST_ASSERT_TRUE_MESSAGE(begin_used, "missing call of HardwareSerial::begin()");
        std::cout << "HardwareSerial::flush() called - TX: " << txBuffer.size() << " Byte(s); RX: " << available() << " Byte(s)" << std::endl;
        flush_done = true;
    }

    uint32_t baudRate() const {
        return mock_baud;
    }

    void updateBaudRate(unsigned long value) {
        TEST_ASSERT_TRUE_MESSAGE(begin_used, "missing call of HardwareSerial::begin()");
        TEST_ASSERT_TRUE_MESSAGE(flush_done, "expect HardwareSerial::flush() before BaudRate is changed");
        std::cout << "HardwareSerial::updateBaudRate() to " << value << " Baud" << std::endl;
        mock_baud = value;
    }

    void mock_Input(const uint8_t data) {
        rxBuffer.push(data);
    }

    void mock_Input(const std::vector<uint8_t>& data) {
        for (uint8_t c : data) {
            rxBuffer.push(c);
        }
    }

private:
    int txCnt = 0;
    int rxCnt = 0;
    uint32_t mock_baud = 0;
    std::queue<uint8_t> loopbackBuffer;
    std::queue<uint8_t> rxBuffer; // Mock RX buffer for incoming data

    bool begin_used = false;
    bool flush_done = true;
};

#endif // MOCK_HARDWARE_SERIAL_H
