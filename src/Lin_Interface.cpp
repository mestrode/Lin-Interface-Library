// Lin-Interface.cpp
//
// Utilizes a UART to provide a Lin-Interface
// This class is inherited from the "HardwareSerial" call
//
// Copyright mestrode <ardlib@mestro.de>
// Original Source: "https://github.com/mestrode/Lin-Interface-Library"

// Lin 2.2A specification
// https://www.lin-cia.org/fileadmin/microsites/lin-cia.org/resources/documents/LIN_2.2A.pdf

#include "Lin_Interface.hpp"

#include <Arduino.h>

// Lin Config and ID Specification 2.1 Chapter 4.2.3.2 NAD
// 0            = go sleep command
// 1-125 (0x7D) = Slave Node Adress (NAD)
// 126   (0x7E) = functional node adress (functional NAD), only used for diagnostic
// 127   (0x7F) = Slave node adress broadcast (broadcast NAD)
// 128   (0x80)
// 255   (0xFF) = Free usage

// Lin Transport Layer Specification 2.1 Chapter 3.2.1.3 PCI
// high nibble = PCI type, low nibble = length
//  Single Frame      0b0000 length
//  First Frame       0b0001 length / 256 = high byte of length; low byte of length will be transmitted in len
//  Consecutive Frame 0b0010 FrameCounter % 16, starting with one

// Lin Transport Layer Specification 2.1 Chapter 3.2.1.5 SID
// Service Indentifier
// 0x00-0xAF   and
// 0xB8-0xFE = diagnostics
// ---- - node configuration
//      0xB0 = Assign NAD
//      0xB1 = Assign Frame Identifier (obsolete, see Lin 2.0)
//      0xB2 = Read by Identifier
//      0xB3 = Conditional Change NAD
//      0xB4 = Data Dump
//      0xB5 = Assign NAD via SNPD (reserved for Node Position detection)
//      0xB6 = Save Configuration (optional)
//      0xB7 = Assign frame identifier range
// 0xB8-0xFF = reserved

constexpr auto BREAK_BYTE = 0x00;
constexpr auto SYNC_BYTE = 0x55;

// send wakeup command by sending a bus dominant for 1.6ms (at 9600 Baud)
void Lin_Interface::writeCmdWakeup()
{
    // Lin Protocol Specification 2.1 Chapter 2.6.2 Wakeup
    // hold down for 250µs - 5ms
    // 9600 Baud = 104µs per bit ==> half speed = 208µs per bit ==> 8 bits = 1.664ms
    HardwareSerial::flush();
    // configure to half baudrate --> a t_bit will be doubled
    HardwareSerial::updateBaudRate(baud >> 1);
    // write 0x00, including Stop-Bit (=1),
    // qualifies when writing in slow motion for a wake-up-request
    write(uint8_t(BREAK_BYTE));
    // ensure this is send
    HardwareSerial::flush();
    // restore normal speed
    HardwareSerial::updateBaudRate(baud);

    // give the bus some time to wake up (100-150ms)
    delay(150);
}

/// @brief Request bus cluster to go to sleep
void Lin_Interface::writeCmdSleep()
{
    // Lin Protocol Specification 2.1 Chapter 2.6.3 Go To Sleep
    // Request from master to all nodes to go to sleep
    // only NodeID=0 shall be considered by nodes
    LinMessage[0] = 0;    // NodeId
    LinMessage[1] = 0xFF; // PCI
    LinMessage[2] = 0xFF; // SID
    LinMessage[3] = 0xFF; // D1
    LinMessage[4] = 0xFF; // D2
    LinMessage[5] = 0xFF; // D3
    LinMessage[6] = 0xFF; // D4
    LinMessage[7] = 0xFF; // D5

    writeFrame(0x3C, 8);
}

/// @brief reads data from a lin device by requesting a specific FrameID
/// @details Start frame and read answer from bus device
/// The received data will be passed to the Lin_Interface::LinMessage[] array
/// Receives as much as possible, but maximum 8 data byte + checksum
/// Verify Checksum according to LIN 2.0 rules
/// @param FrameID ID of frame (will be converted to protected ID)
/// @param expectedDataLen Length of expected data (?) bytes
/// @returns verification of checksum was succesful
bool Lin_Interface::readFrame(const uint8_t FrameID, const uint8_t expectedDataLen)
{
    uint8_t ProtectedID = getProtectedID(FrameID);
    bool ChecksumValid = false;

    startTransmission(ProtectedID);
    HardwareSerial::flush();

    // wait for available data
    if (expectedDataLen > 0)
    {
        unsigned long wtime = millis();
        while ((millis() - wtime) < 400)
        {
            if (HardwareSerial::available() >= (expectedDataLen + 4)) // 4 -> break, sync, pid, checksum
            {
                break;
            }
        }
    }
    else
    {
        delay(100);
    }

    constexpr auto START_IDX = -4;
    constexpr auto BREAK_IDX = -3;
    constexpr auto SYNC_IDX = -2;
    constexpr auto PROTECTED_ID_IDX = -1;

    // Break, Sync and ProtectedID will be received --> discard them
    int8_t bytes_received = START_IDX;
    while (HardwareSerial::available())
    {
        if (bytes_received >= (8 + 1)) // max 8x Data + 1x Checksum
        {
            // receive max 9 Bytes: 8 Data + 1 Chksum
            break;
        }

        switch (bytes_received)
        {
        case START_IDX:        // ??
        case BREAK_IDX:        // break = 0x00
        case SYNC_IDX:         // sync = SYNC_BYTE
        case PROTECTED_ID_IDX: // Protected ID
        {
            // discard Sync and PID (send by us)
            uint8_t buffer = HardwareSerial::read();
            // Sync and PID may to be verified here
            if (buffer == BREAK_BYTE)
            {
                bytes_received = BREAK_IDX;
            }
            if (buffer == SYNC_BYTE)
            {
                bytes_received = SYNC_IDX;
            }
            if (buffer == ProtectedID)
            {
                bytes_received = PROTECTED_ID_IDX;
            }
            break;
        }
        default: // Data 0...7, Checksum
            // Receive and save only Data Byte (send by slave)
            LinMessage[bytes_received] = HardwareSerial::read();
        }
        bytes_received++;
    }
    uint8_t Checksum = LinMessage[bytes_received - 1];
    bytes_received--;

    // erase data in buffer, in case a 9th or 10th Byte was received
    HardwareSerial::flush();
    while (HardwareSerial::available())
    {
        HardwareSerial::read();
        if (verboseMode > 0)
        {
            Serial.print("additional byte discarded\n");
        }
    }

    HardwareSerial::end();

    // verify Checksum
    ChecksumValid = (bytes_received > 0) && isChecksumValid(Checksum, ProtectedID, bytes_received);

    if (verboseMode > 0)
    {
        Serial.printf(" --->>>>>> FID %02Xh        = 55|%02X|", FrameID, ProtectedID);
        for (int i = 0; i < 8; ++i)
        {
            if (i >= bytes_received)
            {
                break;
            }
            Serial.printf("%02X.", LinMessage[i]);
        }

        if (bytes_received > 0)
        {
            Serial.printf("\b|%02X", Checksum);
            Serial.printf(" bytes received %d", bytes_received);

            if (!ChecksumValid)
            {
                Serial.printf(" Checksum failed");
            }
        }
        else
        {
            Serial.printf(" no bytes received");
        }

        Serial.println();
    }

    return ChecksumValid;
} // bool readFrame()

/// @brief write a complete LIN2.0 frame without request of data to the lin-bus
/// @details write LIN Frame (Break, Synk, PID, Data, Checksum) to the Bus, and hope somebody will read this
/// Checksum Calculations regarding LIN 2.0
/// The data of this frame is 'dataLen' long and incuded in the Lin_Interface::LinMessage[] array
/// @param FrameID ID of frame (will be converted to protected ID)
/// @param dataLen count of data within the LinMessage array (containing only the data) should be transmitted
void Lin_Interface::writeFrame(uint8_t FrameID, uint8_t dataLen)
{
    uint8_t ProtectedID = getProtectedID(FrameID);
    uint8_t cksum = getChecksum(ProtectedID, dataLen);

    startTransmission(ProtectedID);

    for (int i = 0; i < dataLen; ++i)
    {
        HardwareSerial::write(LinMessage[i]); // Message (array from 1..8)
    }
    HardwareSerial::write(cksum);

    // wait for available data
    delay(20);

/// TODO: read back of the break needs to be verified
    // verboseMode = 1;

    // Read Break and discard
    if (HardwareSerial::available())
    {
        HardwareSerial::read();
    }
    // Read Sync
    uint8_t RX_Sync = 0x00;
    if (HardwareSerial::available())
    {
        RX_Sync = HardwareSerial::read();
    }
    // Read PID
    uint8_t RX_ProtectedID = 0x00;
    if (HardwareSerial::available())
    {
        RX_ProtectedID = HardwareSerial::read();
    }
    // read DATA + CHKSUM
    bool moreData = false;
    int bytes_received = 0;
    while (HardwareSerial::available())
    {
        if (bytes_received >= 8 + 1 + 4)
        {
            // receive max 9 Bytes = 8 Data + 1 Chksum
            moreData = true;
            break;
        }
        // Receive Byte from Bus (Slave)
        LinMessage[bytes_received] = HardwareSerial::read();
        bytes_received++;
    }
    uint8_t Checksum_received = LinMessage[bytes_received - 1];
    bytes_received--;

    // erase data in buffer, in case a 9th or 10th Byte was received
    HardwareSerial::flush();
    HardwareSerial::end();

    // use received PID  for verification
    uint8_t ChkSumCalc = getChecksum(RX_ProtectedID, bytes_received);

    if (verboseMode > 0)
    {
        Serial.printf(" <<<<<<--- FID %02Xh (%02X)   = %02X|%02X|", FrameID, ProtectedID, RX_Sync, RX_ProtectedID);
        for (int i = 0; i < 8 + 1 + 4; ++i)
        {
            if (i >= bytes_received)
                break;
            Serial.printf("%02X ", LinMessage[i]);
        }

        Serial.printf("\b|%02X", Checksum_received);
        if (Checksum_received != ChkSumCalc)
        {
            Serial.printf("\b != ChkSum calc %02Xh| TX %02Xh ", ChkSumCalc, cksum);
        }

        if (moreData)
        {
            Serial.print("more Bytes available");
        }

        Serial.println();
    }
} // void writeFrame()

/// TODO: function needs to be verified
/// send Frame (Break, Synk, PID, Data, Classic-Checksum) to the Bus
/// Checksum Calculations regarding LIN 1.x
void Lin_Interface::writeFrameClassic(uint8_t FrameID, uint8_t dataLen)
{
    uint8_t ProtectedID = getProtectedID(FrameID);
    uint8_t cksum = getChecksum(0x00, dataLen);

    startTransmission(ProtectedID);

    for (int i = 0; i < dataLen; ++i)
    {
        HardwareSerial::write(LinMessage[i]); // Message (array from 1..8)
    }
    HardwareSerial::write(cksum);
    HardwareSerial::flush();

/// TODO: verification of written data (see Lin_Interface::writeFrame)

    HardwareSerial::end();
} // void Lin_Interface::writeFrameClassic

void Lin_Interface::writeFrameClassicNoChecksum(uint8_t FrameID, uint8_t dataLen)
{
    uint8_t ProtectedID = getProtectedID(FrameID);

    startTransmission(ProtectedID);

    for (int i = 0; i < dataLen; ++i)
    {
        HardwareSerial::write(LinMessage[i]); // Message (array from 1..8)
    }
    HardwareSerial::flush();
    HardwareSerial::end();
}

/// Introduce Frame (Start UART, Break, Sync, PID)
void Lin_Interface::startTransmission(uint8_t ProtectedID)
{
    // start UART
    if (rxPin < 0 && txPin < 0)
    {
        // no custom pins are defined
        HardwareSerial::begin(baud, SERIAL_8N1);
    }
    else
    {
        HardwareSerial::begin(baud, SERIAL_8N1, rxPin, txPin);
    }

    writeBreak();                       // initiate Frame with a Break
    HardwareSerial::write(SYNC_BYTE);   // Sync
    HardwareSerial::write(ProtectedID); // PID
}

/// Send a Break for introduction of a Frame
/// This is done by sending a Byte (0x00) + Stop Bit by using half baud rate
/// @returns if the 0x00 has been send
size_t Lin_Interface::writeBreak()
{
    HardwareSerial::flush();
    // configure to half baudrate --> a t_bit will be doubled
    HardwareSerial::updateBaudRate(baud >> 1);
    // write 0x00, including Stop-Bit (=1),
    // qualifies when writing in slow motion like a Break in normal speed
    size_t ret = HardwareSerial::write(uint8_t(0x00));
    // ensure this is send
    HardwareSerial::flush();
    // restore normal speed
    HardwareSerial::updateBaudRate(baud);
    return ret;
}

/// get Protected ID by calculating parity bits and combine with Frame ID
/// @param FrameID to be converted
/// @return Protected ID
uint8_t Lin_Interface::getProtectedID(uint8_t FrameID)
{
    // calc Parity Bit 0
    uint8_t p0 = bitRead(FrameID, 0) ^ bitRead(FrameID, 1) ^ bitRead(FrameID, 2) ^ bitRead(FrameID, 4);
    // calc Parity Bit 1
    uint8_t p1 = ~(bitRead(FrameID, 1) ^ bitRead(FrameID, 3) ^ bitRead(FrameID, 4) ^ bitRead(FrameID, 5));
    // combine bits to protected ID
    // 0..5 id is limited between 0x00..0x3F
    // 6    parity bit 0
    // 7    parity bit 1
    return ((p1 << 7) | (p0 << 6) | (FrameID & 0x3F));
}

/// @brief Checksum calculation for LIN Frame
/// @details
/// EnhancedChecksum considers ProtectedID
///     LIN 2.0 only for FrameID between 0x00..0x3B
///     LIN 2.0 uses for 0x3C and above ClassicChecksum for legacy (auto detected)
/// ClassicChecksum
///     LIN 1.x in general (use 'ProtectedID' = 0x00 to ensure that)
/// see LIN Specification 2.2A (2021-12-31) for details
///     https://microchipdeveloper.com/local--files/lin:specification/LIN-Spec_2.2_Rev_A.PDF
///     2.8.3 Example of Checksum Calculation
/// @param ProtectedID initial Byte, set to 0x00, when calc Checksum for classic LIN Frame
/// @param dataLen length of Frame (only Data Bytes)
/// @returns calculated checksum
uint8_t Lin_Interface::getChecksum(uint8_t ProtectedID, uint8_t dataLen)
{
    uint16_t sum = 0x00;

    // consider configuration and reserved frames
    if ((ProtectedID & 0x3F) < 0x3C)
    {
        // for LIN 2.0 configuration and reserved frames:
        //   include Protected ID in Checksum calculation
        // (=legacy: LIN 1.3 Procedted ID is included in all types of Frames)
        sum = ProtectedID;
    }

    // sum up all bytes (including carryover to the high byte)
    for (uint8_t i = 0; i < dataLen; i++)
    {
        sum += LinMessage[i];
    }

    // add high byte (carry over) to the low byte
    sum = (sum & 0xFF) + (sum >> 8);
    sum += (sum >> 8);

    // invert result
    return static_cast<uint8_t>(~sum);
}

bool Lin_Interface::isChecksumValid(uint8_t Checksum, uint8_t ProtectedID, size_t bytes_received)
{
    constexpr uint8_t CHECKSUM_MASK = 0xFF;
    bool valid = CHECKSUM_MASK == static_cast<uint8_t>(Checksum + static_cast<uint8_t>(~getChecksum(ProtectedID, bytes_received)));

    if (!valid && verboseMode > 0)
    {
        Serial.println("Checksum verification failed.");
    }

    return valid;
}
