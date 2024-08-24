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

#include "DebugStream.hpp"

DebugStream Debug(&Serial, 2);

constexpr auto TIMEOUT_INTERVAL = 11; // ms = 10 Bytes @ 9600 Baud (including start+stop bits)

constexpr uint8_t LIN_BREAK = 0x00;
constexpr uint8_t LIN_SYNC_FIELD = 0x55;

// Lin Config and ID Specification 2.1 Chapter 4.2.3.2 NAD
// 0            = go sleep command
// 1-125 (0x7D) = Slave Node Adress (NAD)
// 126   (0x7E) = functional node adress (functional NAD), only used for diagnostic
// 127   (0x7F) = Slave node adress broadcast (broadcast NAD)
// 128   (0x80)
// 255   (0xFF) = Free usage

// LIN Specification said abaout FrameIDs:
//    0-50 (0x00-0x3B) are used for normal Signal/data carrying frames.
//    60 (0x3C) and 61 (0x3D) are used to carry diagnostic and configuration data.
//    62 (0x3E) and 63 (0x3F) are reserved for future protocol enhancements.
constexpr auto FRAME_ID_MASTER_REQUEST = 0x3C;
constexpr auto FRAME_ID_SLAVE_REQUEST = 0x3D;

// Source https://www.lin-cia.org/fileadmin/microsites/lin-cia.org/resources/documents/LIN_2.2A.pdf
// 4.2.3.2 NAD = Node Address
constexpr uint8_t NAD_SLEEP = 0x00; // Reserved for go to sleep command, see Section 2.6.3
// 0x01 - 0x7D = Slave node adresses (NAD)
constexpr uint8_t NAD_FUNCTIONAL = 0x7E; // Functional node address (functional NAD), only used for diagnostics (using the transport layer)
constexpr uint8_t NAD_BROADCAST = 0x7F; // Slave node address broadcast (broadcast NAD)
// 0x80 - 0xFF = Free Usage

// 3.2.1.3 PCI
// 4.2.3.3 PCI = Protocol Control Information (= Length of Message)
// 0x0L = high nibble == 0 --> Single Frame, message fits into the single PDU
//      = low  nibble == length of message --> SID + max 5 Bytes --> length = max. 6 

// not implemented here:
// 0x1? = high nibble == 1 --> First Frame
//      = low  nibble  == length / 256
// 0x2? = high nibble == 2 --> Frame Counter, Consecutive Frame
//      = low  nibble == FrameCounter % 16, starting with one

// 3.2.1.4 SID
// 4.2.3.5 SID = Service Identifier
// 0x00 - 0xAF = diagnostic
// 0xB0 - 0xB7 = node configuration
// 0xB8 - 0xFE = diagnostic
constexpr auto SID_ASSIGN_NAD = 0xB0; // Assign NAD (Optional)
constexpr auto SID_ASSIGN_FRAME_ID = 0xB1; // Assign Frame Identifier (obsolete, see Lin 2.0)
constexpr auto SID_READ_BY_ID = 0xB2; // Read by Identifier (Mandatory)
constexpr auto SID_CONDITIONAL_CHANGE = 0xB3; // Conditional Change NAD (Optional)
constexpr auto SID_DATA_DUMP = 0xB4; // Data Dump (Optional)
constexpr auto SID_RESERVED = 0xB5; // Assign NAD via SNPD (reserved for Node Position detection)
constexpr auto SID_SAVE_CONFIG = 0xB6; // Save Configuration (Optional)
constexpr auto SID_ASSIGN_FRAME_IDENTIFIER_RANGE = 0xB7; // Assign frame identifier range (Mandatory)

// 4.2.3.5 RSID = Response Service Identifier
// RSID = SID + 0x40
constexpr auto SID_TO_RSID_MASK = 0x40;

// 4.2.3.6 D1 to D5
// up to five data bytes in a node configuration PDU
// interpretation depends on SID / RSID

// ------------------------------------

// DTL standard payload
constexpr auto DTL_NEGATIVE_RESPONSE = 0x7F;

// Negative Response Codes NRC
constexpr auto NRC_GENERAL_REJECT = 0x10;
constexpr auto NRC_SERVICE_NOT_SUPPORTED = 0x11;
constexpr auto NRC_SUBFUNCTION_NOT_SUPPORTED = 0x12;
constexpr auto NRC_INCORRECT_MSG_LENGTH_OR_INVALID_FORMAT = 0x13;
constexpr auto NRC_RESPONSE_TOO_LONG = 0x14;
constexpr auto NRC_BUSY_REPEAT_REQUEST = 0x21;
constexpr auto NRC_CONDITIONS_NOT_CORRECT = 0x22;
constexpr auto NRC_REQUEST_OUT_OF_RANGE = 0x31;
constexpr auto NRC_SECURITY_ACCESS_DENIED = 0x33;
constexpr auto NRC_INVALID_KEY = 0x35;


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
    write(uint8_t(LIN_BREAK));
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
    // https://www.lin-cia.org/fileadmin/microsites/lin-cia.org/resources/documents/LIN_2.2A.pdf
    // 2.6.3 Go To Sleep
    // Request from master to all nodes to go to sleep
    LinMessage[0] = NAD_SLEEP; // NAD = All Slaves
    LinMessage[1] = 0xFF; // PCI
    LinMessage[2] = 0xFF; // SID
    LinMessage[3] = 0xFF; // D1
    LinMessage[4] = 0xFF; // D2
    LinMessage[5] = 0xFF; // D3
    LinMessage[6] = 0xFF; // D4
    LinMessage[7] = 0xFF; // D5

    writeDiagnosticMasterRequest();
}

bool Lin_Interface::writeDiagnosticMasterRequest()
{
    // preserve SID for response evaluation
    uint8_t SID = LinMessage[0];

    writeFrame(FRAME_ID_MASTER_REQUEST, 8);
    bool chkSumValid = readFrame(FRAME_ID_SLAVE_REQUEST);

    // evaluate Negative response
    if (DTL_NEGATIVE_RESPONSE == LinMessage[0])
    {
        Debug.print(0, "writeDiagnosticMasterRequest failed: SID=0x");
        Debug.print(0, LinMessage[1], HEX);
        Debug.print(0, " Error Code=0x");
        Debug.print(0, LinMessage[2], HEX);
        Debug.print(0, " = ");
        switch (LinMessage[2])
        {
            case NRC_GENERAL_REJECT:
                Debug.print(0, "NRC_GENERAL_REJECT");
                break;
            case NRC_SERVICE_NOT_SUPPORTED:
                Debug.print(0, "NRC_SERVICE_NOT_SUPPORTED");
                break;
            case NRC_SUBFUNCTION_NOT_SUPPORTED:
                Debug.print(0, "NRC_SUBFUNCTION_NOT_SUPPORTED");
                break;
            case NRC_INCORRECT_MSG_LENGTH_OR_INVALID_FORMAT:
                Debug.print(0, "NRC_INCORRECT_MSG_LENGTH_OR_INVALID_FORMAT");
                break;
            case NRC_RESPONSE_TOO_LONG:
                Debug.print(0, "NRC_RESPONSE_TOO_LONG");
                break;
            case NRC_BUSY_REPEAT_REQUEST:
                Debug.print(0, "NRC_BUSY_REPEAT_REQUEST");
                break;
            case NRC_CONDITIONS_NOT_CORRECT:
                Debug.print(0, "NRC_CONDITIONS_NOT_CORRECT");
                break;
            case NRC_REQUEST_OUT_OF_RANGE:
                Debug.print(0, "NRC_REQUEST_OUT_OF_RANGE");
                break;
            case NRC_SECURITY_ACCESS_DENIED:
                Debug.print(0, "NRC_SECURITY_ACCESS_DENIED");
                break;
            case NRC_INVALID_KEY:
                Debug.print(0, "NRC_INVALID_KEY");
                break;
        }
        Debug.println(0);
        return false;
    }

    // irregular Response
    if ((SID | SID_TO_RSID_MASK) != LinMessage[0])
    {
        Debug.print(-1, "writeDiagnosticMasterRequest failed irregular");
        return false;
    }

    // success --> (SID | SID_TO_RSID_MASK) == LinMessage[0]
        // TODO: validation on SID == LinMessage[1]

    Debug.print(0, "DiagnosticMaster Response: SID=0x");
    Debug.print(0, LinMessage[1], HEX);
    Debug.print(0, " Error Code=0x");
    Debug.print(0, LinMessage[2], HEX);
    Debug.print(0, " unused=0x");
    Debug.print(0, LinMessage[3], HEX);
    Debug.print(0, " ");
    Debug.print(0, LinMessage[4], HEX);
    Debug.print(0, " ");
    Debug.print(0, LinMessage[5], HEX);
    Debug.println(0, "");

    return true;
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
        case SYNC_IDX:         // sync = LIN_SYNC_FIELD
        case PROTECTED_ID_IDX: // Protected ID
        {
            // discard Sync and PID (send by us)
            uint8_t buffer = HardwareSerial::read();
            // Sync and PID may to be verified here
            if (buffer == LIN_BREAK)
            {
                bytes_received = BREAK_IDX;
            }
            if (buffer == LIN_SYNC_FIELD)
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
        Debug.print(0, "additional byte discarded\n");
    }

    HardwareSerial::end();

    printRxFrame(ChecksumValid, bytes_received, Checksum, ProtectedID, FrameID);

    return ChecksumValid;
} // bool readFrame()

/// @brief write a complete LIN2.0 frame without request of data to the lin-bus
/// @details write LIN Frame (Break, Synk, PID, Data, Checksum) to the Bus, and hope somebody will read this
/// Checksum Calculations regarding LIN 2.0
/// The data of this frame is 'dataLen' long and incuded in the Lin_Interface::LinMessage[] array
/// @param FrameID ID of frame (will be converted to protected ID)
/// @param dataLen count of data within the LinMessage array (containing only the data) should be transmitted
bool Lin_Interface::writeFrame(const uint8_t FrameID, const uint8_t dataLen)
{
    // ---------------------------- write Message
    uint8_t ProtectedID = getProtectedID(FrameID);
    uint8_t chksum = getChecksum(ProtectedID, dataLen);

    startTransmission(ProtectedID); // Break, Sync, PID

    for (int i = 0; i < dataLen; ++i)
    {
        HardwareSerial::write(LinMessage[i]); // Data (array from 1..8)
    }

    HardwareSerial::write(chksum); // ChkSum

    // ---------------------------- read Answer

    // verboseMode = 1;

    // Read Break and discard
    if (!waitForData(TIMEOUT_INTERVAL)) { return false; }
    HardwareSerial::read();
/// TODO: read back of the break needs to be verified

    // Read Sync
    if (!waitForData(TIMEOUT_INTERVAL)) { return false; }
    uint8_t RX_Sync = HardwareSerial::read();

    // Read PID
    if (!waitForData(TIMEOUT_INTERVAL)) { return false; }
    uint8_t RX_ProtectedID = HardwareSerial::read();

    // read DATA + CHKSUM
    int bytes_received = 0;
    for (bytes_received = 0; bytes_received < 8 + 1; ++bytes_received)
    {
        if (!waitForData(TIMEOUT_INTERVAL)) { return false; }
        LinMessage[bytes_received] = HardwareSerial::read();
    }

    // seperate ChkSumRx
    uint8_t ChkSumRx = LinMessage[bytes_received - 1];
    bytes_received--;

    // erase data in buffer, in case a 9th or 10th Byte was received
//    HardwareSerial::flush();
    HardwareSerial::end();

    // use received PID for verification
    uint8_t ChkSumCalc = getChecksum(RX_ProtectedID, bytes_received);

    printTxFrame(FrameID, ProtectedID, RX_Sync, RX_ProtectedID, bytes_received, ChkSumRx, ChkSumCalc, chksum);

    if (ChkSumRx != ChkSumCalc) {
        return false;
    }

    return true;
} // void writeFrame()

/// TODO: function needs to be verified
/// send Frame (Break, Synk, PID, Data, Classic-Checksum) to the Bus
/// Checksum Calculations regarding LIN 1.x
void Lin_Interface::writeFrameClassic(const uint8_t FrameID, const uint8_t dataLen)
{
    uint8_t ProtectedID = getProtectedID(FrameID);
    uint8_t chksum = getChecksum(0x00, dataLen);

    startTransmission(ProtectedID);

    for (int i = 0; i < dataLen; ++i)
    {
        HardwareSerial::write(LinMessage[i]); // Message (array from 1..8)
    }
    HardwareSerial::write(chksum);
    HardwareSerial::flush();

/// TODO: verification of written data (see Lin_Interface::writeFrame)

    HardwareSerial::end();
} // void Lin_Interface::writeFrameClassic

void Lin_Interface::writeFrameClassicNoChecksum(const uint8_t FrameID, const uint8_t dataLen)
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
void Lin_Interface::startTransmission(const uint8_t ProtectedID)
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
    writeSync();                        // Sync
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
    size_t ret = HardwareSerial::write(LIN_BREAK);
    // ensure this is send
    HardwareSerial::flush();
    // restore normal speed
    HardwareSerial::updateBaudRate(baud);
    return ret;
}

size_t Lin_Interface::writeSync()
{
    size_t ret = HardwareSerial::write(LIN_SYNC_FIELD);
    return ret;
}

bool Lin_Interface::waitForData(unsigned long timeout)
{
    unsigned long startMillis = millis();

    while (!HardwareSerial::available())
    {
        if (millis() - startMillis >= timeout)
        {
            // timeout
            return false;
        }
    }

    // data avaliable
    return true;
}

/// get Protected ID by calculating parity bits and combine with Frame ID
/// @param FrameID to be converted
/// @return Protected ID
uint8_t Lin_Interface::getProtectedID(const uint8_t FrameID)
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
uint8_t Lin_Interface::getChecksum(const uint8_t ProtectedID, const uint8_t dataLen)
{
    uint16_t sum = 0x00;

    // consider configuration and reserved frames
    if ((ProtectedID & 0x3F) < FRAME_ID_MASTER_REQUEST)
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

bool Lin_Interface::isChecksumValid(const uint8_t Checksum, const uint8_t ProtectedID, const size_t bytes_received)
{
    constexpr uint8_t CHECKSUM_MASK = 0xFF;
    bool valid = CHECKSUM_MASK == static_cast<uint8_t>(Checksum + static_cast<uint8_t>(~getChecksum(ProtectedID, bytes_received)));

    if (!valid)
    {
        Debug.println(0, "Checksum verification failed.");
    }

    return valid;
}

void Lin_Interface::printRxFrame(bool &ChecksumValid, int8_t bytes_received, uint8_t Checksum, uint8_t ProtectedID, const uint8_t FrameID)
{
    if (Debug.getDebugLevel() >= 0)
    {
        return;
    }
    
    // verify Checksum
    ChecksumValid = (bytes_received > 0) && isChecksumValid(Checksum, ProtectedID, bytes_received);

    Debug.printf(0, " --->>>>>> FID %02Xh        = 55|%02X|", FrameID, ProtectedID);
    for (int i = 0; i < bytes_received; ++i)
    {
        Debug.printf(0, "%02X.", LinMessage[i]);
    }

    if (bytes_received > 0)
    {
        Debug.printf(0, "\b|%02X", Checksum);
        Debug.printf(0, " bytes received %d", bytes_received);

        if (!ChecksumValid)
        {
            Debug.printf(0, " Checksum failed");
        }
    }
    else
    {
        Debug.printf(0, " no bytes received");
    }

    Debug.println(0);
}

void Lin_Interface::printTxFrame(const uint8_t FrameID, uint8_t ProtectedID, uint8_t RX_Sync, uint8_t RX_ProtectedID, int bytes_received, uint8_t ChkSumRx, uint8_t ChkSumCalc, uint8_t chksum)
{
    if (Debug.getDebugLevel() >= 0)
    {
        return;
    }
    
    Debug.printf(0, " <<<<<<--- FID %02Xh (%02X)   = %02X|%02X|", FrameID, ProtectedID, RX_Sync, RX_ProtectedID);
    for (int i = 0; i < bytes_received; ++i)
    {
        Debug.printf(0, "%02X ", LinMessage[i]);
    }
    Debug.printf(2, "\b|%02X", ChkSumRx);

    if (ChkSumRx != ChkSumCalc)
    {
        Debug.printf(0, "\b != ChkSum calc %02Xh| TX %02Xh\n", ChkSumCalc, chksum);
    }
    Debug.println(0);
}
