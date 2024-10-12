#pragma once

#include <cstdint>
#include <algorithm>
#include <vector>
#include <array>

union PDU {
public:
    // 4.2.3.2 NAD = Node Address
    enum NAD : uint8_t {
    // 0     (0x00) = Reserved for go to sleep command, see Section 2.6.3
        SLEEP = 0x00,
    // 1-125 (0x01-0x7D) = Slave Node Adress (NAD)
    // 126   (0x7E) = Functional node address (functional NAD), only used for diagnostics (using the transport layer)
        FUNCTIONAL = 0x7E,
    // 127   (0x7F) = Slave node address broadcast (broadcast NAD)
        BROADCAST = 0x7F  // = wildcard
    // 128-255   (0x80-0xFF) = Free usage
    };

    // 3.2.1.3 PCI
    // 4.2.3.3 PCI = Protocol Control Information (= Length of Message)
    static constexpr uint8_t MASK_PCI_TYPE = 0xF0;
    static constexpr uint8_t MASK_PCI_LEN = 0x0F; // SF and FF
    static constexpr uint8_t MASK_PCI_SN = 0x0F; // CF

    enum class PCI_Type : uint8_t {
        SINGLE = 0x00,
        FIRST = 0x10,
        CONSECUTIVE = 0x20
    };

    static constexpr size_t dataLenSingle = 6;
    static constexpr size_t dataLenFirst = dataLenSingle-1;
    static constexpr size_t dataLenConsecutive = dataLenSingle;
    static constexpr uint8_t fillByte = 0xFF;

    struct Common {
        uint8_t NAD;
        uint8_t PCI; // PCI data in high nibble
        std::array<uint8_t, dataLenSingle> framedata;
    };

    // Single Frame, payload fits into the single PDU
    class SingleFrame {
    private:
        uint8_t NAD;
        uint8_t PCI_LEN; // B7..B4 = type (SF=0, FF=1, CF=2)
                         // B3..B0 = LEN of data bytes
        std::array<uint8_t, dataLenSingle> DATA;

        /// @brief encodes PCI and LEN, does not adjust DATA
        /// @param len of payload encoded within frame
        inline void setLen(std::size_t len)
        {
            PCI_LEN = static_cast<uint8_t>(PCI_Type::SINGLE) | static_cast<uint8_t>(len & MASK_PCI_LEN);
        }

    public:
        /// @brief returns count of bytes encoded within frame
        /// @return len of data
        inline std::size_t getLen() const
        {
            return PCI_LEN & PDU::MASK_PCI_LEN;
        }

        /// @brief retuns payload of frame
        /// @return data vector (0..6 bytes)
        std::vector<uint8_t> getData() const
        {
            size_t l = getLen();
            return {DATA.begin(), DATA.begin() + l};
        }

        /// @brief copy data into single PDU, encode correct value to LEN
        /// @param new_data source vector, must not exceed 6 bytes
        /// @return count of encoded bytes
        size_t setDataAndLen(const std::vector<uint8_t>& payload)
        {
            int len = std::min(DATA.size(), payload.size());
            setLen(len);
            std::copy_n(payload.begin(), len, DATA.begin());
            if (len < DATA.size()) {
                std::fill(
                    DATA.begin() + len,
                    DATA.end(),
                    fillByte
                );
            }
            return len;
        }
    };

    // First Frame, when payload does not fit into a single PDU, musst be followed by Consecutive Frames
    class FirstFrame {
    protected:
        uint8_t NAD;
        uint8_t PCI_LEN_MSB; // B7..B4 = type (SF=0, FF=1, CF=2)
                             // B3..B0 = LEN/256 of databytes
        uint8_t LEN_LSB;
        std::array<uint8_t, dataLenFirst> DATA;

    public:
        /// @brief encode length of complete payload for announcement
        /// @param len of whole payload - only first 5 bytes are encoded in first frame
        inline void setLen(const std::size_t len)
        {
            PCI_LEN_MSB = static_cast<uint8_t>(PCI_Type::FIRST) | static_cast<uint8_t>(len >> 8);
            LEN_LSB = static_cast<uint8_t>(len & 0xFF);
        }

        /// @brief decodes length of announced payload (will exceed data of first frame)
        /// @return len of whole payload - only first 5 bytes are encoded in first frame
        inline std::size_t getLen() const
        {
            return (PCI_LEN_MSB & MASK_PCI_LEN) << 8 | LEN_LSB;
        }

        /// @brief copy data into first PDU, does not encode LEN
        /// @param new_data source vector, must exceed 6 bytes
        /// @return count of encoded bytes
        size_t setData(const std::vector<uint8_t>& new_data)
        {
            // according to spec: every valid FirstFrame does have
            // - full use of DATA byes: len = 5
            // - no need for fill bytes
            // --> no verification of sizeof(new_data)
            std::copy_n(new_data.begin(), DATA.size(), DATA.begin());
            return DATA.size();
        }

        /// @brief returns the part of the payload that is coded in the first frame
        /// @return vector (5 bytes)
        std::vector<uint8_t> getData() const
        {
            return {DATA.begin(), DATA.end()};
        }
    };

    // Consecutive Frames follows on FF, when payload does not fit into a single PDU
    class ConsecutiveFrame {
    protected:
        uint8_t NAD;
        uint8_t PCI_SN; // B7..B4 = type (SF=0, FF=1, CF=2)
                        // B3..B0 = Sequence Number % 0x0F (wraps around)
        std::array<uint8_t, dataLenConsecutive> DATA;

    public:
        /// @brief encodes the sequence number of this frame
        /// @param sequenceNumber 
        inline void setSequenceNumber(const uint8_t sequenceNumber)
        {
            PCI_SN = static_cast<uint8_t>(PCI_Type::CONSECUTIVE) | (sequenceNumber & MASK_PCI_SN);
        }

        /// @brief returns the part of the sequence number encoded within the frame
        /// @return secuenceNumber - consider: limited to lower 4 bits, will wrap around
        inline int getSequenceNumber() const
        {
            return PCI_SN & PDU::MASK_PCI_SN;
        }

        /// @brief check if sequence number of current frame matches the expected value
        /// @param expectedSequenceNumber to be verified
        /// @return bool: expectation fulfilled
        inline bool verifySequenceNumber(int expectedSequenceNumber)
        {
            auto SN = expectedSequenceNumber & PDU::MASK_PCI_SN;
            return (getSequenceNumber() == SN);
        }

        /// @brief encodes (up to 6 bytes of) the payload within a consecutive frame
        /// @param payload source, may exceed capacity of frame
        /// @param offset first n bytes will be skipped
        /// @return count of encoded bytes 
        size_t setData(const std::vector<uint8_t>& payload, const int offset = 0)
        {
            int len = std::min(DATA.size(), payload.size() - offset);
            std::copy_n(payload.begin() + offset, len, DATA.begin());
            if (len < DATA.size()) {
                std::fill(
                    DATA.begin() + len,
                    DATA.end(),
                    fillByte
                );
            }
            return len;
        }

        // returns 
        std::vector<uint8_t> getData() const
        {
            return {DATA.begin(), DATA.end()};
        }

        std::vector<uint8_t> getData(size_t len) const
        {
            size_t l = std::min(DATA.size(), len);
            return {DATA.begin(), DATA.begin() + l};
        }
    };

    // static_assert(offsetof(SingleFrame, PCI_LEN) == offsetof(FirstFrame, PCI_LEN_MSB),
    //     "Data position of SingleFrame::PCI_LEN does not match FirstFrame::PCI_LEN256");
    // static_assert(offsetof(SingleFrame, PCI_LEN) == offsetof(ConsecutiveFrame, PCI_SN),
    //     "Data position of SingleFrame::PCI_LEN does not match PDU::ConsecutiveFrame::PCI_SN");

    static_assert(sizeof(PDU::Common) == sizeof(PDU::SingleFrame),
        "Size of PDU::Common does not match PDU::SingleFrame");
    static_assert(sizeof(PDU::Common) == sizeof(PDU::FirstFrame),
        "Size of PDU::Common does not match PDU::FirstFrame");
    static_assert(sizeof(PDU::Common) == sizeof(PDU::ConsecutiveFrame),
        "Size of PDU::Common does not match PDU::ConsecutiveFrame");

    Common common;
    SingleFrame singleFrame;
    FirstFrame firstFrame;
    ConsecutiveFrame consecutiveFrame;

    PDU() {}

    PDU(uint8_t NAD, uint8_t PCI, std::array<uint8_t, dataLenSingle> otherBytes):
        common{NAD, PCI, otherBytes}
    {}

    ~PDU() = default;

    const void setNAD(uint8_t nad)
    {
        common.NAD = nad;
    }

    const uint8_t getNad() const
    {
        return common.NAD;
    }

    /// @brief decode type of PDU - can only handel SingleFrame, FirstFrame, ConsecutiveFrame; NOT sleepCmd
    /// @return type of PDU
    const PCI_Type getType() const
    {
        return static_cast<PCI_Type>(common.PCI & PDU::MASK_PCI_TYPE);
    }

    /// @brief conversion of PDU to vector of uint8_t
    /// @return
    std::vector<uint8_t> asVector() const
    {
        const uint8_t* dataPtr = reinterpret_cast<const uint8_t*>(&common);
        return std::vector<uint8_t>(dataPtr, dataPtr + sizeof(Common));
    }

    static PDU getSleepCmd()
    {
        // https://www.lin-cia.org/fileadmin/microsites/lin-cia.org/resources/documents/LIN_2.2A.pdf
        // 2.6.3 Go To Sleep
        // Request from master to all nodes to go to sleep
        PDU sleepCmd;
        sleepCmd.common.NAD = NAD::SLEEP;
        sleepCmd.common.PCI = fillByte;
        sleepCmd.common.framedata.fill(fillByte);
        return sleepCmd;
    }
};
