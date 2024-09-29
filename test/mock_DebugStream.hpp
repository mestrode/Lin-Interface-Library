#pragma once

class mock_DebugStream : public Stream {
public:
    size_t write(uint8_t c) override {
        std::cout.put(c);
        return 1;
    }

    size_t write(const uint8_t *buffer, size_t size)
    {
        for (auto i = 0; i<size; ++i)
        {
            write(buffer[i]);
        }   
        return size;
    }

    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
};
