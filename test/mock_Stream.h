#pragma once

#include <stddef.h>
#include <stdint.h>
#include <vector>
#include <sstream>
#include <memory>

#include <cstdio>
#include <iostream>
#include <cstring>

#define DEC 10
#define HEX 16

class mock_Stream
{
public:
    mock_Stream();
    virtual ~mock_Stream();

    void begin(unsigned long);
    void begin(unsigned long, uint8_t);
    void end();

    virtual int available();
    virtual int peek();
    virtual int read();
    virtual void flush();

    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buffer, size_t size);

    size_t write(unsigned long n);
    size_t write(long n);
    size_t write(unsigned int n);
    size_t write(int n);

    size_t print(const char* pValue);
    size_t print(char c);
    size_t print(const int pValue, int pBase = DEC);
    size_t print(const short pValue, int pBase = DEC);
    size_t print(const long pValue, int pBase = DEC);
    size_t print(const unsigned int pValue, int pBase = DEC);
    size_t print(const unsigned long pValue, int pBase = DEC);
    size_t print(const unsigned long long pValue, int pBase = DEC);
    size_t print(const double pValue, int pPrecision = 2);

    size_t println(const char* pString);
    size_t println(char pChar);
    size_t println(unsigned char pValue, int pBase = DEC);
    size_t println(int pValue, int pBase = DEC);
    size_t println(unsigned int pValue, int pBase = DEC);
    size_t println(long pValue, int pBase = DEC);
    size_t println(unsigned long pValue, int pBase = DEC);
    size_t println(double pValue, int pPrecision = 2);
    size_t println();

    std::vector<uint8_t> txBuffer;
private:

    size_t print(long pNumber, int pBase, int pNumBits, bool pNewLine);
    size_t print(unsigned long pNumber, int pBase, int pNumBits, bool pNewLine);

    static const int NUMBER_OF_BITS_FOR_LONG = sizeof(long) * 8;
    static const int NUMBER_OF_BITS_FOR_INT = sizeof(int) * 8;
    static const int NUMBER_OF_BITS_FOR_CHAR = sizeof(char) * 8;
};
