#include "mock_Stream.h"
#include <cstdio>
#include <iostream>
#include <cstring>

#if defined(_WIN32) || defined(_WIN64)
constexpr std::string_view lineBreak = "\r\n";
#else
constexpr std::string_view lineBreak = "\n";
#endif

mock_Stream::mock_Stream() {}
mock_Stream::~mock_Stream() {}

void mock_Stream::begin(unsigned long) {}
void mock_Stream::begin(unsigned long, uint8_t) {}
void mock_Stream::end() {}

int mock_Stream::available() { return 0; }
int mock_Stream::peek() { return -1; }
int mock_Stream::read() { return -1; }

void mock_Stream::flush() {
    std::cout << "mock_Stream::flush()" << std::endl;
}

size_t mock_Stream::write(uint8_t c)
{
    txBuffer.push_back(c);
    return 1;
}

size_t mock_Stream::write(const uint8_t *buffer, size_t size)
{
    for (auto i = 0; i<size; ++i)
    {
        txBuffer.push_back(buffer[i]);
    }   
    return size;
}

size_t mock_Stream::write(unsigned long n)
{
    // TODO: sends not a number only a char
    return write(static_cast<uint8_t>(n));
}

size_t mock_Stream::write(long n)
{
    // TODO: sends not a number only a char
    return write(static_cast<uint8_t>(n));
}

size_t mock_Stream::write(unsigned int n)
{
    // TODO: sends not a number only a char
    return write(static_cast<uint8_t>(n));
}

size_t mock_Stream::write(int n)
{
    // TODO: sends not a number only a char
    return write(static_cast<uint8_t>(n));
}

/// @brief  Prints a cstring --> no 0x00 supported
/// @param pValue c_str, with null-termination
/// @return count of bytes
size_t mock_Stream::print(const char *pValue)
{
    if (!pValue) // avoid nil pointer
    {
        return 0;
    }

    size_t len = strlen(pValue);
    write(reinterpret_cast<const uint8_t *>(pValue), len);
    return len;
}

size_t mock_Stream::print(char pChar)
{
    txBuffer.push_back(static_cast<char>(pChar));
    return 1;
}

size_t mock_Stream::print(const int value, int pBase)
{
    char buffer[100];
    if (pBase == DEC)
    {
        snprintf(buffer, sizeof(buffer), "%d", value);
    }
    else if (pBase == HEX)
    {
        snprintf(buffer, sizeof(buffer), "%X", value);
    }
    else
    {
        return 0; // unknown basis
    }

    auto len = strlen(buffer);
    write(reinterpret_cast<const uint8_t *>(buffer), len);
    return len;
}

size_t mock_Stream::print(const unsigned int pValue, int pBase)
{
    return print(static_cast<int>(pValue), pBase);
}

size_t mock_Stream::print(short value, int pBase)
{
    return print(static_cast<int>(value), pBase);
}

size_t mock_Stream::print(const long value, int pBase)
{
    char buffer[100];
    if (pBase == DEC)
    {
        snprintf(buffer, sizeof(buffer), "%ld", value);
    }
    else if (pBase == HEX)
    {
        snprintf(buffer, sizeof(buffer), "%lX", value);
    }
    else
    {
        return 0; // unknown basis
    }

    auto len = strlen(buffer);
    write(reinterpret_cast<const uint8_t *>(buffer), len);
    return len;
}

size_t mock_Stream::print(const unsigned long pValue, int pBase)
{
    char buffer[100];
    if (pBase == DEC)
    {
        snprintf(buffer, sizeof(buffer), "%lu", pValue);
    }
    else if (pBase == HEX)
    {
        snprintf(buffer, sizeof(buffer), "%lX", pValue);
    }
    else
    {
        return 0; // unknown basis
    }

    auto len = strlen(buffer);
    write(reinterpret_cast<const uint8_t *>(buffer), len);
    return len;
}

size_t mock_Stream::print(const unsigned long long pValue, int pBase)
{
    char buffer[100];
    if (pBase == DEC)
    {
        snprintf(buffer, sizeof(buffer), "%llu", pValue);
    }
    else if (pBase == HEX)
    {
        snprintf(buffer, sizeof(buffer), "%llX", pValue);
    }
    else
    {
        return 0; // unknown basis
    }

    auto len = strlen(buffer);
    write(reinterpret_cast<const uint8_t *>(buffer), len);
    return len;
}

size_t mock_Stream::print(double pValue, int pPrecision)
{
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "%.*f", pPrecision, pValue);
    auto len = strlen(buffer);
    write(reinterpret_cast<const uint8_t *>(buffer), len);
    return len;
}

/// @brief print c_str --> print of 0x00 not supported
/// @param pString c_str
/// @return count of bytes
size_t mock_Stream::println(const char *pString)
{
    auto len = strlen(pString);
    write(reinterpret_cast<const uint8_t *>(pString), len);
    len += println();
    return len;
}

size_t mock_Stream::println(char pChar)
{
    txBuffer.push_back(pChar);
    size_t len = 1;
    len += println();
    return len;
}

size_t mock_Stream::println(unsigned char pValue, int pBase)
{
    size_t written = print(pValue, pBase);
    written += println();
    return written;
}

size_t mock_Stream::println(int pValue, int pBase)
{
    size_t written = print(pValue, pBase);
    written += println();
    return written;
}

size_t mock_Stream::println(unsigned int pValue, int pBase)
{
    size_t written = print(pValue, pBase);
    written += println();
    return written;
}

size_t mock_Stream::println(long pValue, int pBase)
{
    size_t written = print(pValue, pBase);
    written += println();
    return written;
}

size_t mock_Stream::println(unsigned long pValue, int pBase)
{
    size_t written = print(pValue, pBase);
    written += println();
    return written;
}

size_t mock_Stream::println(double pValue, int pPrecision)
{
    size_t written = print(pValue, pPrecision);
    written += println();
    return written;
}

size_t mock_Stream::println()
{
    print(lineBreak.data());
    return lineBreak.length();
}
