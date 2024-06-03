#pragma once

#ifndef Stream_h
#define Stream_h
#include <iostream>
#include <cstdint>


class Stream
{
public:
    Stream() {}
    virtual ~Stream() {}

    // Stream methods
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;

    // Print methods
    virtual size_t write(uint8_t c) = 0;
    virtual size_t write(const uint8_t *s, size_t l) = 0;

    int print(const char *s) {return 0;}
    int print(uint8_t s) {return 0;}
    int print(uint8_t s, int radix) {return 0;}
    int println() {return 0;}
    int println(const char *s) {return 0;}
    int println(uint8_t s) {return 0;}
    int println(uint8_t s, int radix) {return 0;}
};
#endif