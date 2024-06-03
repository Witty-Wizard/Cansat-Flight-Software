#pragma once

#ifndef Stream_h
#define Stream_h
#include <iostream>
#include <cstdint>

/**
 * @brief Abstract base class for stream-based input/output operations.
 *
 * This class defines a set of virtual methods for stream-based input/output operations.
 * It serves as an interface for classes that implement specific types of streams,
 * such as serial communication or file I/O.
 */
class Stream
{
public:
    /**
     * @brief Constructor for the Stream class.
     */
    Stream() {}

    /**
     * @brief Destructor for the Stream class.
     */
    virtual ~Stream() {}

    // Stream methods
    /**
     * @brief Checks if data is available to read from the stream.
     *
     * @return The number of bytes available to read, or 0 if no data is available.
     */
    virtual int available() = 0;

    /**
     * @brief Reads a byte of data from the stream.
     *
     * @return The byte read from the stream, or -1 if no data is available.
     */
    virtual int read() = 0;

    /**
     * @brief Peeks at the next byte of data in the stream without consuming it.
     *
     * @return The next byte of data in the stream, or -1 if no data is available.
     */
    virtual int peek() = 0;

    /**
     * @brief Flushes any buffered data in the stream.
     *
     * This method may be overridden by derived classes to perform any necessary flushing
     * of buffered data to the underlying stream.
     */
    virtual void flush() = 0;

    // Print methods
    /**
     * @brief Writes a single byte of data to the stream.
     *
     * @param c The byte to write.
     * @return The number of bytes written, which is always 1.
     */
    virtual size_t write(uint8_t c) = 0;

    /**
     * @brief Writes a block of data to the stream.
     *
     * @param s Pointer to the data to write.
     * @param l Length of the data to write.
     * @return The number of bytes written.
     */
    virtual size_t write(const uint8_t *s, size_t l) = 0;

    // Additional print methods (not pure virtual)
    int print(const char *s) {return 0;}
    int print(uint8_t s) {return 0;}
    int print(uint8_t s, int radix) {return 0;}
    int println() {return 0;}
    int println(const char *s) {return 0;}
    int println(uint8_t s) {return 0;}
    int println(uint8_t s, int radix) {return 0;}
};
#endif
