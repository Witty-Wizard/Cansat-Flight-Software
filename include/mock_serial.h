#pragma once
#ifdef NATIVE_ENV
#include "Stream.h"
#include <cstdint>
#include <string>

/**
 * @brief Mock serial port class for testing purposes.
 *
 * This class mimics the behavior of a serial port by using a string buffer.
 * It allows testing of code that interacts with serial communication without
 * requiring physical hardware.
 */
class Serial : public Stream {
public:
  /**
   * @brief Constructor for the Serial class.
   *
   * @param s Reference to a string buffer to store the data sent to the "serial port".
   */
  Serial(std::string &s) : buf(s), position(0) {}

  // Stream methods
  int available() { return buf.length() - position; }
  int read() { return position < buf.length() ? buf[position++] : -1; }
  int peek() { return position < buf.length() ? buf[position] : -1; }
  void flush() {}

  // Print methods
  /**
   * @brief Writes a single byte of data to the "serial port" buffer.
   *
   * @param c The byte to write.
   * @return The number of bytes written, which is always 1.
   */
  size_t write(uint8_t c) {
    buf += (char)c;
    return 1;
  }

  /**
   * @brief Writes a block of data to the "serial port" buffer.
   *
   * @param c Pointer to the data to write.
   * @param l Length of the data to write.
   * @return The number of bytes written, which is always equal to the length of the data.
   */
  size_t write(const uint8_t *c, size_t l) {
    for (int i = 0; i < l; buf += *c, i++, c++)
      ;
    return l;
  }

private:
  std::string &buf; ///< Reference to the string buffer.
  unsigned int position; ///< Current position in the buffer.
};
#endif
