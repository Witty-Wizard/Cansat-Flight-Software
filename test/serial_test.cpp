#ifdef NATIVE_ENV
#include "mock_serial.h" // Include your mock serial class header file
#include <string>
#include <unity.h>

// Define a global variable to hold the test stream
std::string testBuffer;
Serial testStream(testBuffer);

/**
 * @brief Set up function called before each test case.
 */
void setUp() {
  // Initialize the test buffer and stream before each test
  testBuffer.clear();
}

/**
 * @brief Tear down function called after each test case.
 */
void tearDown() {
  // Clean up after each test (if needed)
}

/**
 * @brief Test case for testing CRSF communication.
 */
void test_crsfCommunication() {
  // Write data to the test stream
  testStream.write('H');
  testStream.write('e');
  testStream.write('l');
  testStream.write('l');
  testStream.write('o');

  // Check if the data is received correctly
  TEST_ASSERT_EQUAL_INT('H', testStream.read());
  TEST_ASSERT_EQUAL_INT('e', testStream.read());
  TEST_ASSERT_EQUAL_INT('l', testStream.read());
  TEST_ASSERT_EQUAL_INT('l', testStream.read());
  TEST_ASSERT_EQUAL_INT('o', testStream.read());
}

/**
 * @brief Main function to run Unity tests.
 *
 * @return Returns the result of Unity tests.
 */
int runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(test_crsfCommunication);
  return UNITY_END();
}

/**
 * @brief Entry point of the program.
 *
 * @return Returns the result of Unity tests.
 */
int main(void) { return runUnityTests(); }
#endif
