#ifdef NATIVE_ENV
#include <unity.h>
#include <string>
#include "mock_serial.h" // Include your mock serial class header file

// Define a global variable to hold the test stream
std::string testBuffer;
Serial testStream(testBuffer);

void setUp() {
    // Initialize the test buffer and stream before each test
    testBuffer.clear();
}

void tearDown() {
    // Clean up after each test (if needed)
}

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

int runUnityTests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_crsfCommunication);
    return UNITY_END();
}

int main(void) { return runUnityTests(); }
#endif