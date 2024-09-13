
#include "diagnostics/sensors/internet_signal_monitor.h"
#include "../msg/sensor_data.pb.h"

#include <chrono>

#include <gtest/gtest.h>

class InternetSignalMonitorTest : public ::testing::Test {
protected:
    InternetSignalMonitor* internet_signal_monitor;

    // Setup function, called before each test
    void SetUp() override {
        internet_signal_monitor = new InternetSignalMonitor("InternetSignalMonitor", "/gps/diagnostics");
    }

    // TearDown function, called after each test
    void TearDown() override {
        delete internet_signal_monitor;
    }
};


// Test for sensor below error threshold
TEST_F(InternetSignalMonitorTest, InternetSignalFine) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData internet_signal_data;
    internet_signal_data.set_data(0);  // This should trigger an error

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(internet_signal_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    internet_signal_monitor->collectData(serialized_data);

    // Perform diagnostics
    internet_signal_monitor->performDiagnostics();

    EXPECT_FALSE(internet_signal_monitor->isError());  // No Error should be triggered
}

// Test for sensor below error threshold
TEST_F(InternetSignalMonitorTest, InternetSignalLow) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData internet_signal_data;
    internet_signal_data.set_data(2);  // This should trigger an error

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(internet_signal_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    internet_signal_monitor->collectData(serialized_data);

    // Perform diagnostics
    internet_signal_monitor->performDiagnostics();

    //validating gps accuracy monitor logic
    EXPECT_FALSE(internet_signal_monitor->isError()); //no error before 20s 
    std::this_thread::sleep_for(std::chrono::milliseconds(20001)); //error is generated after 20s
    EXPECT_TRUE(internet_signal_monitor->isError());  // Error should be triggered
}

// Test for sensor above safe threshold (OK case)
TEST_F(InternetSignalMonitorTest, Unconnected) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData internet_signal_data;
    internet_signal_data.set_data(0);  // This should be OK

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(internet_signal_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    internet_signal_monitor->collectData(serialized_data);

    // Perform diagnostics
    internet_signal_monitor->performDiagnostics();

      //validating gps accuracy monitor logic
    EXPECT_FALSE(internet_signal_monitor->isError()); //no error before 10s 
    std::this_thread::sleep_for(std::chrono::milliseconds(10001)); //error is generated after 10s
    EXPECT_TRUE(internet_signal_monitor->isError());  // Error should be triggered
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
