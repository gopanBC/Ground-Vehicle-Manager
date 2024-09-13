
#include "diagnostics/sensors/gps_accuracy_monitor.h"
#include "../msg/sensor_data.pb.h"

#include <chrono>

#include <gtest/gtest.h>

class GPSAccuracyMonitorTest : public ::testing::Test {
protected:
    GPSAccuracyMonitor* gps_monitor;

    // Setup function, called before each test
    void SetUp() override {
        gps_monitor = new GPSAccuracyMonitor("GPSAccuracyMonitor", "/gps/diagnostics");
    }

    // TearDown function, called after each test
    void TearDown() override {
        delete gps_monitor;
    }
};


// Test for sensor below error threshold
TEST_F(GPSAccuracyMonitorTest, GPSAccuracyFine) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData gps_data;
    gps_data.set_data(70.0f);  // This should trigger an error

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(gps_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    gps_monitor->collectData(serialized_data);

    // Perform diagnostics
    gps_monitor->performDiagnostics();

    EXPECT_FALSE(gps_monitor->isError());  // Error should be triggered
}

// Test for sensor below error threshold
TEST_F(GPSAccuracyMonitorTest, GPSAccuracyFailure) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData gps_data;
    gps_data.set_data(201.0f);  // This should trigger an error

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(gps_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    gps_monitor->collectData(serialized_data);

    // Perform diagnostics
    gps_monitor->performDiagnostics();

    //validating gps accuracy monitor logic
    EXPECT_FALSE(gps_monitor->isError()); //no error before 15s 
    std::this_thread::sleep_for(std::chrono::milliseconds(15001)); //error is generated after 15s
    EXPECT_TRUE(gps_monitor->isError());  // Error should be triggered
}

// Test for sensor above safe threshold (OK case)
TEST_F(GPSAccuracyMonitorTest, GPSAccuracyAtBoarder) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData gps_data;
    gps_data.set_data(200.0f);  // This should be OK

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(gps_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    gps_monitor->collectData(serialized_data);

    // Perform diagnostics
    gps_monitor->performDiagnostics();

    EXPECT_FALSE(gps_monitor->isError());  // No error should be triggered
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
