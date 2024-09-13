
#include <random>

#include <gtest/gtest.h>
#include "diagnostics/sensors/temperature_level_monitor.h"
#include "../msg/sensor_data.pb.h"


class TemperatureLevelMonitorTest : public ::testing::Test {
protected:
    TemperatureLevelMonitor* temperature_monitor;

    // Setup function, called before each test
    void SetUp() override {
        temperature_monitor = new TemperatureLevelMonitor("BatteryLevelMonitor", "/battery/diagnostics");
    }

    // TearDown function, called after each test
    void TearDown() override {
        delete temperature_monitor;
    }
};


// Test for sensor below error threshold
TEST_F(TemperatureLevelMonitorTest, SensorErrorLevel) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData temperature_data;
    temperature_data.set_data(70.0f);  // This should trigger an error

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(temperature_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    temperature_monitor->collectData(serialized_data);

    // Perform diagnostics
    temperature_monitor->performDiagnostics();

    EXPECT_TRUE(temperature_monitor->isError());  // Error should be triggered
}

// Test for sensor below error threshold
TEST_F(TemperatureLevelMonitorTest, SensorErrorAtExactLevel) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData temperature_data;
    temperature_data.set_data(55.0f);  // This should trigger an error

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(temperature_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    temperature_monitor->collectData(serialized_data);

    // Perform diagnostics
    temperature_monitor->performDiagnostics();

    //validating battery level logic
    EXPECT_TRUE(temperature_monitor->isError());  // Error should be triggered
}

// Test for sensor above safe threshold (OK case)
TEST_F(TemperatureLevelMonitorTest, SensorOK) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData temperature_data;
    temperature_data.set_data(40.0f);  // This should be OK

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(temperature_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    temperature_monitor->collectData(serialized_data);

    // Perform diagnostics
    temperature_monitor->performDiagnostics();

    // Check diagnostics message
    //EXPECT_EQ(temperature_monitor->getDiagnosticsMessage(), "OK: Sensor is working fine.");
    EXPECT_FALSE(temperature_monitor->isError());  // No error should be triggered
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
