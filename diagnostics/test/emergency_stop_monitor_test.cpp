
#include "diagnostics/sensors/emergency_stop_monitor.h"
#include "../msg/sensor_data.pb.h"

#include <gtest/gtest.h>

class EmergencyStopMonitorTest : public ::testing::Test {
protected:
    EmergencyStopMonitor* estop_monitor;

    // Setup function, called before each test
    void SetUp() override {
        estop_monitor = new EmergencyStopMonitor("EmergencyStopMonitor", "/estop/diagnostics");
    }

    // TearDown function, called after each test
    void TearDown() override {
        delete estop_monitor;
    }
};

// Test for valid Protobuf parsing
TEST_F(EmergencyStopMonitorTest, ValidProtobufParsing) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData battery_data;
    battery_data.set_data(0);

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(battery_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    estop_monitor->collectData(serialized_data);

    // Perform diagnostics
    estop_monitor->performDiagnostics();

    EXPECT_FALSE(estop_monitor->isError());  // No error expected since 0
}

// Test for Protobuf parsing failure
TEST_F(EmergencyStopMonitorTest, InvalidProtobufParsing) {
    // Invalid data (non-Protobuf string)
    std::string invalid_data = "Invalid data string";

    // Feed the invalid data to the BatteryLevelMonitor
    estop_monitor->collectData(invalid_data);

    // Perform diagnostics
    estop_monitor->performDiagnostics();

    EXPECT_FALSE(estop_monitor->isError());  // No error should be triggered on parsing failure
}


// Test for sensor below error threshold
TEST_F(EmergencyStopMonitorTest, SensorErrorLevel) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData battery_data;
    battery_data.set_data(1);  // This should trigger an error

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(battery_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    estop_monitor->collectData(serialized_data);

    // Perform diagnostics
    estop_monitor->performDiagnostics();

    EXPECT_TRUE(estop_monitor->isError());  // Error should be triggered
}

// Test that the correct topic and callback are returned
TEST_F(EmergencyStopMonitorTest, SubscribeTopicAndCallback) {
    auto topic_and_callback = estop_monitor->getSubscribeTopics();

    // Check if the correct topic is returned
    EXPECT_EQ(topic_and_callback.first, "/estop/diagnostics");

    // Check if the callback is valid (non-null)
    EXPECT_TRUE(topic_and_callback.second != nullptr);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
