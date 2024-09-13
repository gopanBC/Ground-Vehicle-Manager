
#include <random>

#include <gtest/gtest.h>
#include "diagnostics/sensors/battery_level_monitor.h"
#include "../msg/sensor_data.pb.h"

float generateRandomFloat() {
    // Create a random device and seed the random number generator
    std::random_device rd;
    std::mt19937 gen(rd());  // Mersenne Twister generator
    std::uniform_real_distribution<> dis(0.0, 100.0);  // Range [0, 100]

    // Generate the random float
    double random_float = dis(gen);
    return random_float;
}

class BatteryLevelMonitorTest : public ::testing::Test {
protected:
    BatteryLevelMonitor* battery_monitor;

    // Setup function, called before each test
    void SetUp() override {
        battery_monitor = new BatteryLevelMonitor("BatteryLevelMonitor", "/battery/diagnostics");
    }

    // TearDown function, called after each test
    void TearDown() override {
        delete battery_monitor;
    }
};

// Test for valid Protobuf parsing
TEST_F(BatteryLevelMonitorTest, ValidProtobufParsing) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData battery_data;
    battery_data.set_data(55.0f);

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(battery_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    battery_monitor->collectData(serialized_data);

    // Perform diagnostics
    battery_monitor->performDiagnostics();

    // Check diagnostics message
    //EXPECT_EQ(battery_monitor->getDiagnosticsMessage(), "WARNING: Sensor data nearing lower limit.");
    EXPECT_FALSE(battery_monitor->isError());  // No error expected since 55 > 50
}

// Test for Protobuf parsing failure
TEST_F(BatteryLevelMonitorTest, InvalidProtobufParsing) {
    // Invalid data (non-Protobuf string)
    std::string invalid_data = "Invalid data string";

    // Feed the invalid data to the BatteryLevelMonitor
    battery_monitor->collectData(invalid_data);

    // Perform diagnostics
    battery_monitor->performDiagnostics();

    // Check diagnostics message for parsing failure
    //EXPECT_EQ(battery_monitor->getDiagnosticsMessage(), "ERROR: Failed to parse Protobuf data.");
    EXPECT_FALSE(battery_monitor->isError());  // No error should be triggered on parsing failure
}

// Test for sensor below warning level
TEST_F(BatteryLevelMonitorTest, SensorWarningLevel) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData battery_data;
    battery_data.set_data(59.0f);  // This should trigger a warning

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(battery_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    battery_monitor->collectData(serialized_data);

    // Perform diagnostics
    battery_monitor->performDiagnostics();

    // Check diagnostics message
    //EXPECT_EQ(battery_monitor->getDiagnosticsMessage(), "WARNING: Sensor data nearing lower limit.");
    EXPECT_FALSE(battery_monitor->isError());  // Warning level, but no error
}

// Test for sensor below error threshold
TEST_F(BatteryLevelMonitorTest, SensorErrorLevel) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData battery_data;
    battery_data.set_data(45.0f);  // This should trigger an error

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(battery_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    battery_monitor->collectData(serialized_data);

    // Perform diagnostics
    battery_monitor->performDiagnostics();

    // Check diagnostics message
    //EXPECT_EQ(battery_monitor->getDiagnosticsMessage(), "ERROR: Sensor data below safe threshold!");
    EXPECT_TRUE(battery_monitor->isError());  // Error should be triggered
}

// Test for sensor below error threshold
TEST_F(BatteryLevelMonitorTest, SensorErrorAtExactLevel) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData battery_data;
    battery_data.set_data(50.0f);  // This should trigger an error

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(battery_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    battery_monitor->collectData(serialized_data);

    // Perform diagnostics
    battery_monitor->performDiagnostics();

    //validating battery level logic
    EXPECT_TRUE(battery_monitor->isError());  // Error should be triggered
}

// Test with mock data
TEST_F(BatteryLevelMonitorTest, MockDataTest) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData battery_data;
    std::size_t data = 100;
    while(data != 51) {
        --data;
        std::this_thread::sleep_for(std::chrono::milliseconds(612));
    }

    battery_data.set_data(data);  // This should trigger an error

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(battery_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    battery_monitor->collectData(serialized_data);

    // Perform diagnostics
    battery_monitor->performDiagnostics();

    EXPECT_FALSE(battery_monitor->isError());  // Error should be triggered

    battery_data.set_data(49.0f);
    battery_data.SerializeToString(&serialized_data);
    battery_monitor->collectData(serialized_data);
    battery_monitor->performDiagnostics();

    //validating battery level logic
    EXPECT_TRUE(battery_monitor->isError());  // Error should be triggered
}

// Test for sensor below error threshold
TEST_F(BatteryLevelMonitorTest, RandomLevelInput) {

    std::size_t i = 0;
    while(i < 10) {
        // Create a SensorData Protobuf message and set sensor data
        SensorData battery_data;
        auto mock_data = generateRandomFloat();
        battery_data.set_data(mock_data);  // This should trigger an error

        // Serialize the message to string
        std::string serialized_data;
        ASSERT_TRUE(battery_data.SerializeToString(&serialized_data));

        // Feed the serialized data to the BatteryLevelMonitor
        battery_monitor->collectData(serialized_data);

        // Perform diagnostics
        battery_monitor->performDiagnostics();

        if(mock_data <= 50) {
            EXPECT_TRUE(battery_monitor->isError());  // Error should be triggered
        }
        else {
            EXPECT_FALSE(battery_monitor->isError());
        }
        ++i;
    }
}

// Test for sensor above safe threshold (OK case)
TEST_F(BatteryLevelMonitorTest, SensorOK) {
    // Create a SensorData Protobuf message and set sensor data
    SensorData battery_data;
    battery_data.set_data(65.0f);  // This should be OK

    // Serialize the message to string
    std::string serialized_data;
    ASSERT_TRUE(battery_data.SerializeToString(&serialized_data));

    // Feed the serialized data to the BatteryLevelMonitor
    battery_monitor->collectData(serialized_data);

    // Perform diagnostics
    battery_monitor->performDiagnostics();

    // Check diagnostics message
    //EXPECT_EQ(battery_monitor->getDiagnosticsMessage(), "OK: Sensor is working fine.");
    EXPECT_FALSE(battery_monitor->isError());  // No error should be triggered
}

// Test that the correct topic and callback are returned
TEST_F(BatteryLevelMonitorTest, SubscribeTopicAndCallback) {
    auto topic_and_callback = battery_monitor->getSubscribeTopics();

    // Check if the correct topic is returned
    EXPECT_EQ(topic_and_callback.first, "/battery/diagnostics");

    // Check if the callback is valid (non-null)
    EXPECT_TRUE(topic_and_callback.second != nullptr);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
