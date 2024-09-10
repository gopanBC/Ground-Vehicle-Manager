#include "diagnostics/sensors/battery_level_monitor.h"

BatteryLevelMonitor::BatteryLevelMonitor(const std::string& name, const std::string& topic) : is_error_(false) {
    component_name_ = name;
    topic_and_cb_.first = topic;
    topic_and_cb_.second = std::bind(&BatteryLevelMonitor::collectData, this, std::placeholders::_1);
}

void BatteryLevelMonitor::collectData(const std::string& protobuf_data) {
    SensorData battery_data;
    if (battery_data.ParseFromString(protobuf_data)) {
        sensor_data_ = battery_data.data();
    } 
    else {
        diagnostics_message_ = "ERROR: Failed to parse Protobuf data.";
    }
}

void BatteryLevelMonitor::performDiagnostics() {
    // Example diagnostic check
    if (sensor_data_ < 60) {
	is_error_ = false;
        diagnostics_message_ = "WARNING: Sensor data nearing lower limit.";
    } 
    else if (sensor_data_ < 50) {
        diagnostics_message_ = "ERROR: Sensor data below safe threshold!";
	is_error_ = true;
    } 
    else {
        diagnostics_message_ = "OK: Sensor is working fine.";
	is_error_ = false;
    }
}

std::string BatteryLevelMonitor::getDiagnosticsMessage() const {
    return diagnostics_message_;
}

std::pair<std::string, std::function<void(const std::string&)>> BatteryLevelMonitor::getSubscribeTopics() const {
    return  topic_and_cb_;
}

bool BatteryLevelMonitor::isError() const {
    return is_error_;
}
