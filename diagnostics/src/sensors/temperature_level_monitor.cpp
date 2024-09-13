#include "diagnostics/sensors/temperature_level_monitor.h"

TemperatureLevelMonitor::TemperatureLevelMonitor(const std::string& name, const std::string& topic) : is_error_(false), sensor_data_(30.0) {
    component_name_ = name;
    topic_and_cb_.first = topic;
    topic_and_cb_.second = std::bind(&TemperatureLevelMonitor::collectData, this, std::placeholders::_1);
}

void TemperatureLevelMonitor::collectData(const std::string& protobuf_data) {
    SensorData op_temperature_data;
    if (op_temperature_data.ParseFromString(protobuf_data)) {
        sensor_data_ = op_temperature_data.data();
    } 
    else {
        diagnostics_message_ = "ERROR: Failed to parse Protobuf dpata.";
    }
}

void TemperatureLevelMonitor::performDiagnostics() {
    if (sensor_data_ < 55) {
	    is_error_ = false;
        diagnostics_message_ = "OK: Temperature if the system is below the threshold!";
    } 
    else if (sensor_data_ >= 55) {
        diagnostics_message_ = "ERROR: Temperature of the system is High!";
	    is_error_ = true;
    } 
}

std::string TemperatureLevelMonitor::getDiagnosticsMessage() const {
    return diagnostics_message_;
}

std::pair<std::string, std::function<void(const std::string&)>> TemperatureLevelMonitor::getSubscribeTopics() const {
    return  topic_and_cb_;
}

bool TemperatureLevelMonitor::isError() const {
    return is_error_;
}
