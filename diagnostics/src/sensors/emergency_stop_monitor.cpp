#include "diagnostics/sensors/emergency_stop_monitor.h"

EmergencyStopMonitor::EmergencyStopMonitor(const std::string& name, const std::string& topic) : is_error_(false) {
    component_name_ = name;
    topic_and_cb_.first = topic;
    topic_and_cb_.second = std::bind(&EmergencyStopMonitor::collectData, this, std::placeholders::_1);
}

void EmergencyStopMonitor::collectData(const std::string& protobuf_data) {
    SensorData emergency_button_info;
    if (emergency_button_info.ParseFromString(protobuf_data)) {
        sensor_data_ = emergency_button_info.data();
    } 
    else {
        diagnostics_message_ = "ERROR: Failed to parse Protobuf data.";
    }
}

void EmergencyStopMonitor::performDiagnostics() {
    if (sensor_data_ == 0) {
	    is_error_ = false;
        diagnostics_message_ = "OK: Emergency Button is not Pressed.";
    } 
    else if (sensor_data_ > 0) {
        diagnostics_message_ = "ERROR: Emergency Button is Pressed!";
	    is_error_ = true;
    } 
}

std::string EmergencyStopMonitor::getDiagnosticsMessage() const {
    return diagnostics_message_;
}

std::pair<std::string, std::function<void(const std::string&)>> EmergencyStopMonitor::getSubscribeTopics() const {
    return  topic_and_cb_;
}

bool EmergencyStopMonitor::isError() const {
    return is_error_;
}
