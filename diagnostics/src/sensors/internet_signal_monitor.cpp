#include "diagnostics/sensors/internet_signal_monitor.h"

InternetSignalMonitor::InternetSignalMonitor(const std::string& name, const std::string& topic) : is_error_(false) {
    component_name_ = name;
    topic_and_cb_.first = topic;
    topic_and_cb_.second = std::bind(&InternetSignalMonitor::collectData, this, std::placeholders::_1);
}

void InternetSignalMonitor::collectData(const std::string& protobuf_data) {
    SensorData internet_signal_info;
    if (internet_signal_info.ParseFromString(protobuf_data)) {
        sensor_data_ = internet_signal_info.data();
    } 
    else {
        diagnostics_message_ = "ERROR: Failed to parse Protobuf data.";
    }
}

void InternetSignalMonitor::performDiagnostics() {
    // Example diagnostic check
    if (sensor_data_ == 2) {
        //start timer thread and update the is_error_ after 20s
	    //is_error_ = false;
        //diagnostics_message_ = "WARNING: Sensor data nearing lower limit.";
    } 
    else if (sensor_data_ == 0) {
        //start timer thread and update the is_error_ after 10s
        //diagnostics_message_ = "ERROR: Sensor data below safe threshold!";
	    //is_error_ = true;
    } 
    else if (sensor_data_ == 1){
        diagnostics_message_ = "OK: Sensor is working fine.";
	    is_error_ = false;
    }
}

std::string InternetSignalMonitor::getDiagnosticsMessage() const {
    return diagnostics_message_;
}

std::pair<std::string, std::function<void(const std::string&)>> InternetSignalMonitor::getSubscribeTopics() const {
    return  topic_and_cb_;
}

bool InternetSignalMonitor::isError() const {
    return is_error_;
}
