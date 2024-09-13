#include "diagnostics/sensors/internet_signal_monitor.h"

InternetSignalMonitor::InternetSignalMonitor(const std::string& name, const std::string& topic) : is_error_(false), timer(nullptr)
                        , sensor_data_(1) {
    component_name_ = name;
    topic_and_cb_.first = topic;
    topic_and_cb_.second = std::bind(&InternetSignalMonitor::collectData, this, std::placeholders::_1);
}

void InternetSignalMonitor::collectData(const std::string& protobuf_data) {
    SensorData internet_signal_info;
    if (internet_signal_info.ParseFromString(protobuf_data)) {
        if(sensor_data_ != internet_signal_info.data()) {
            sensor_data_ = internet_signal_info.data();
            if(timer != nullptr) {
                delete timer;
                timer = nullptr;
            }
        }
    } 
    else {
        diagnostics_message_ = "ERROR: Failed to parse Protobuf data.";
    }
}

void InternetSignalMonitor::performDiagnostics() {
    // Example diagnostic check
    if (sensor_data_ == 2) {
        //start timer thread and update the is_error_ after 20s
        timer = new Timer();
        timer->start([&](){is_error_ = true;}, 20 * 1000);
        //diagnostics_message_ = "WARNING: Sensor data nearing lower limit.";
    } 
    else if (sensor_data_ == 0) {
        //start timer thread and update the is_error_ after 10s
        timer = new Timer();
        timer->start([&](){is_error_ = true;}, 10 * 1000);
        //diagnostics_message_ = "ERROR: Sensor data below safe threshold!";
    } 
    else if (sensor_data_ == 1){
        if(timer != nullptr) {
            delete timer;
            timer = nullptr;
        }
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

InternetSignalMonitor::~InternetSignalMonitor() {
    if(timer != nullptr) {
        delete timer;
    }
}