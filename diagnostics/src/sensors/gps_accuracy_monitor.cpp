#include "diagnostics/sensors/gps_accuracy_monitor.h"

GPSAccuracyMonitor::GPSAccuracyMonitor(const std::string& name, const std::string& topic) : is_error_(false), timer(nullptr) {
    component_name_ = name;
    topic_and_cb_.first = topic;
    topic_and_cb_.second = std::bind(&GPSAccuracyMonitor::collectData, this, std::placeholders::_1);
}

void GPSAccuracyMonitor::collectData(const std::string& protobuf_data) {
    SensorData gps_accuracy_data;
    if (gps_accuracy_data.ParseFromString(protobuf_data)) {
        if(sensor_data_ != gps_accuracy_data.data()) {
            sensor_data_ = gps_accuracy_data.data();
        }
    } 
    else {
        diagnostics_message_ = "ERROR: Failed to parse Protobuf data.";
    }
}

void GPSAccuracyMonitor::performDiagnostics() {
    //Example diagnostic check
    if (sensor_data_ <= 200) {
        if(timer != nullptr) {
            delete timer;
            timer = nullptr;
        }
	    is_error_ = false;
        diagnostics_message_ = "";
    } 
    else if (sensor_data_ > 200) {
        //run in a thread for 15s after that update teh is_error_ to true.
        timer = new Timer();
        timer->start([&](){is_error_ = true;}, 15 * 1000);
        //diagnostics_message_ = "ERROR: GPS accuracy fallen below 200mm!";
    }
}

std::string GPSAccuracyMonitor::getDiagnosticsMessage() const {
    return diagnostics_message_;
}

std::pair<std::string, std::function<void(const std::string&)>> GPSAccuracyMonitor::getSubscribeTopics() const {
    return  topic_and_cb_;
}

bool GPSAccuracyMonitor::isError() const {
    return is_error_;
}

GPSAccuracyMonitor::~GPSAccuracyMonitor() {
    if(timer != nullptr) {
        delete timer;
    }
}