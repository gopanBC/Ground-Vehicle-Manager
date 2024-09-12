#ifndef EMERGENCY_STOP_MONITOR_H
#define EMERGENCY_STOP_MONITOR_H

#include "sensor_monitor_interface.h"
#include "../msg/sensor_data.pb.h"  // Include Protobuf-generated header

/**
 * @brief Emergency stop monitor implmentation.
 * report error if emergency button pressed%.
 */
class EmergencyStopMonitor : public SensorMonitorInterface {
public:

    //constructor
    EmergencyStopMonitor(const std::string& name, const std::string& topic);

    /**
     * @brief Emergency stop callback monitor
     * @param protobuf_data emergency stop information in protobuf string
     */
    void collectData(const std::string& protobuf_data) override;

    void performDiagnostics() override;

    std::string getDiagnosticsMessage() const override;

    virtual std::pair<std::string, std::function<void(const std::string&)>> getSubscribeTopics() const override;

    bool isError() const override;

private:
    std::atomic<bool> is_error_;
    float sensor_data_;
    std::string diagnostics_message_;
    std::pair<std::string, std::function<void(const std::string&)>> topic_and_cb_;
    //Timer* timer;
};

#endif // EMERGENCY_STOP_MONITOR_H

