#ifndef BATTERY_LEVEL_MONITOR_H
#define BATTERY_LEVEL_MONITOR_H

#include "sensor_monitor_interface.h"
#include "../msg/sensor_data.pb.h"  // Include Protobuf-generated header

/**
 * @brief Batter level monitor implementation.
 * report error if battery level falls below 50%.
 */
class BatteryLevelMonitor : public SensorMonitorInterface {
public:

    //constructor
    BatteryLevelMonitor(const std::string& name, const std::string& topic);

    /**
     * @brief bms data callback implementation.
     * @param batter level information in protobuf string
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

#endif // BATTERY_LEVEL_MONITOR_H

