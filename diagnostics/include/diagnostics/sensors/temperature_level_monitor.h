#ifndef TEMPERATURE_LEVEL_MONITOR_H
#define TEMPERATURE_LEVEL_MONITOR_H

#include "sensor_monitor_interface.h"
#include "../msg/sensor_data.pb.h"  // Include Protobuf-generated header

/**
 * @brief temperature monitor implementation.
 */
class TemperatureLevelMonitor : public SensorMonitorInterface {
public:

    //constructor
    TemperatureLevelMonitor(const std::string& name, const std::string& topic);

    /**
     * @brief temperature data callback implementation.
     * @param protobuf_data temperature level information in protobuf string
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

#endif // TEMPERATURE_LEVEL_MONITOR_H