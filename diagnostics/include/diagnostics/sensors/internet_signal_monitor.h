#ifndef INTERNET_SIGNAL_MONITOR_H
#define INTERNET_SIGNAL_MONITOR_H

#include "sensor_monitor_interface.h"
#include "../msg/sensor_data.pb.h"  // Include Protobuf-generated header

/**
 * @brief Internet signal monitor implementation.
 */
class InternetSignalMonitor : public SensorMonitorInterface {
public:

    //constructor
    InternetSignalMonitor(const std::string& name, const std::string& topic);
    ~InternetSignalMonitor();

    /**
     * @brief internet singal data callback implementation.
     * @param protobuf_data internet signal information in protobuf string
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
    Timer* timer;
};

#endif // INTERNET_SIGNAL_MONITOR_H

