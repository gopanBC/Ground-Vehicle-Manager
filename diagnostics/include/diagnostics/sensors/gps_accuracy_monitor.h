#ifndef GPS_ACCURACY_MONITOR_H
#define GPS_ACCURACY_MONITOR_H

#include "sensor_monitor_interface.h"
#include "../msg/sensor_data.pb.h" 
#include "../utility/timer.h"

/**
 * @brief GPS Accuracy monitor implementation
 */
class GPSAccuracyMonitor : public SensorMonitorInterface {
public:

    //constructor
    GPSAccuracyMonitor(const std::string& name, const std::string& topic);

    ~GPSAccuracyMonitor();

    /**
     * @brief gps accuracy data callback implementation.
     * @param protobuf_data gps accuracy information in protobuf string
     */
    void collectData(const std::string& protobuf_data) override;

    void performDiagnostics() override;

    std::string getDiagnosticsMessage() const override;

    virtual std::pair<std::string, std::function<void(const std::string&)>> getSubscribeTopics() const override;

    bool isError() const override;

    Timer* timer;

private:
    std::atomic<bool> is_error_;
    float sensor_data_;
    std::string diagnostics_message_;
    std::pair<std::string, std::function<void(const std::string&)>> topic_and_cb_;
};

#endif // GPS_ACCURACY_MONITOR_H

