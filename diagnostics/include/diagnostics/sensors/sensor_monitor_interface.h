#ifndef SENSOR_MONITOR_INTERFACE_H
#define SENSOR_MONITOR_INTERFACE_H

#include <string>
#include <vector>
#include <functional>
#include <utility>

/**
 * @brief Interface class to implement all sensor monitoring.
 */
class SensorMonitorInterface {
public:
    virtual ~SensorMonitorInterface() = default;

    /**
     * @brief interface to receive senor data 
     * @param Protobuf-encoded string msg
     */
    virtual void collectData(const std::string& protobuf_data) = 0;
    
    /**
     * @brief Interface for performing diagnostics based on collected data
     */
    virtual void performDiagnostics() = 0;

    /**
     * @brief Interface to update diagnostics information for a particular sensor
     * @return diangnostics in string fromat
     */
    virtual std::string getDiagnosticsMessage() const = 0;

    virtual std::pair<std::string, std::function<void(const std::string&)>> getSubscribeTopics() const = 0;

    virtual bool isError() const = 0;

protected:
    std::string component_name_;
};

#endif // SENSOR_MONITOR_INTERFACE_H

