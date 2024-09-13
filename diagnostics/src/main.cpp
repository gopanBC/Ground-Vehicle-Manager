#include <iostream>
#include "diagnostics/sensors/battery_level_monitor.h"
#include "diagnostics/sensors/emergency_stop_monitor.h"
#include "diagnostics/sensors/gps_accuracy_monitor.h"
#include "diagnostics/sensors/internet_signal_monitor.h"
#include "diagnostics/sensors/temperature_level_monitor.h"

#include "diagnostics/middleware/ros2_middleware.h"
#include "diagnostics/middleware/ros_middleware.h"
#include "diagnostics/middleware/zeromq.h"

#include "diagnostics/rti/real_time_scheduler.h"

#include "../msg/diagnostics.pb.h"

// Function to initialize middleware based on selection
// TODO : Implement a std::map<"string", std::unique_ptr<MiddlewareInterface>> to handle more middleware
std::unique_ptr<MiddlewareInterface> createMiddleware(const std::string& type, const std::string& topic) {
    if (type == "ROS") {
        return std::make_unique<ROSMiddleware>(topic);
    } 
    else {
        return std::make_unique<ZeroMQMiddleware>();  // Replace with actual implementation
    }
}

int main(int argc, char** argv) {
    // Middleware type and topic
    std::string middleware_type = "ROS";
    std::string realtime_middleware_type = "ZEROMQ";  //DDS will be better 
    std::string topic = "/diagnostics";

    // Create and initialize middleware
    auto middleware = createMiddleware(middleware_type, topic);
    auto rt_middleware = createMiddleware(realtime_middleware_type, topic);
    middleware->initialize(argc, argv); //In this case its ROS1 node initialisation.
    rt_middleware->initialize(argc, argv);

    // Create sensor monitor
    BatteryLevelMonitor bms("BMS", {"bms_data"});
    EmergencyStopMonitor estop("Estop", {"Estop_data"});
    GPSAccuracyMonitor gps("GPS", {"gps_accuracy_data"});
    InternetSignalMonitor internet("Internet", {"internet_signal_data"});
    TemperatureLevelMonitor temp("Temp", {"temp_level_data"});
    std::vector<SensorMonitorInterface*> sensors = {&bms, &estop, &gps, &internet, &temp};
    //std::vector<SensorMonitorInterface*> sensors = {&bms};

    //updating the subscriber topics-callback pair to the middleware.
    for(auto sensor : sensors) {
       auto topic_and_cb = sensor->getSubscribeTopics();
       middleware->addSubscribers(topic_and_cb);
       middleware->subscribeAll();
    }

    //publishing system status through a scheduler.
    //checks all sensor status and publishes combined system status.
    RealTimeScheduler scheduler([&]() {
        bool overall_error = false;
        DiagnosticsMessage combined_diag_msg;
        combined_diag_msg.set_component_name("Sensor Status");

        for (auto sensor : sensors) {
            sensor->performDiagnostics();
            if (sensor->isError()) {
                overall_error = true;
            }
        }

        // Set overall status based on all sensors
        if (overall_error) {
            combined_diag_msg.set_status(DiagnosticsMessage::ERROR);
            combined_diag_msg.set_error_details("One or more sensors are in error.");
        } else {
            combined_diag_msg.set_status(DiagnosticsMessage::OK);
            combined_diag_msg.set_error_details("All sensors are functioning normally.");
        }

        // Serialize the combined message and publish
        std::string serialized_msg;
        combined_diag_msg.SerializeToString(&serialized_msg);
        middleware->publish(serialized_msg);
        rt_middleware->publish(serialized_msg);
    }, std::chrono::nanoseconds(10000000), 90); //10ms

    // Spin to process middleware events
    middleware->spin();
    return 0;
}

