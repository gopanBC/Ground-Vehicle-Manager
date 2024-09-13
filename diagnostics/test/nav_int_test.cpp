#include <gtest/gtest.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include "../msg/diagnostics.pb.h"  // Protobuf message header


// Global flag for test result
bool message_received = false;
DiagnosticsMessage received_combined_message;

// Callback for the combined diagnostics message
void diagnosticsCallback(const std_msgs::String::ConstPtr& msg) {
    DiagnosticsMessage diag_msg;
    if (diag_msg.ParseFromString(msg->data)) {
        received_combined_message = diag_msg;
        message_received = true;
    } else {
        ROS_ERROR("Failed to parse DiagnosticsMessage.");
    }
}

// Integration test for the diagnostics system
TEST(NavIntegrationTest, CombinedDiagnosticsMessage) {
    //ros::NodeHandle nh;
    // Subscribe to the combined diagnostics topic
    //ros::Subscriber diag_sub = nh.subscribe("/system/diagnostics", 10, diagnosticsCallback);

    // Create publishers to simulate sensor diagnostics messages
    //ros::Publisher sensor1_pub = nh.advertise<std_msgs::String>("/sensor_1/diagnostics", 10);
    //ros::Publisher sensor2_pub = nh.advertise<std_msgs::String>("/sensor_2/diagnostics", 10);

    // Wait for publishers and subscribers to connect
    //ros::Duration(1.0).sleep();

    // Create diagnostics messages for each sensor
    /*DiagnosticsMessage sensor1_msg;
    sensor1_msg.set_component_name("Sensor_1");
    sensor1_msg.set_status(DiagnosticsMessage::OK);
    sensor1_msg.set_error_details("Sensor 1 functioning normally");

    DiagnosticsMessage sensor2_msg;
    sensor2_msg.set_component_name("Sensor_2");
    sensor2_msg.set_status(DiagnosticsMessage::ERROR);
    sensor2_msg.set_error_details("Sensor 2 has failed");

    // Serialize the messages
    std::string sensor1_serialized;
    std::string sensor2_serialized;
    ASSERT_TRUE(sensor1_msg.SerializeToString(&sensor1_serialized));
    ASSERT_TRUE(sensor2_msg.SerializeToString(&sensor2_serialized));

    // Publish the sensor diagnostics messages
    std_msgs::String sensor1_ros_msg;
    sensor1_ros_msg.data = sensor1_serialized;
    sensor1_pub.publish(sensor1_ros_msg);

    std_msgs::String sensor2_ros_msg;
    sensor2_ros_msg.data = sensor2_serialized;
    sensor2_pub.publish(sensor2_ros_msg);

    // Wait for the diagnostics node to process and publish the combined message
    ros::Duration(2.0).sleep();

    // Spin to process the callback
    ros::spinOnce();

    // Check if the combined diagnostics message was received
    ASSERT_TRUE(message_received);

    // Verify the contents of the combined diagnostics message
    EXPECT_EQ(received_combined_message.component_name(), "Overall_System");
    EXPECT_EQ(received_combined_message.status(), DiagnosticsMessage::ERROR);  // Since Sensor 2 is in error
    EXPECT_EQ(received_combined_message.error_details(), "One or more sensors are in error.");*/
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "diagnostics_integration_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
