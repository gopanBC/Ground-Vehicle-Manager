#include "diagnostics/middleware/zeromq.h"

#include <iostream>
#include <thread>
#include <chrono>

// Constructor initializes ZeroMQ context and sockets
ZeroMQMiddleware::ZeroMQMiddleware()
    : context(1), publisher_socket(context, ZMQ_PUB), subscriber_socket(context, ZMQ_SUB)
{}

// Destructor closes the sockets and context
ZeroMQMiddleware::~ZeroMQMiddleware() {
    publisher_socket.close();
    subscriber_socket.close();
    context.close();
}

// Initialize ZeroMQ sockets, bind publisher, and connect subscriber
void ZeroMQMiddleware::initialize(int argc, char** argv) {
    // Binding publisher to a TCP port
    publisher_socket.bind("tcp://*:5556");

    // Connect the subscriber socket to the publisher (assuming localhost)
    subscriber_socket.connect("tcp://localhost:5556");

    // Subscribe to all topics
    subscribeAll();
}

// Spin: Checks for incoming messages and calls the associated callback
void ZeroMQMiddleware::spin() {
    zmq::message_t zmq_message;
    while (true) {
        for (const auto& topic_and_callback : subscription_list) {
            // Non-blocking check for messages on each topic
            subscriber_socket.recv(zmq_message, zmq::recv_flags::dontwait);
            if (zmq_message.size() > 0) {
                std::string message(static_cast<char*>(zmq_message.data()), zmq_message.size());
                topic_and_callback.second(message);  // Call the callback function
            }
        }
        // Sleep to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Publish a message via the publisher socket
void ZeroMQMiddleware::publish(const std::string& message) {
    zmq::message_t zmq_message(message.data(), message.size());
    publisher_socket.send(zmq_message, zmq::send_flags::none);
    //std::cout << "Published message: " << message << std::endl;
}

// Subscribe to all topics in the subscription_list
void ZeroMQMiddleware::subscribeAll() {
    for (const auto& topic_and_callback : subscription_list) {
        // Subscribe to the topic
        subscriber_socket.setsockopt(ZMQ_SUBSCRIBE, topic_and_callback.first.c_str(), topic_and_callback.first.size());
        std::cout << "Subscribed to topic: " << topic_and_callback.first << std::endl;
    }
}
