#ifndef ZEROMQ_MIDDLEWARE_H
#define ZEROMQ_MIDDLEWARE_H

#include "middleware_interface.h"

#include <zmq.hpp>

#include <string>
#include <functional>
#include <vector>
#include <utility>

class ZeroMQMiddleware : public MiddlewareInterface {
public:
    ZeroMQMiddleware();
    ~ZeroMQMiddleware() override;

    void initialize(int argc, char** argv) override;
    void spin() override;
    void publish(const std::string& message) override;
    void subscribeAll() override;

private:
    zmq::context_t context;
    zmq::socket_t publisher_socket;
    zmq::socket_t subscriber_socket;
};

#endif // ZEROMQ_MIDDLEWARE_H
