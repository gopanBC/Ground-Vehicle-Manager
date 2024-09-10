#ifndef MIDDLEWARE_INTERFACE_H
#define MIDDLEWARE_INTERFACE_H

#include <string>
#include <vector>
#include <functional>
#include <utility>

class MiddlewareInterface {
public:
    virtual ~MiddlewareInterface() = default;

    virtual void initialize(int argc, char** argv) = 0;
    virtual void spin() = 0;
    virtual void publish(const std::string& message) = 0;
    virtual void subscribeAll() = 0;
    inline void addSubscribers(std::pair<const std::string, std::function<void(const std::string&)>> topic_and_callback)  {
        subscription_list.push_back(topic_and_callback);
    }

protected:
    std::vector<std::pair<std::string, std::function<void(const std::string&)>>> subscription_list;
};

#endif // MIDDLEWARE_INTERFACE_H

