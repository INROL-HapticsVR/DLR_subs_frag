#include "core/parser.hpp"

#include <mosquitto.h>
#include <string>
#include <vector>

#include <chrono>

class MQTTSubscriber {
public:
    MQTTSubscriber(const std::string& broker, int port, const std::string& topic, Parser& queueConsumer);
    ~MQTTSubscriber();

    void subscribe();

private:
    std::string broker_;
    int port_;
    std::string topic_;
    struct mosquitto* mosq_;
    Parser& queueConsumer;

    // debug0
    std::chrono::_V2::steady_clock::time_point tik;
    std::chrono::_V2::steady_clock::time_point tak;
    // debug-1

    void init();
    static void onMessageWrapper(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* message);
    void onMessage(const struct mosquitto_message* message);
};