#include "Test/Test1.hpp"
#include "core/subs.hpp"

#include <iostream>

namespace Test {

void Test1() {
    std::cout << "=============Test1=============" << std::endl;

    // // MQTTSubscriber 객체 생성
    // const std::string broker = "147.46.149.20";
    // const int port = 1883;
    // const std::string topic = "ddd";

    // MQTTSubscriber subscriber(broker, port, topic);

    // // MQTT 구독 시작
    // subscriber.subscribe();

    std::cout << "=============End=============" << std::endl;
}

}; // namespace Test