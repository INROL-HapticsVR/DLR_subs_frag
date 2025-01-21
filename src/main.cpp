#include "core/parser.hpp"
#include "core/subs.hpp"
#include <thread>
#include <memory>


std::array<std::array<uint32_t, BUFFER_NUM + 3>, 3> buffer_global_index = {};
std::mutex global_index_mutex;

int main() {
    // MQTT 브로커 정보 및 토픽 정의
    // std::string broker_address = "147.46.149.20";
    std::string broker_address = "192.168.0.12";
    int port = 1883;
    // std::vector<std::string> topics = {"topic/image0"};
    std::vector<std::string> topics = {"topic/image0", "topic/depth0", "topic/pose0"};


    // std::vector<std::string> topics = {"topic/image0", "topic/depth0"};
    std::vector<std::unique_ptr<Parser>> queueConsumers;
    std::vector<std::unique_ptr<MQTTSubscriber>> subscribers;

    std::cout << "start" << std::endl;

    auto parser = std::make_unique<Parser>(0, TopicType::RGB);
    queueConsumers.push_back(std::move(parser));          // 소유권 공유
    auto subscriber = std::make_unique<MQTTSubscriber>(broker_address, port, topics[0], *queueConsumers.back()); 
    subscribers.push_back(std::move(subscriber));

    parser = std::make_unique<Parser>(1, TopicType::DEPTH);
    queueConsumers.push_back(std::move(parser));          // 소유권 공유
    subscriber = std::make_unique<MQTTSubscriber>(broker_address, port, topics[1], *queueConsumers.back()); 
    subscribers.push_back(std::move(subscriber));

    parser = std::make_unique<Parser>(2, TopicType::POSE);
    queueConsumers.push_back(std::move(parser));          // 소유권 공유
    subscriber = std::make_unique<MQTTSubscriber>(broker_address, port, topics[2], *queueConsumers.back()); 
    subscribers.push_back(std::move(subscriber));

    // 쓰레드 배열
    std::vector<std::thread> subscriberThreads;
    std::vector<std::thread> consumerThreads;

    // 구독 쓰레드 생성
    for (size_t i = 0; i < subscribers.size(); ++i) {
        subscriberThreads.emplace_back([&subscribers, i]() {
            subscribers[i]->subscribe();
        });
    }
    
    // 소비 쓰레드 생성
    for (size_t i = 0; i < queueConsumers.size(); ++i) {
        consumerThreads.emplace_back([&queueConsumers, i]() {
            queueConsumers[i]->stack();
        });
    }
    // // 파싱 쓰레드 생성
    // std::thread parsingThread;

    // 모든 쓰레드 합류
    for (auto& thread : subscriberThreads) {
        thread.join();
    }
    for (auto& thread : consumerThreads) {
        thread.join();
    }

    return 0;
}