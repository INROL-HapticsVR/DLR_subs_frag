#include "core/subs.hpp"
#include <iostream>

MQTTSubscriber::MQTTSubscriber(const std::string& broker, int port, const std::string& topic, Parser& queueConsumer)
    : broker_(broker), port_(port), topic_(topic), mosq_(nullptr), queueConsumer(queueConsumer) {
    init();

    tik = std::chrono::steady_clock::now();
}

MQTTSubscriber::~MQTTSubscriber() {
    if (mosq_) {
        mosquitto_destroy(mosq_); // Mosquitto 클라이언트 제거
        mosq_ = nullptr;
    }
    mosquitto_lib_cleanup(); // Mosquitto 라이브러리 정리
}

void MQTTSubscriber::subscribe() {
    mosquitto_message_callback_set(mosq_, MQTTSubscriber::onMessageWrapper);

    if (mosquitto_subscribe(mosq_, nullptr, topic_.c_str(), 1) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to subscribe to topic: " << topic_ << std::endl;
        return;
    }

    std::cout << "Subscribed to topic: " << topic_ << std::endl;

    // Non-blocking loop
    int ret = mosquitto_loop_start(mosq_);
    if (ret != MOSQ_ERR_SUCCESS) {
         std::cerr << "Mosquitto loop start failed with error: " << ret << std::endl;
     }
}

void MQTTSubscriber::init() {
    mosquitto_lib_init(); // Mosquitto 라이브러리 초기화
    mosq_ = mosquitto_new(nullptr, true, this);
    if (!mosq_) {
        std::cerr << "Failed to create Mosquitto client." << std::endl;
        mosquitto_lib_cleanup();
        return;
    }

    if (mosquitto_connect(mosq_, broker_.c_str(), port_, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to connect to broker." << std::endl;
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
        return;
    }

    // dg0
    tik = std::chrono::steady_clock::now();
}

void MQTTSubscriber::onMessageWrapper(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* message) {
    if (!userdata) return;
    auto* self = static_cast<MQTTSubscriber*>(userdata);

    self->onMessage(message);
}

void MQTTSubscriber::onMessage(const struct mosquitto_message* message) {
    if (message->payloadlen > 0) {
        queueConsumer.push(std::string(static_cast<char*>(message->payload), message->payloadlen)); 
    }
    
    // 현재 시간 기록 (디버깅용)
    tak = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(tak - tik);
    std::cout << "[Debug] Time: " << topic_ << ": " << duration.count() << " ms" << std::endl;
    tik = tak;
}