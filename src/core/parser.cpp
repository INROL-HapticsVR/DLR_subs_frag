#include "core/parser.hpp"

#include <stdexcept>
#include <cstring>
#include <opencv2/opencv.hpp> // OpenCV의 모든 기본 모듈 포함
#include <opencv2/highgui.hpp> // GUI 창을 위한 모듈
#include <opencv2/imgproc.hpp> // 이미지 처리 모듈
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sstream>

#include <chrono>

Parser::Parser(int topic, TopicType topic_type_) : shm_fd(-1), shm_ptr(nullptr), shm_size(4 * WIDTH * HEIGHT * sizeof(uint8_t)) {
    topic_idx = std::to_string(topic);
    shm_name = "img_rendered_" + topic_idx;
    topic_type = topic_type_;

    // Shared Memory 초기화
    shm_fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        throw std::runtime_error("Failed to create shared memory.");
    }

    if (ftruncate(shm_fd, shm_size) == -1) {
        throw std::runtime_error("Failed to set shared memory size.");
    }

    shm_ptr = static_cast<uint8_t*>(mmap(0, shm_size, PROT_WRITE, MAP_SHARED, shm_fd, 0));
    if (shm_ptr == MAP_FAILED) {
        throw std::runtime_error("Failed to map shared memory.");
    }
}

Parser::~Parser() {
    if (shm_ptr) {
        munmap(shm_ptr, shm_size);
        shm_ptr = nullptr;
    }
    if (shm_fd != -1) {
        close(shm_fd);
        shm_fd = -1;
    }
    shm_unlink(shm_name.c_str());
}

void Parser::push(const std::string& payload) {
    std::lock_guard<std::mutex> lock(dataMutex);
    // 가장 최근 데이터 저장
    latestPayload = payload;
    dataCondition.notify_one(); // 소비 스레드에 알림
}

std::string Parser::pop() {
    std::unique_lock<std::mutex> lock(dataMutex);
    dataCondition.wait(lock, [&]() { return !latestPayload.empty(); }); // 데이터가 비어있으면 대기
    return latestPayload; // 가장 최근 데이터 반환
}

void Parser::consume() {
    while (true) {
        auto message = pop(); // 가장 최근 메시지 가져오기

        auto tik = std::chrono::steady_clock::now();

        parsing(message);

        auto tak = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(tak - tik);
        // std::cout << "[Debug] Parser time: " << topic_idx << ": " << duration.count() << " ms" << std::endl;
    }
}

void Parser::parsing(std::string payload) {
#ifdef USE_RENDERING
    //RGB-D 카메라
    if (topic_type == TopicType::RGB) {
        // 데이터를 바이너리에서 구조체로 변환
        std::memcpy(&global_index, &payload[0], sizeof(uint32_t));
        std::memcpy(&fragment_index, &payload[1], sizeof(uint32_t));

        // 하림씨가 RGBRGB로 준다도르
        int part_num = WIDTH * HEIGHT / FRAGMENT_NUM;
        for (int i = 0; i < part_num; i++) {
            rgb.r[i + part_num * fragment_index] = static_cast<uint8_t>(payload[16 + 3 * i + 0]);
            rgb.g[i + part_num * fragment_index] = static_cast<uint8_t>(payload[16 + 3 * i + 1]);
            rgb.b[i + part_num * fragment_index] = static_cast<uint8_t>(payload[16 + 3 * i + 2]);
        }

        fragmentChecker();

        if (fragment_index == FRAGMENT_NUM){
            if (fragmentChecker()){
                displayRGBImage();
            }
            else {
                std::cout << "fragments are not fullfilled" << std::endl;
            }
        }
    } 
        //DEPTH 카메라
    else if (topic_type == TopicType::DEPTH) {
        std::memcpy(&global_index, &payload[0], sizeof(uint32_t));
        std::memcpy(&fragment_index, &payload[1], sizeof(uint32_t));

        // 하림씨가 DEPTH를 Float로 준다도르
        int part_num = WIDTH * HEIGHT / FRAGMENT_NUM;
        for (int i = 0; i < part_num; i++) {
            std::memcpy(&depth.d[i + part_num], &payload[16 + 4 * i], sizeof(float));
        }

        fragmentChecker();

        if (fragment_index == FRAGMENT_NUM){
            if (fragmentChecker()){
                displayDepthImage();
            }
            else {
                std::cout << "fragments are not fullfilled" << std::endl;
            }
        }
    }

    else if (topic_type == TopicType::POSE){
        std::memcpy(&global_index, &payload[0], sizeof(uint32_t));
        for (int i = 0; i < 3; i++) {
            std::memcpy(&pose.pos[i], &payload[8 + i], sizeof(float));
        }
        for (int i = 0; i < 4; i++) {
            std::memcpy(&pose.rot[i], &payload[8 + 3 + i], sizeof(float));
        }
    }
    
#else
    if (!shm_ptr) {
        throw std::runtime_error("Shared memory pointer is null.");
    }

    std::memcpy(&time, &payload[0], sizeof(int64_t));

    if (topic_type == TopicType::RGB) {
        for (int i = 0; i < WIDTH * HEIGHT; ++i) {
            shm_ptr[3 * i + 0] = payload[8 + 3 * i + 0]; // R
            shm_ptr[3 * i + 1] = payload[8 + 3 * i + 1]; // G
            shm_ptr[3 * i + 2] = payload[8 + 3 * i + 2]; // B
        }
        if (msync(shm_ptr, shm_size, MS_SYNC) == -1) {
            throw std::runtime_error("Failed to sync shared memory.");
        }
    }
    else if (topic_type == TopicType::DEPTH){
        for (int i = 0; i < 4 * WIDTH * HEIGHT; i++) {
            shm_ptr[i] = payload[8 + i]; //d
        }
        if (msync(shm_ptr, shm_size, MS_SYNC) == -1) {
            throw std::runtime_error("Failed to sync shared memory.");
        }
    }
    else if (topic_type == TopicType::POSE){
        for (int i = 0; i < 3; i++) {
            std::memcpy(&pose.pos[i], &payload[8 + i], sizeof(float));
        }
        for (int i = 0; i < 4; i++) {
            std::memcpy(&pose.rot[i], &payload[8 + 3 + i], sizeof(float));
        }
    }

#endif
}

bool Parser::fragmentChecker() {
    if (fragment_index == 0){
        for (int i=0; i<FRAGMENT_NUM; i++){
            fragment_checker[i] = false;
        }
        fragment_checker[0] = true;
    }
    else {
        fragment_checker[fragment_index] = true;
    }
    bool true_checker = true;
    for (int i=0; i<FRAGMENT_NUM; i++){
        true_checker = fragment_checker[i] && true_checker;
    }
    return true_checker;
}

void Parser::displayRGBImage() {
    cv::Mat image(HEIGHT, WIDTH, CV_8UC3);

    for (int i = 0; i < HEIGHT; ++i) {
        for (int j = 0; j < WIDTH; ++j) {
            int idx = i * WIDTH + j;
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(rgb.r[idx], rgb.g[idx], rgb.b[idx]);
        }
    }

    cv::imshow("RGB Image " + topic_idx, image);
    cv::waitKey(1); // 1ms 대기 (실시간 처리)
}

void Parser::displayDepthImage() {
    // float 데이터를 표시하기 위해 CV_32FC1 타입의 Mat 생성
    cv::Mat depthImage(HEIGHT, WIDTH, CV_32FC1);

    // float 데이터를 Mat으로 복사
    for (int i = 0; i < HEIGHT; ++i) {
        for (int j = 0; j < WIDTH; ++j) {
            int idx = i * WIDTH + j;
            depthImage.at<float>(i, j) = depth.d[idx];
        }
    }

    // 표시할 이미지를 0~255 범위로 정규화 (시각화를 위해)
    cv::Mat depthImageNormalized;
    cv::normalize(depthImage, depthImageNormalized, 0, 255, cv::NORM_MINMAX);
    depthImageNormalized.convertTo(depthImageNormalized, CV_8UC1);

    // Depth 이미지 표시
    cv::imshow("Depth Image", depthImageNormalized);
    cv::waitKey(1); // 1ms 대기 (실시간 처리)
}