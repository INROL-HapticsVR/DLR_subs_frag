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

#include <thread>
#include <chrono>

Parser::Parser(int topic, TopicType topic_type_) : shm_fd(-1), shm_ptr(nullptr), shm_size((1+4+4 * WIDTH * HEIGHT) * sizeof(uint8_t)) {
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
    buffer_ptr = new uint8_t*[BUFFER_NUM]; // buffer_ptr[i][j] = some_value;  // i번째 버퍼의 j번째 데이터
    for (size_t i = 0; i < BUFFER_NUM; ++i) {
        buffer_ptr[i] = new uint8_t[shm_size];
    }

    buffer_last_idx = 0;
    buffer_write_idx = 0;
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
    for (size_t i = 0; i < BUFFER_NUM; ++i) {
        delete[] buffer_ptr[i];
    }
    delete[] buffer_ptr;
}

void Parser::push(const std::string& payload) {
    std::lock_guard<std::mutex> lock(dataMutex);
    // 가장 최근 데이터 저장
    latestPayload = payload;
    dataCondition.notify_one(); // 소비 스레드에 알림
}

// std::string Parser::pop() {
//     std::unique_lock<std::mutex> lock(dataMutex);
//     dataCondition.wait(lock, [&]() { return !latestPayload.empty(); }); // 데이터가 비어있으면 대기
//     return latestPayload; // 가장 최근 데이터 반환
// }

// void Parser::consume() {
//     while (true) {
//         auto message = pop(); // 가장 최근 메시지 가져오기
//         parsing(message);
//     }
// }

std::string Parser::pop() {
    std::unique_lock<std::mutex> lock(dataMutex);
    // 데이터가 비어있으면 대기
    dataCondition.wait(lock, [&]() { return !latestPayload.empty(); }); 
    std::string payload = std::move(latestPayload); // 데이터 복사 후 비움
    latestPayload.clear(); // 데이터를 처리한 후 삭제
    return payload; 
}

void Parser::consume() {
    while (true) {
        auto message = pop(); // 가장 최근 메시지 가져오기
        parsing(message); // 메시지 처리
    }
}

void Parser::parsing(std::string payload) {
#ifdef USE_RENDERING
    //RGB 카메라
    if (topic_type == TopicType::RGB) {
        // 데이터를 바이너리에서 구조체로 변환
        std::memcpy(&global_index, &payload[0], sizeof(uint32_t));
        std::memcpy(&fragment_index, &payload[4], sizeof(uint32_t));

        // 하림씨가 RGBRGB로 준다도르
        int part_num = WIDTH * HEIGHT / FRAGMENT_NUM;
        for (int i = 0; i < part_num; i++) {
            rgb.r[i + part_num * fragment_index] = static_cast<uint8_t>(payload[8 + 3 * i + 0]);
            rgb.g[i + part_num * fragment_index] = static_cast<uint8_t>(payload[8 + 3 * i + 1]);
            rgb.b[i + part_num * fragment_index] = static_cast<uint8_t>(payload[8 + 3 * i + 2]);
        }

        fragmentChecker();

        if (fragment_index == FRAGMENT_NUM - 1){
            if (fragmentChecker() && !write_locker){
                displayRGBImage();
            }
            else {
                // std::cout << "fragments are not fullfilled" << std::endl;
            }
        }
    } 

    //DEPTH 카메라
    else if (topic_type == TopicType::DEPTH) {
        std::memcpy(&global_index, &payload[0], sizeof(uint32_t));
        std::memcpy(&fragment_index, &payload[4], sizeof(uint32_t));

        // 하림씨가 DEPTH를 Float로 준다도르
        int part_num = WIDTH * HEIGHT / FRAGMENT_NUM;
        for (int i = 0; i < part_num; i++) {
            std::memcpy(&depth.d[i + part_num* fragment_index], &payload[8 + 4 * i], sizeof(float));
        }

        fragmentChecker();

        if (fragment_index == FRAGMENT_NUM - 1){
            if (fragmentChecker() && !write_locker){
                displayDepthImage();
            }
            else {
                // std::cout << "fragments are not fullfilled" << std::endl;
            }
        }
    }

    // POSE
    else if (topic_type == TopicType::POSE){
        std::memcpy(&global_index, &payload[0], sizeof(uint32_t));
        for (int i = 0; i < 3; i++) {
            std::memcpy(&pose.pos[i], &payload[4 + 4 * i], sizeof(float));
        }
        for (int i = 0; i < 4; i++) {
            std::memcpy(&pose.rot[i], &payload[4 + 12 + 4 * i], sizeof(float));
        }
    }
    
#else
    //RGB 카메라
    if (topic_type == TopicType::RGB) {
        std::memcpy(&global_index, &payload[0], sizeof(uint32_t));
        std::memcpy(&fragment_index, &payload[4], sizeof(uint32_t));

        // 하림씨가 RGBRGB로 준다도르
        int part_num = WIDTH * HEIGHT / FRAGMENT_NUM;
        for (int i = 0; i < 3 * part_num; i++) {
            payload_tmp[i + 3 * part_num * fragment_index] = payload[8 + i];
        }

        fragmentChecker();
        if (fragment_index == FRAGMENT_NUM - 1){
            if (fragmentChecker() && !write_locker){
                // Global index
                for (int i = 0; i< 4; i++){
                    buffer_ptr[buffer_last_idx][i] = payload[i];
                }
                // Data
                for (int i = 0; i < 3 * WIDTH * HEIGHT; i++){
                    buffer_ptr[buffer_last_idx][4+i] = payload_tmp[i];
                }
                write_locker = true;
                indexChecker();
                if (buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][BUFFER_NUM+2] == buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][BUFFER_NUM+1]){
                    buffer_last_idx++;
                    bufferManager();
                }
                else {
                    shmWrite();
                }
            }
        }
    }

    //DEPTH 카메라
    else if (topic_type == TopicType::DEPTH){
        std::memcpy(&global_index, &payload[0], sizeof(uint32_t));
        std::memcpy(&fragment_index, &payload[4], sizeof(uint32_t));

        // 하림씨가 DEPTH를 Float로 준다도르
        int part_num = WIDTH * HEIGHT / FRAGMENT_NUM;
        for (int i = 0; i < 4 * part_num; i++) {
            payload_tmp[i + 4 * part_num * fragment_index] = payload[8 + i];
        }

        fragmentChecker();
        if (fragment_index == FRAGMENT_NUM - 1){
            if (fragmentChecker() && !write_locker){
                // Global index
                for (int i = 0; i< 4; i++){
                    buffer_ptr[buffer_last_idx][i] = payload[i];
                }
                // Data
                for (int i = 0; i < 4 * WIDTH * HEIGHT; i++){
                    buffer_ptr[buffer_last_idx][4+i] = payload_tmp[i];
                }
                write_locker = true;
                indexChecker();
                if (buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][BUFFER_NUM+2] == buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][BUFFER_NUM+1]){
                    buffer_last_idx++;
                    bufferManager();
                }
                else {
                    shmWrite();
                }
            }
        }
    }

    // POSE
    else if (topic_type == TopicType::POSE){
        // std::cout << "ddddd" << std::endl;
        std::memcpy(&global_index, &payload[0], sizeof(uint32_t));
        // Global index
        for (int i = 0; i< 4; i++){
            buffer_ptr[buffer_last_idx][i] = payload[i];
        }
        // Data
        for (int i = 0; i < (3+4)*4; i++){
            buffer_ptr[buffer_last_idx][4+i] = payload[4+i];
        }
        indexChecker();
        if (buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][BUFFER_NUM+2] == buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][BUFFER_NUM+1]){
            buffer_last_idx++;
            bufferManager();
        }
        else {
            shmWrite();
        }
    }

#endif
}

bool Parser::fragmentChecker() {
    if (fragment_index == 0){
        for (int i=0; i<FRAGMENT_NUM; i++){
            write_locker = false;
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

bool Parser::globalChecker() {
    std::lock_guard<std::mutex> lock(global_check_mutex);
    global_check[static_cast<uint32_t>(topic_type)]= global_index;

    // std::cout << "global: " << global_check[0] << " " << global_check[1] << " " << global_check[2] << std::endl;

    if ((global_check[static_cast<uint32_t>(TopicType::RGB)] == global_check[static_cast<uint32_t>(TopicType::DEPTH)])
        &&(global_check[static_cast<uint32_t>(TopicType::DEPTH)] == global_check[static_cast<uint32_t>(TopicType::POSE)])) {
        return true;
    }
    else {
        return false;
    }
}

int Parser::indexChecker() {
    std::lock_guard<std::mutex> lock(global_index_mutex);
    buffer_global_index[static_cast<uint32_t>(topic_type)][buffer_last_idx] = global_index;

    buffer_write_idx = -1;
    if (topic_type == TopicType::RGB){
        for (int i = buffer_last_idx ; i >= 0; i--) {
            for (int j = BUFFER_NUM; j >= 0; j--) {
                for (int k = BUFFER_NUM; k >= 0; k--) {
                    if (buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][i] == buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][j]
                    && buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][i] == buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][k]
                    && buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][i] != 0){
                        buffer_write_idx = i;
                        buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][BUFFER_NUM] = i;
                        buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][BUFFER_NUM] = j;
                        buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][BUFFER_NUM] = k;
                        buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][BUFFER_NUM+1] = buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][i];
                        buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][BUFFER_NUM+1] = buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][j];
                        buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][BUFFER_NUM+1] = buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][k];
                    }
                }
            }
        }
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < BUFFER_NUM+3 ; j++){
                std::cout << " " << buffer_global_index[i][j];
            }
            std::cout << "|";
        }
        std::cout << std::endl;
    }
    else if (topic_type == TopicType::DEPTH){
        for (int i = buffer_last_idx ; i >= 0; i--) {
            for (int j = BUFFER_NUM; j >= 0; j--) {
                for (int k = BUFFER_NUM; k >= 0; k--) {
                    if (buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][i] == buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][j]
                    && buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][i] == buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][k]
                    && buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][i] != 0){
                        buffer_write_idx = i;
                        buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][BUFFER_NUM] = j;
                        buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][BUFFER_NUM] = i;
                        buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][BUFFER_NUM] = k;
                        buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][BUFFER_NUM+1] = buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][j];
                        buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][BUFFER_NUM+1] = buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][i];
                        buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][BUFFER_NUM+1] = buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][k];
                    }
                }
            }
        }
    }
    else if (topic_type == TopicType::POSE){
        for (int i = buffer_last_idx ; i >= 0; i--) {
            for (int j = BUFFER_NUM; j >= 0; j--) {
                for (int k = BUFFER_NUM; k >= 0; k--) {
                    if (buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][i] == buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][j]
                    && buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][i] == buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][k]
                    && buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][i] != 0){
                        buffer_write_idx = i;
                        buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][BUFFER_NUM] = j;
                        buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][BUFFER_NUM] = k;
                        buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][BUFFER_NUM] = i;
                        buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][BUFFER_NUM+1] = buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][j];
                        buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][BUFFER_NUM+1] = buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][k];
                        buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][BUFFER_NUM+1] = buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][i];
                    }
                }
            }
        }
    }
    return buffer_write_idx;
}


void Parser::bufferManager() {
    if (buffer_last_idx > BUFFER_NUM-1){
        buffer_last_idx = 0;
    }
}


void Parser::shmWrite(){
    shm_ptr[0] = 0;
    // Global index
    for (int i = 0; i< 4; i++){
        shm_ptr[1+i] = buffer_ptr[buffer_write_idx][i];
    }
    // Data
    if (topic_type == TopicType::RGB) {
        std::lock_guard<std::mutex> lock(global_index_mutex);
        std::cout << "iii" << std::endl;
        // std::cout << "image" << buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][BUFFER_NUM+1] << std::endl;
        for (int i = 0; i < 3 * WIDTH * HEIGHT; i++){
            shm_ptr[1+4+i] = buffer_ptr[buffer_write_idx][i];
        }
        // 버퍼 앞으로 밀기
        for (int i = buffer_write_idx + 1; i <= buffer_last_idx; i++){
            for (int j = 0; j < 4 + 3 * WIDTH * HEIGHT; j++){
                buffer_ptr[i-(buffer_write_idx+1)][j] = buffer_ptr[i][j];
            }
            // std::lock_guard<std::mutex> lock(global_index_mutex);
            buffer_global_index[i-(buffer_write_idx+1)] = buffer_global_index[i];
        }
        buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][BUFFER_NUM+2] = buffer_global_index[static_cast<uint32_t>(TopicType::RGB)][BUFFER_NUM+1];
    }
    else if (topic_type == TopicType::DEPTH) {
        std::lock_guard<std::mutex> lock(global_index_mutex);
        std::cout << "ddd" << std::endl;
        // std::cout << "depth" << buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][BUFFER_NUM+1] << std::endl;
        for (int i = 0; i < 4 * WIDTH * HEIGHT; i++){
            shm_ptr[1+4+i] = buffer_ptr[buffer_write_idx][i];
        }
        // 버퍼 앞으로 밀기
        for (int i = buffer_write_idx + 1; i <= buffer_last_idx; i++){
            for (int j = 0; j < 4 + 4 * WIDTH * HEIGHT; j++){
                buffer_ptr[i-(buffer_write_idx+1)][j] = buffer_ptr[i][j];
            }
            // std::lock_guard<std::mutex> lock(global_index_mutex);
            buffer_global_index[i-(buffer_write_idx+1)] = buffer_global_index[i];
        }
        buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][BUFFER_NUM+2] = buffer_global_index[static_cast<uint32_t>(TopicType::DEPTH)][BUFFER_NUM+1];
    }
    else if (topic_type == TopicType::POSE) {
        std::lock_guard<std::mutex> lock(global_index_mutex);
        std::cout << "ppp" << std::endl;
        // std::cout << "pose" << buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][BUFFER_NUM+1] << std::endl;
        for (int i = 0; i < (3+4)*4; i++){
            shm_ptr[1+4+i] = buffer_ptr[buffer_write_idx][i];
        }
        // 버퍼 앞으로 밀기
        for (int i = buffer_write_idx + 1; i <= buffer_last_idx; i++){
            for (int j = 0; j < 4 + (3+4)*4; j++){
                buffer_ptr[i-(buffer_write_idx+1)][j] = buffer_ptr[i][j];
            }
            // std::lock_guard<std::mutex> lock(global_index_mutex);
            buffer_global_index[i-(buffer_write_idx+1)] = buffer_global_index[i];
        }
        buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][BUFFER_NUM+2] = buffer_global_index[static_cast<uint32_t>(TopicType::POSE)][BUFFER_NUM+1];
    }    

    shm_ptr[0] = 1;
    buffer_last_idx = buffer_last_idx - buffer_write_idx;
    // std::cout << buffer_last_idx << std::endl;
}


void Parser::displayRGBImage() {
    cv::Mat image(HEIGHT, WIDTH, CV_8UC3);

    for (int i = 0; i < HEIGHT; ++i) {
        for (int j = 0; j < WIDTH; ++j) {
            int idx = i * WIDTH + j;
            image.at<cv::Vec3b>(i, j) = cv::Vec3b(rgb.b[idx], rgb.g[idx], rgb.r[idx]);
        }
    }

    write_locker = true;

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

    write_locker = true;

    // Depth 이미지 표시
    cv::imshow("Depth Image", depthImageNormalized);
    cv::waitKey(1); // 1ms 대기 (실시간 처리)
}