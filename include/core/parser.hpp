#ifndef PARSER_HPP
#define PARSER_HPP

#include <deque> // std::deque 사용
#include <mutex>
#include <string>
#include <condition_variable>
#include <iostream>
#include <cstdint>
#include <array>

// # define BUFFER_NUM 2
# define USE_RENDERING
# define WIDTH 1280
# define HEIGHT 720
# define FRAGMENT_NUM 20

// struct image_shared {
//     // int width = WIDTH;
//     // int height = HEIGHT;
//     std::array<uint8_t, WIDTH * HEIGHT> r;   // RGB 데이터
//     std::array<uint8_t, WIDTH * HEIGHT> g;
//     std::array<uint8_t, WIDTH * HEIGHT> b;
//     std::array<float, WIDTH * HEIGHT> d;    // Depth 데이터
// };

struct topic_rgb_t {
    int width = WIDTH;
    int height = HEIGHT;
    std::array<uint8_t, WIDTH * HEIGHT> r, g, b, a;
};

struct topic_d_t {
    int width = WIDTH;
    int height = HEIGHT;
    std::array<float, WIDTH * HEIGHT> d;
};

struct topic_pose_t {
    std::array<float, 3> pos;
    std::array<float, 4> rot;
};

enum class TopicType {
    RGB,
    DEPTH,
    POSE
};

class Parser {
public:
    Parser(int topic, TopicType image_type_);
    ~Parser();

    void push(const std::string& payload);
    std::string pop();

    void consume();
    void parsing(std::string payload);
    void displayRGBImage();
    void displayDepthImage();
    bool fragmentChecker();

    TopicType topic_type;
    std::string topic_idx;
    std::string payload;

    // shared memory
    std::string shm_name;
    int shm_fd;
    size_t shm_size;
    uint8_t* shm_ptr;

    int activeBuffer;               // 활성 버퍼 인덱스 (0 또는 1)
    uint32_t global_index;
    uint32_t fragment_index;
    // uint32_t fragment_index_prev;
    topic_rgb_t rgb;
    topic_d_t depth;
    topic_pose_t pose;
    std::array<bool, 20> fragment_checker; 

private:
    std::string latestPayload;            // 가장 최근의 메시지
    std::mutex dataMutex;                 // 데이터 보호를 위한 mutex
    std::condition_variable dataCondition;
};

#endif // PARSER_HPP
