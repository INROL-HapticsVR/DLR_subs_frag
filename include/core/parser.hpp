#ifndef PARSER_HPP
#define PARSER_HPP

#include <deque> // std::deque 사용
#include <mutex>
#include <string>
#include <condition_variable>
#include <iostream>
#include <cstdint>
#include <array>
#include <vector>
#include <mutex>


// # define USE_RENDERING
# define WIDTH 1280
# define HEIGHT 720
# define FRAGMENT_NUM 20
# define BUFFER_NUM 10


// // 전역 동기화 객체
// extern std::vector<uint32_t> global_check;
// extern std::mutex global_check_mutex;
// extern std::vector<std::vector<uint32_t>> buffer_global_index;
// extern std::mutex global_index_mutex;

extern std::array<std::array<uint32_t, BUFFER_NUM + 3>, 3> buffer_global_index;
extern std::mutex global_index_mutex;

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

    void stack();
    void parsing(std::string payload);
    void displayRGBImage();
    void displayDepthImage();
    bool fragmentChecker();

    void bufferSaver();
    void indexSaver();
    void indexChecker();
    bool writeChecker();
    void shmWrite();
    void bufferManager();
    

    TopicType topic_type;
    std::string topic_idx;

    std::array<uint8_t, 4*WIDTH * HEIGHT> payload;
    std::array<uint8_t, 4*WIDTH * HEIGHT> payload_tmp;

    // shared memory
    std::string shm_name;
    int shm_fd;
    size_t shm_size;
    uint8_t* shm_ptr;

    // buffer
    uint8_t** buffer_ptr;
    int buffer_last_idx;

    uint32_t global_index;
    uint32_t fragment_index;
    int write_index;

    topic_rgb_t rgb;
    topic_d_t depth;
    topic_pose_t pose;
    std::array<bool, 20> fragment_checker; 
    bool write_locker;

    std::string latestPayload;            // 가장 최근의 메시지
    std::mutex dataMutex;                 // 데이터 보호를 위한 mutex
    std::condition_variable dataCondition;
};

#endif // PARSER_HPP
