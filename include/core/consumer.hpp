#ifndef CONSUMER_HPP
#define CONSUMER_HPP

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
    bool globalChecker();
    int indexChecker();
    void bufferManager();
    void shmWrite();

    TopicType topic_type;
    std::string topic_idx;

    std::array<uint8_t, 4*WIDTH * HEIGHT> payload;
    std::array<uint8_t, 4*WIDTH * HEIGHT> payload_tmp;
    // std::string payload_tmp;

    // shared memory
    std::string shm_name;
    int shm_fd;
    size_t shm_size;
    uint8_t* shm_ptr;
    uint8_t** buffer_ptr;
    int buffer_last_idx;
    int buffer_write_idx;

    int activeBuffer;               // 활성 버퍼 인덱스 (0 또는 1)
    uint32_t global_index;
    uint32_t global_index_prev;
    uint32_t global_index_write;
    uint32_t global_index_write_prev;

    uint32_t fragment_index;
    // uint32_t fragment_index_prev;
    topic_rgb_t rgb;
    topic_d_t depth;
    topic_pose_t pose;
    std::array<bool, 20> fragment_checker; 

    bool write_locker;
    std::string latestPayload;            // 가장 최근의 메시지
    std::mutex dataMutex;                 // 데이터 보호를 위한 mutex
    std::condition_variable dataCondition;
};

#endif // CONSUMER_HPP
