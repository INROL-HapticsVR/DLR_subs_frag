cmake_minimum_required(VERSION 3.10)

# 프로젝트 설정
project(DLR_subs VERSION 1.0 LANGUAGES CXX)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Mosquitto 소스 및 빌드 디렉토리 설정
set(MOSQUITTO_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/mosquitto)
set(MOSQUITTO_LIB_DIR ${MOSQUITTO_SOURCE_DIR}/build/lib)
set(MOSQUITTO_INCLUDE_DIR ${MOSQUITTO_SOURCE_DIR}/include)

# # Mosquitto 라이브러리와 헤더 확인
# if(NOT EXISTS "${MOSQUITTO_LIB_DIR}/libmosquitto.so")
#     message(FATAL_ERROR "Mosquitto 라이브러리가 ${MOSQUITTO_LIB_DIR}/libmosquitto.so 경로에 존재하지 않습니다. Mosquitto를 먼저 빌드하세요.")
# endif()

# if(NOT EXISTS "${MOSQUITTO_INCLUDE_DIR}/mosquitto.h")
#     message(FATAL_ERROR "Mosquitto 헤더 파일이 ${MOSQUITTO_INCLUDE_DIR}/mosquitto.h 경로에 존재하지 않습니다.")
# endif()

# Mosquitto 헤더 파일 경로 추가
include_directories(
    ${MOSQUITTO_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Mosquitto 라이브러리 경로 추가
link_directories(
    ${MOSQUITTO_LIB_DIR}
)

# OpenCV 라이브러리 설정
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 소스 파일 추가
set(SOURCES
    # src/core/consumer.cpp
    src/core/parser.cpp
    src/core/subs.cpp
    src/main.cpp
)

# 실행 파일 생성
add_executable(${PROJECT_NAME} ${SOURCES})

# Mosquitto, OpenCV, pthread 및 librt 라이브러리 링크
target_link_libraries(${PROJECT_NAME} PRIVATE mosquitto pthread ${OpenCV_LIBS} rt)
