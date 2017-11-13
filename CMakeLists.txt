cmake_minimum_required(VERSION 3.7)
project(GPS_INS_NEW)

set(CMAKE_C_STANDARD 11)
set(PROJECT_NAME GPS_INS_NEW)
set(SOURCE_FILES main.c MahonyAHRS.c NewKalmanFilter.c matrix_lc.c SVD.c Turnning.c QueueWindow.c)
add_executable(GPS_INS_NEW ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} -lm)