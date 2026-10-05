# IMU service
target_sources(h7_model PRIVATE
    imu.c
)

target_include_directories(h7_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
