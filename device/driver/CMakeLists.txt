target_sources(h7_device PRIVATE
    BMI088driver.c
    BMI088Middleware.c
    imu_io.c
    ist8310driver.c
    ist8310driver_middleware.c
)
target_include_directories(h7_device PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(motor)
