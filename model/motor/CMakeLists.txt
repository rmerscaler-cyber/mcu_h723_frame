# Protocol-independent motor control
target_sources(h7_model PRIVATE
    motor.c
)

target_include_directories(h7_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
