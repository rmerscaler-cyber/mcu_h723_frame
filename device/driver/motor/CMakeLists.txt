# Motor protocol implementations and CAN board adapter.
target_sources(h7_device PRIVATE
    dji_motor.c
    lk_motor.c
    motor_port.c
    motor_reg.c
)

target_include_directories(h7_device PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
