target_sources(h7_device PRIVATE
    cmd_port.c
    crc_check.c
    CRC8_CRC16.c
    fifo.c
    remote_control.c
)
target_include_directories(h7_device PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
