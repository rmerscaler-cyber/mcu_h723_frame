target_sources(h7_device PRIVATE
    detect_task.c
    INS_task.c
    led_flow_task.c
    referee.c
    referee_usart_task.c
)
target_include_directories(h7_device PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
