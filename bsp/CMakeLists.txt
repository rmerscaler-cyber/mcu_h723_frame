# Board support package (BSP)
add_library(h7_bsp STATIC)
target_sources(h7_bsp PRIVATE
    bsp_adc.c
    bsp_buzzer.c
    bsp_can.c
    bsp_delay.c
    bsp_exti.c
    bsp_flash.c
    bsp_iic.c
    bsp_imu_pwm.c
    bsp_led.c
    bsp_rc.c
    bsp_spi.c
    bsp_usart.c
    io.c
)

target_include_directories(h7_bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(h7_bsp PRIVATE h7_platform h7_types)
