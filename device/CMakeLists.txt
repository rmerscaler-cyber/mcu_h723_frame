# Hardware-facing communication, chip drivers, and runtime tasks.
add_library(h7_device STATIC)
target_include_directories(h7_device PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/comm
    ${CMAKE_CURRENT_SOURCE_DIR}/driver
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/motor
    ${CMAKE_CURRENT_SOURCE_DIR}/tasks
)
target_link_libraries(h7_device PRIVATE h7_platform h7_model_api h7_bsp)

add_subdirectory(comm)
add_subdirectory(driver)
add_subdirectory(tasks)
