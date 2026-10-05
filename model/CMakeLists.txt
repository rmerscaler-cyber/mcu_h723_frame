# Public model contracts are header-only. Device ports implement these contracts
# without linking back to the model implementation.
add_library(h7_model_api INTERFACE)
target_include_directories(h7_model_api INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/algo
    ${CMAKE_CURRENT_SOURCE_DIR}/algo/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/cmd
    ${CMAKE_CURRENT_SOURCE_DIR}/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/motor
)
target_link_libraries(h7_model_api INTERFACE h7_types)

# Device models and intermediate control logic.
add_library(h7_model STATIC)
target_link_libraries(h7_model PUBLIC h7_model_api)
target_link_libraries(h7_model PRIVATE h7_platform)

add_subdirectory(algo)
add_subdirectory(cmd)
add_subdirectory(imu)
add_subdirectory(motor)
