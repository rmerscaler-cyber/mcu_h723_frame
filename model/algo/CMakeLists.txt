# Algorithms and control utilities
target_sources(h7_model PRIVATE
    AHRS_middleware.c
    ahrs.c
    anglecalc.c
    cmsis_dsp_compat.c
    filter.c
    kalman.c
    kalman_filter.c
    MahonyAHRS.c
    pid.c
    user_lib.c
)

target_include_directories(h7_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
)
