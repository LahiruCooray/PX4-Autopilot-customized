px4_add_module(
    MODULE modules__residual_test
    MAIN residual_test
    SRCS
        residual_test.cpp
    DEPENDS
        px4_work_queue
)

