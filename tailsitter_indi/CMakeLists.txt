px4_add_module(
    MODULE modules__tailsitter_indi
    MAIN tailsitter_indi
    STACK_MAIN 2000
    SRCS
        tailsitter_indi.cpp
        tailsitter_indi.hpp
    MODULE_CONFIG
        tailsitter_indi_params.yaml
    DEPENDS
        mathlib
        px4_work_queue
)
