# C target, built up module by module.
#   M7: E2E protection (CRC-8 + alive counter), the test framework, e2e_demo
#   M8: CAN message codecs (can_msg.c), can_vectors (reference frames for the Python contract test)
cmake_minimum_required(VERSION 3.16)
project(fusion_target C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
# warnings are errors: the code must compile cleanly on gcc and clang
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion -Werror)

option(SANITIZE "Build with address + undefined-behaviour sanitizers (Linux/macOS)" OFF)
if(SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

include_directories(include)

add_library(fc_common STATIC src/common/e2e.c src/common/can_msg.c)
target_link_libraries(fc_common m)          # lround, fmod

add_executable(e2e_demo tools/e2e_demo.c)
target_link_libraries(e2e_demo fc_common)

add_executable(can_vectors tools/can_vectors.c)
target_link_libraries(can_vectors fc_common)

add_executable(unit_tests tests/test_main.c tests/test_e2e.c tests/test_msg.c)
target_link_libraries(unit_tests fc_common)

enable_testing()
add_test(NAME unit_tests COMMAND unit_tests)
