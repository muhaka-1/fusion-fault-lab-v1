# C target, built up module by module.
#   M7: E2E protection (CRC-8 + alive counter), the test framework, e2e_demo
#   M8: CAN message codecs (can_msg.c), can_vectors (reference frames for the Python contract test)
#   M9: cluster core, input rules R1-R5 and R10 (cluster_core.c), cluster_sim (timing diagram)
#   M10: ECU and nav supervision R6-R9, R11-R13
#   M11: the bus (UDP multicast), cluster_core and display_term as separate processes
#   M12: the EKF, chi-square gate and fault monitor in C (fc_nav), nav_replay for the parity test
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

add_library(fc_common STATIC src/common/e2e.c src/common/can_msg.c src/common/canbus_wire.c)
target_link_libraries(fc_common m)          # lround, fmod

add_library(fc_core STATIC src/core/cluster_core.c)
target_link_libraries(fc_core fc_common)

add_library(fc_nav STATIC src/nav/ekf.c src/nav/chi2.c src/nav/monitor.c)
target_link_libraries(fc_nav fc_common)

add_executable(e2e_demo tools/e2e_demo.c)
target_link_libraries(e2e_demo fc_common)

add_executable(can_vectors tools/can_vectors.c)
target_link_libraries(can_vectors fc_common)

add_executable(nav_replay tools/nav_replay.c)
target_link_libraries(nav_replay fc_nav)

add_executable(cluster_sim tools/cluster_sim.c)
target_link_libraries(cluster_sim fc_core)

add_executable(unit_tests tests/test_main.c tests/test_e2e.c tests/test_msg.c tests/test_core.c tests/test_nav.c)
target_link_libraries(unit_tests fc_core fc_nav)

# M11: everything that needs sockets and a monotonic clock (Linux, WSL, QNX; not MinGW)
if(NOT WIN32)
  add_library(fc_posix STATIC src/common/canbus_udp.c src/common/clock.c)
  target_compile_definitions(fc_posix PRIVATE _POSIX_C_SOURCE=200809L)
  target_link_libraries(fc_posix fc_common)
  if(QNX OR CMAKE_SYSTEM_NAME STREQUAL "QNX")
    target_link_libraries(fc_posix socket)
  endif()

  add_executable(cluster_core src/core/core_main.c)
  target_compile_definitions(cluster_core PRIVATE _POSIX_C_SOURCE=200809L)
  target_link_libraries(cluster_core fc_core fc_posix)

  add_executable(display_term src/display/display_term.c)
  target_compile_definitions(display_term PRIVATE _POSIX_C_SOURCE=200809L)
  target_link_libraries(display_term fc_posix)
endif()

enable_testing()
add_test(NAME unit_tests COMMAND unit_tests)
