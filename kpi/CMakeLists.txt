# The ruler's C++ targets (lane RULER; frozen after M0). Everything lands in build/kpi/.
#
# Built always: the fake native .so files of kpi/miswire (plain C, no like_keras).
# Built once cmake/like_keras.cmake (lane CPP) provides lk_component(<target> <sources...>), which makes a component
# .so lib<target>.so, and the target like_keras::rt, whose include directories hold like_keras/lk.hpp, abi.h and the
# generated msgs/*.hpp (the ruler's reading of the CMake seam: docs/friction/contract-002.md):
#   libarm_controller.so, libempty.so, libcpp_gain.so, libecho.so, libchain4.so   (components)
#   alloc_probe (gtest), bench_chain (Google Benchmark)                              (the gate and criterion 2)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/kpi)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/kpi)
set(KPI_BUILD_DIR ${CMAKE_BINARY_DIR}/kpi)

foreach(variant ok skew abi2)
  string(TOUPPER ${variant} VARIANT)
  add_library(fake_native_${variant} SHARED miswire/fake_native.c)
  target_compile_definitions(fake_native_${variant} PRIVATE FAKE_${VARIANT})
  target_include_directories(fake_native_${variant} PRIVATE miswire)
endforeach()

if(COMMAND lk_component AND TARGET like_keras::rt)
  # WHY -ffp-contract=off: the JTC law must round as its Python twin does (T1 holds them to 1e-9 rad); a fused
  # multiply-add would round differently (robot_controllers' joint_trajectory_controller.hpp says the same).
  lk_component(arm_controller tasks/arm_cell/arm_controller.cpp)
  target_compile_options(arm_controller PRIVATE -ffp-contract=off)
  lk_component(empty tasks/empty/empty.cpp)
  add_subdirectory(tasks/cpp_gain)
  lk_component(echo polyglot/echo.cpp)
  lk_component(chain4 bench/chain4.cpp)

  find_package(GTest REQUIRED)
  add_executable(alloc_probe alloc_probe.cpp)
  target_include_directories(alloc_probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_definitions(alloc_probe PRIVATE KPI_BUILD_DIR="${KPI_BUILD_DIR}" KPI_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(alloc_probe PRIVATE GTest::gtest GTest::gtest_main ${CMAKE_DL_LIBS} pthread)
  set_target_properties(alloc_probe PROPERTIES ENABLE_EXPORTS ON)  # -rdynamic: the replacements interpose on the .so
  add_dependencies(alloc_probe arm_controller)

  find_package(benchmark REQUIRED)
  add_executable(bench_chain bench/bench_chain.cpp)
  target_compile_definitions(bench_chain PRIVATE KPI_BUILD_DIR="${KPI_BUILD_DIR}")
  target_link_libraries(bench_chain PRIVATE benchmark::benchmark like_keras::rt ${CMAKE_DL_LIBS})
  add_dependencies(bench_chain chain4)
else()
  message(STATUS "kpi: the like_keras C++ runtime (cmake/like_keras.cmake: lk_component, like_keras::rt) is absent; "
                 "only the fake native .so files are built")
endif()
