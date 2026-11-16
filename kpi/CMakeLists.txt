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

  # CONTRACT §15 A27 (ruler M0.3): one box per consumer decision A16-A26 that needs C++. Each is kept out of `all`
  # and built by its own test (kpi/support/lk.py feature_library), so a decision that has not landed fails its one
  # test and never breaks the build of the rest of the ruler.
  lk_component(joint_names polyglot/joint_names.cpp)      # A16 (consumer-001)
  lk_component(kept_plan polyglot/kept_plan.cpp)          # A17 (consumer-002)
  lk_component(hand_back facilities/hand_back.cpp)        # A18 (consumer-003)
  lk_component(rt_ladder blackbox/rt_ladder.cpp)          # A19 (consumer-004)
  lk_component(period_probe consumer/period_probe.cpp)    # A20 (consumer-005)
  lk_component(self_health facilities/self_health.cpp)    # A22 (consumer-007)
  lk_component(verdict polyglot/verdict.cpp)              # A23 (consumer-008)
  lk_component(table_params facilities/table_params.cpp)  # A24 (consumer-009)
  lk_component(arrivals consumer/arrivals.cpp)            # A25 (consumer-010)
  # Ruler M0.4: the library fix round 1's defects, built the same way.
  lk_component(optional_queue polyglot/optional_queue.cpp)  # A23 for a queue input (A30)
  lk_component(log_steps consumer/log_steps.cpp)            # A30, T1: a native log line at the step that logged it
  lk_component(tie_health facilities/tie_health.cpp)        # A22, §4.5: a tie keeps the facility's message
  # Ruler M0.5 (CONTRACT §15 A38): the second consumer round's decisions that need C++ (A31, A33, A34).
  lk_component(owner_conversion polyglot/owner_conversion.cpp)  # A31 (consumer-014): from(const T&, const Owner&)
  lk_component(non_finite facilities/non_finite.cpp)            # A33 (consumer-016): NaN and +-inf defaults
  lk_component(refuser consumer/refuser.cpp)                    # A34 (consumer-017): a refusal is final under dora
  # Ruler M0.7 (CONTRACT §15 A43): A41 and A42 (consumer-020, consumer-021).
  lk_component(described_params facilities/described_params.cpp)  # A41: describe() lists a native box's defaults
  lk_component(refused_pushes facilities/refused_pushes.cpp)      # A42: refused pushes are counted and logged
  set_target_properties(joint_names kept_plan hand_back rt_ladder period_probe self_health verdict table_params
                        arrivals optional_queue log_steps tie_health owner_conversion non_finite refuser
                        described_params refused_pushes
                        PROPERTIES EXCLUDE_FROM_ALL ON)

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
