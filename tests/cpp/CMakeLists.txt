# Lane cpp's own C++ tests (not the ruler's kpi/): the box's counters and clock, on the box itself (cpp/src/box.hpp).
#
#   cmake -S /home/bam/like_keras/tests/cpp -B /home/bam/like_keras/build/tests-cpp -DCMAKE_BUILD_TYPE=Release
#   nice -n 19 cmake --build /home/bam/like_keras/build/tests-cpp -j 2
#   /home/bam/like_keras/build/tests-cpp/box_tests
#
# tests/test_cpp_box.py runs the same three steps under pytest. WHY a project of its own: the top-level
# CMakeLists.txt is the ruler's, and cmake/like_keras.cmake is what component builds include, so test targets go in
# neither.
cmake_minimum_required(VERSION 3.28)
project(like_keras_tests LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/like_keras.cmake)

find_package(GTest REQUIRED)
add_executable(box_tests test_box.cpp test_friction.cpp)
target_include_directories(box_tests PRIVATE ${LIKE_KERAS_ROOT}/cpp/src)
target_link_libraries(box_tests PRIVATE like_keras::rt GTest::gtest GTest::gtest_main ${CMAKE_DL_LIBS})

# CONTRACT §15 A44: nested tables given in part, driven through the .so's own C ABI by box_tests (dlopen) and through
# lk.native by tests/test_native_params.py
lk_component(row_tables row_tables.cpp)
add_dependencies(box_tests row_tables)
target_compile_definitions(box_tests PRIVATE LK_ROW_TABLES_SO="$<TARGET_FILE:row_tables>")

# CONTRACT §15 A28 (consumer-013): a component that links a static library compiled with default visibility, as the
# consumer's mcn_core: lk_component must still export the lk_* C ABI and nothing else (tests/test_component_exports.py).
add_library(default_visibility STATIC default_visibility.cpp)
set_target_properties(default_visibility PROPERTIES CXX_VISIBILITY_PRESET default POSITION_INDEPENDENT_CODE ON)
lk_component(links_static links_static.cpp)
target_link_libraries(links_static PRIVATE default_visibility)

# CONTRACT §15 A19: the RT ladder probe's box (tests/test_rt_ladder.py)
lk_component(spinner spinner.cpp)

# CONTRACT §15 A23: an exposed optional queue input, wired or left unwired by the deployment (tests/test_optional_queue.py)
lk_component(optional_queue optional_queue.cpp)

# CONTRACT §15 A28 (consumer-012): a line logged between two facility rounds, for a reader (tests/test_log_readers.py)
lk_component(log_once log_once.cpp)
