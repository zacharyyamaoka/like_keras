# like_keras's C++ runtime for CMake (CONTRACT §6): the target like_keras::rt and lk_component().
#
#   include(/home/bam/like_keras/cmake/like_keras.cmake)
#   lk_component(arm_controller arm_controller.cpp)   # -> libarm_controller.so, which carries the C ABI (abi.h)
#
# CMake never runs Python (CONTRACT §7.4): the message views under cpp/include/like_keras/msgs are committed.
include_guard(GLOBAL)
find_package(Threads REQUIRED)
get_filename_component(LIKE_KERAS_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

# WHY an OBJECT library: every component .so carries the runtime and the lk_* entry points whole (a static archive
# would drop them, since nothing in the component calls them), and a host that only wants the headers links code
# it never calls. Hidden visibility: a .so exports the C ABI and nothing else.
add_library(like_keras_rt OBJECT ${LIKE_KERAS_ROOT}/cpp/src/rt.cpp ${LIKE_KERAS_ROOT}/cpp/src/abi.cpp)
target_include_directories(like_keras_rt PUBLIC ${LIKE_KERAS_ROOT}/cpp/include)
target_compile_features(like_keras_rt PUBLIC cxx_std_20)
target_link_libraries(like_keras_rt PUBLIC Threads::Threads)
set_target_properties(like_keras_rt PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden
                                               VISIBILITY_INLINES_HIDDEN ON)
add_library(like_keras::rt ALIAS like_keras_rt)

# lk_component(<target> <sources...>): one C++ System, exported with LK_EXPORT, as lib<target>.so.
function(lk_component target)
  add_library(${target} SHARED ${ARGN})
  target_link_libraries(${target} PRIVATE like_keras::rt)
  set_target_properties(${target} PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
endfunction()
