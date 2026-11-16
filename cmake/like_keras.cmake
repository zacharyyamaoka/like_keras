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
# WHY the version script and --exclude-libs (CONTRACT §15 A28, consumer-013): hidden visibility reaches only the
# component's own sources, so a .so exported every static library it linked (182 mcn:: and mjbots:: symbols in the
# consumer's) and five std:: instantiations of the runtime itself; two components in one process then bound each
# other's code. Now every .so this builds exports the lk_* C ABI and nothing else, whatever it links.
function(lk_component target)
  add_library(${target} SHARED ${ARGN})
  target_link_libraries(${target} PRIVATE like_keras::rt)
  set_target_properties(${target} PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON
                                             LINK_DEPENDS ${LIKE_KERAS_ROOT}/cmake/lk_component.map)
  target_link_options(${target} PRIVATE "LINKER:--version-script=${LIKE_KERAS_ROOT}/cmake/lk_component.map"
                      "LINKER:--exclude-libs,ALL")
endfunction()
