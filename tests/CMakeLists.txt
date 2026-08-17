if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required(VERSION 3.10)
  project(fast_lio_effective_transition_tests LANGUAGES CXX)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)

if(NOT DEFINED CATKIN_DEVEL_PREFIX)
  find_package(GTest REQUIRED)
endif()

enable_testing()

function(add_fast_lio_contract_test target source)
  if(DEFINED CATKIN_DEVEL_PREFIX)
    catkin_add_gtest(${target} ${source} gtest_main.cpp)
  else()
    add_executable(${target} ${source})
  endif()
  target_include_directories(
    ${target}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../include
  )
  if(DEFINED CATKIN_DEVEL_PREFIX)
    target_link_libraries(${target} Eigen3::Eigen OpenMP::OpenMP_CXX)
  else()
    target_link_libraries(
      ${target}
      PRIVATE
        Eigen3::Eigen
        GTest::gtest
        GTest::gtest_main
        OpenMP::OpenMP_CXX
    )
    add_test(NAME ${target} COMMAND ${target})
  endif()
endfunction()

add_fast_lio_contract_test(
  test_effective_transition
  test_effective_transition.cpp
)
add_fast_lio_contract_test(
  test_effective_transition_manifolds
  test_effective_transition_manifolds.cpp
)
