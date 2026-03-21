cmake_minimum_required(VERSION 3.16.3)
project(libpyflexiv VERSION 0.2.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find flexiv_rdk (resolved via CMAKE_PREFIX_PATH, e.g. ~/rdk_install).
# flexiv_rdk-config.cmake calls find_dependency(spdlog REQUIRED) internally,
# which sets spdlog_DIR to the spdlog cmake directory inside rdk_install.
find_package(flexiv_rdk REQUIRED)

# ---------------------------------------------------------------------------
# Force the spdlog headers from the cmake-discovered installation to come
# before conda's PyPI spdlog package headers.
#
# Problem: conda's `spdlog` PyPI package (v2.0.6, wrapping spdlog 1.10.0)
# installs C++ headers at $CONDA_PREFIX/include/python3.12/spdlog/.
# Pybind11 injects $CONDA_PREFIX/include/python3.12 as an early -isystem
# path, which shadows the correct rdk_install/include/spdlog (v1.14.1).
# That causes a compile-vs-link fmt ABI mismatch (fmt::v8 vs fmt::v10).
#
# Fix: derive the spdlog prefix from spdlog_DIR (set by find_dependency
# inside flexiv_rdk-config.cmake) and prepend its include dir globally.
# spdlog_DIR points to  <prefix>/lib/cmake/spdlog  → go up 3 levels.
# ---------------------------------------------------------------------------
if(spdlog_DIR)
    get_filename_component(_spdlog_prefix "${spdlog_DIR}/../../.." ABSOLUTE)
    include_directories(BEFORE SYSTEM "${_spdlog_prefix}/include")
    message(STATUS "Prioritizing spdlog headers from: ${_spdlog_prefix}/include")
else()
    message(WARNING "spdlog_DIR not set; cannot guarantee correct spdlog header priority")
endif()

# Find Python and pybind11
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c
        "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pybind11_result
)
if(NOT _pybind11_result EQUAL 0)
    message(FATAL_ERROR "Could not find pybind11. Install it with: pip install pybind11")
endif()
set(pybind11_DIR ${PYBIND11_CMAKE_DIR})
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_flexiv_rt
    src/flexiv_rt.cpp
    src/joint_impedance_control.cpp
    src/cartesian_control.cpp
)

target_include_directories(_flexiv_rt PRIVATE include/)

# GCC 13+ -Wfree-nonheap-object can false-positive on inlined std::vector in
# setTargetJoints; suppress only when the compiler supports the flag.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "13")
    target_compile_options(_flexiv_rt PRIVATE -Wno-free-nonheap-object)
endif()

# flexiv::flexiv_rdk transitively brings Eigen3, spdlog, fastrtps, fastcdr, Threads
target_link_libraries(_flexiv_rt PRIVATE flexiv::flexiv_rdk)

# Intercept mlockall(): the Flexiv Scheduler constructor calls mlockall(MCL_CURRENT|
# MCL_FUTURE) which pins ALL process virtual memory into physical RAM.  In a Python
# process with PyTorch/numpy this can be 10+ GB and triggers the OOM killer.
# --wrap redirects all calls to mlockall inside the linked static library to our
# __wrap_mlockall (a no-op that returns 0).  The Scheduler still gets SCHED_FIFO
# priority and semaphore — only the page-locking is skipped.
target_link_options(_flexiv_rt PRIVATE -Wl,--wrap=mlockall -Wl,--wrap=sem_unlink)

set_target_properties(_flexiv_rt PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rt
    OUTPUT_NAME "_flexiv_rt"
)

# Tests
option(BUILD_TESTS "Build C++ unit tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests/cpp)
endif()
