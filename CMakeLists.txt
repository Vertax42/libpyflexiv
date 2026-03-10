cmake_minimum_required(VERSION 3.16.3)
project(libpyflexiv VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find flexiv_rdk (resolved via CMAKE_PREFIX_PATH, e.g. ~/rdk_install)
find_package(flexiv_rdk REQUIRED)

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

# Ensure we always compile against the same spdlog headers as the linked
# spdlog library. In conda envs, pybind11 often injects Python include dirs
# early, and some envs contain stale C++ spdlog headers under python include.
# Prepending spdlog::spdlog include dirs avoids ABI mismatches (fmt v8/v10).
if(TARGET spdlog::spdlog)
    get_target_property(_spdlog_inc_dirs spdlog::spdlog INTERFACE_INCLUDE_DIRECTORIES)
    if(_spdlog_inc_dirs)
        target_include_directories(_flexiv_rt BEFORE PRIVATE ${_spdlog_inc_dirs})
        message(STATUS "Prioritizing spdlog headers from: ${_spdlog_inc_dirs}")
    endif()
endif()

# Some spdlog builds use external fmt (SPDLOG_FMT_EXTERNAL) but don't
# consistently propagate fmt on all downstream link lines. Explicitly link
# fmt when that compile mode is active to avoid runtime undefined fmt symbols.
if(TARGET spdlog::spdlog)
    get_target_property(_spdlog_defs spdlog::spdlog INTERFACE_COMPILE_DEFINITIONS)
    if(_spdlog_defs)
        string(REPLACE ";" " " _spdlog_defs_str "${_spdlog_defs}")
        if(_spdlog_defs_str MATCHES "(^| )SPDLOG_FMT_EXTERNAL($| )")
            if(TARGET fmt::fmt)
                target_link_libraries(_flexiv_rt PRIVATE fmt::fmt)
                message(STATUS "spdlog uses external fmt; linking fmt::fmt explicitly")
            else()
                message(FATAL_ERROR
                    "spdlog::spdlog is configured with SPDLOG_FMT_EXTERNAL, but fmt::fmt is missing. "
                    "Set CMAKE_PREFIX_PATH to a prefix containing matching spdlog+fmt (e.g. ~/rdk_install)."
                )
            endif()
        endif()
    endif()
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
