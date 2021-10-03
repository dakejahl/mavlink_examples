include_directories(${CMAKE_SOURCE_DIR}/c_library_v2/common)
include_directories(${CMAKE_SOURCE_DIR}/c_library_v2/a2z)

add_executable(winch
    a2zwinch.cpp
)
