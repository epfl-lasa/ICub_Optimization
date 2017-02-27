cmake_minimum_required (VERSION 2.6)

set(SDFAST "${CMAKE_CURRENT_SOURCE_DIR}/sdfast/")
set(CODES "${CMAKE_CURRENT_SOURCE_DIR}/codes/")

file(GLOB MODEL_SOURCES "${SDFAST}/src/*.c")
file(GLOB EXTRA_SOURCES "${CODES}/src/*.cpp")
file(GLOB STACK_SOURCES "main.cpp")
include_directories (${CODES}/include/)
include_directories (${SDFAST}/include/)

add_library (Model ${MODEL_SOURCES} ${EXTRA_SOURCES})
add_executable (test ${STACK_SOURCES})
TARGET_LINK_LIBRARIES(test Model dl)
