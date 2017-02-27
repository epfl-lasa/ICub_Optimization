cmake_minimum_required (VERSION 2.6)

set(SNOPT "${CMAKE_CURRENT_SOURCE_DIR}/../snopt/")
set(iCUBSDFAST "${CMAKE_CURRENT_SOURCE_DIR}/../icub-sdfast/")

file(GLOB ICUBOPT_SOURCES "src/*.cpp")
file(GLOB STACK_SOURCES "test.cpp")

include_directories (include/)
include_directories (${SNOPT}/include/)
include_directories (${iCUBSDFAST}/codes/include/)
include_directories (${iCUBSDFAST}/sdfast/include/)

link_directories    (${SNOPT}/lib/)
link_directories    (${iCUBSDFAST}/build/)

add_definitions("-std=gnu++0x")
set(CMAKE_CXX_FLAGS "-O3 -fno-omit-frame-pointer -pg -fPIE")

add_library (iCubOpt ${ICUBOPT_SOURCES})
TARGET_LINK_LIBRARIES(iCubOpt Model)

add_executable (test ${STACK_SOURCES})
TARGET_LINK_LIBRARIES(test iCubOpt snopt7_cpp snopt7 Model dl)

