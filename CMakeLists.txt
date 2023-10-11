cmake_minimum_required(VERSION 3.21)

project(cbc_demo CXX)

if (NOT CMAKE_BUILD_TYPE)
      set(CMAKE_BUILD_TYPE Release)
endif()


if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
else()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()

set(sources
      main.cpp
)

add_executable(${CMAKE_PROJECT_NAME} ${sources})
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE cxx_std_17)

target_link_directories(${CMAKE_PROJECT_NAME} PUBLIC "${CMAKE_SOURCE_DIR}/third_party/Cbc/Cbc-releases.2.10.9-x86_64-ubuntu20-gcc940/lib/")
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC "${CMAKE_SOURCE_DIR}/third_party/Cbc/Cbc-releases.2.10.9-x86_64-ubuntu20-gcc940/include/coin")
set(CBC_LIBRARY_LIST ${LIBRARY_LIST} "CoinUtils" "Osi" "Cgl" "Clp" "ClpSolver" "OsiClp" "Cbc" "CbcSolver" "OsiCbc")
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${CBC_LIBRARY_LIST})

file(COPY "${CMAKE_SOURCE_DIR}/third_party/Cbc/Cbc-releases.2.10.9-x86_64-ubuntu20-gcc940/lib/" DESTINATION "${CMAKE_BINARY_DIR}")