cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 23)
project(ejemplo1)

find_package(TBB REQUIRED)

# Specify construction and link process
add_executable( ejemplo1 Threadmain.cpp )
target_link_libraries(ejemplo1 PUBLIC TBB::tbb)

add_executable(ejemplo2 RandomMain.cpp)

add_executable(ejemplo3 InitializerMain.cpp)

add_executable(ejemplo4 TransformsMain.cpp)