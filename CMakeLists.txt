cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 20)
project(ejemplo1)
# Sources set
SET ( SOURCES  ${SOURCES}  main.cpp )
# Headers set
SET ( HEADERS  ${HEADERS}  )

# Specify construction and link process
ADD_EXECUTABLE( ejemplo1 ${SOURCES} ${HEADERS} )