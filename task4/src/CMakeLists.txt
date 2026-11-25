# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/controller.cpp
  ../src/personTracker.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/controller.h
  ../src/personTracker.h
)


LIST(APPEND LIBS ${LIBS})

add_compile_options(
    -O3 
    -march=native 
    -flto=auto
    -fno-plt
)
add_definitions(-DNDEBUG -fmax-errors=1)
add_link_options(-flto=auto)

