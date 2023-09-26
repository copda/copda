# set(CMAKE_C_COMPILER "gcc")
# set(CMAKE_C_FLAGS "-Wall -O3 -fomit-frame-pointer -funroll-loops")
add_library(cliquer
  src/cliquer.c
  src/cliquer.h
  src/graph.c
  src/graph.h
  src/reorder.c
  src/reorder.h
)

target_include_directories(cliquer INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
