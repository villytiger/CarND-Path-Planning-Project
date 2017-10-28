project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

# Work around GCC 5.5 bug.
# https://gcc.gnu.org/bugzilla/show_bug.cgi?id=82346
add_definitions(-D_GLIBCXX_USE_C99 -D_GLIBCXX_USE_C99_MATH)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++1z")

file(GLOB_RECURSE sources src/pp/*.cpp src/pp/*.h)

include_directories("${CMAKE_SOURCE_DIR}/src")

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
