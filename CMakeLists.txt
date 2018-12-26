cmake_minimum_required(VERSION 3.7.2)
project(sensorics VERSION 1.1.0 LANGUAGES C)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_C_STANDARD 11)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(APR_COMPILE_FLAGS "-DLINUX -D_REENTRANT -D_GNU_SOURCE")

include(${CMAKE_MODULE_PATH}/FindAPR.cmake)
find_package(Threads REQUIRED)

file(GLOB_RECURSE bme_sources src/modules/*.c)
add_executable(bmeserver ${CMAKE_SOURCE_DIR}/src/main/bmeserver.c ${bme_sources})
set_target_properties(bmeserver PROPERTIES COMPILE_FLAGS "${APR_COMPILE_FLAGS} -Wall -Wextra -Werror -O3")
target_include_directories(bmeserver PUBLIC ${CMAKE_SOURCE_DIR}/include ${APR_INCLUDE_DIR} ${APRUTIL_INCLUDE_DIR})
target_link_libraries(bmeserver PUBLIC ${APR_LIBRARIES} ${APRUTIL_LIBRARIES} Threads::Threads)

add_executable(simpleserver ${PROJECT_SOURCE_DIR}/src/main/simpleserver.c ${PROJECT_SOURCE_DIR}/src/server/server.c)
set_target_properties(simpleserver PROPERTIES COMPILE_FLAGS "${APR_COMPILE_FLAGS} -Wall -Wextra -Werror -O3")
target_include_directories(simpleserver PUBLIC ${CMAKE_SOURCE_DIR}/include ${APR_INCLUDE_DIR} ${APRUTIL_INCLUDE_DIR})
target_link_libraries(simpleserver PUBLIC ${APR_LIBRARIES} ${APRUTIL_LIBRARIES} Threads::Threads)

