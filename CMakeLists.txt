cmake_minimum_required(VERSION 3.7.2)
project(sensorics VERSION 1.1.0 LANGUAGES C)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_C_STANDARD 11)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(APR_COMPILE_FLAGS "-DLINUX -D_REENTRANT -D_GNU_SOURCE")
set(PROD_COMPILE_FLAGS "-O3 -Wall -Wextra -Werror")
set(SRC_MAIN_DIR "${CMAKE_SOURCE_DIR}/src/main")
set(SRC_MODULES_DIR "${CMAKE_SOURCE_DIR}/src/modules")

find_package(SQLite3)
find_package(APR)
find_package(Threads)

file(GLOB_RECURSE src_modules_all ${SRC_MODULES_DIR}/*.c)
file(GLOB_RECURSE src_modules_sql ${SRC_MODULES_DIR}/sql/*.c)
file(GLOB_RECURSE src_modules_i2c ${SRC_MODULES_DIR}/i2c/*.c)
file(GLOB_RECURSE src_modules_srv ${SRC_MODULES_DIR}/server/*.c)

set(src_modules_no_sql ${src_modules_all})
set(src_modules_no_i2c ${src_modules_all})
set(src_modules_no_srv ${src_modules_all})
list(REMOVE_ITEM src_modules_no_sql ${src_modules_sql})
list(REMOVE_ITEM src_modules_no_i2c ${src_modules_i2c})
list(REMOVE_ITEM src_modules_no_srv ${src_modules_srv})

#
# targets
#

add_executable(bmeserver ${SRC_MAIN_DIR}/bmeserver.c ${src_modules_no_sql})
set_target_properties(bmeserver PROPERTIES COMPILE_FLAGS "${APR_COMPILE_FLAGS} ${PROD_COMPILE_FLAGS}")
target_include_directories(bmeserver PUBLIC ${CMAKE_SOURCE_DIR}/include ${APR_INCLUDE_DIR} ${APRUTIL_INCLUDE_DIR})
target_link_libraries(bmeserver PUBLIC ${APR_LIBRARIES} ${APRUTIL_LIBRARIES} Threads::Threads)
