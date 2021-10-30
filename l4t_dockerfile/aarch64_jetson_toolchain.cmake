set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          aarch64)

# Without that flag CMake is not able to pass test compilation check
#set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)

SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

set(CROSS_COMPILE                   ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-)

set(_CMAKE_TOOLCHAIN_PREFIX         ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-gcc-${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_AR                        ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-gcc-ar${CMAKE_EXECUTABLE_SUFFIX} CACHE FILEPATH "Archiver")
set(CMAKE_ASM_COMPILER              ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-gcc${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_C_COMPILER                ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-gcc${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_CXX_COMPILER              ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-g++${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_LINKER                    ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-ld${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_OBJCOPY                   ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-objcopy${CMAKE_EXECUTABLE_SUFFIX} CACHE INTERNAL "")
set(CMAKE_RANLIB                    ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-ranlib${CMAKE_EXECUTABLE_SUFFIX} CACHE INTERNAL "")
set(CMAKE_SIZE                      ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-size${CMAKE_EXECUTABLE_SUFFIX} CACHE INTERNAL "")
set(CMAKE_STRIP                     ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-strip${CMAKE_EXECUTABLE_SUFFIX} CACHE INTERNAL "")

set(CMAKE_C_FLAGS                   "-Wno-psabi -fdata-sections -ffunction-sections -Wl,--gc-sections" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS                 "${CMAKE_C_FLAGS} -std=c++11 -Wno-write-strings" CACHE INTERNAL "")

set(CMAKE_C_FLAGS_DEBUG             "-Os -g" CACHE INTERNAL "")
set(CMAKE_C_FLAGS_RELEASE           "-Os -DNDEBUG" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_C_FLAGS_DEBUG}" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_C_FLAGS_RELEASE}" CACHE INTERNAL "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)