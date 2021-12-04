set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          aarch64)

# Without that flag CMake is not able to pass test compilation check
#set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)

SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

set(CROSS_COMPILE                   ${JETSON_TOOLCHAIN_PATH}aarch64-linux-gnu-)

SET(CMAKE_INSTALL_PREFIX "/jetsonfs")
set(CMAKE_SYSROOT ${CMAKE_INSTALL_PREFIX})
set(CMAKE_FIND_ROOT_PATH ${CMAKE_INSTALL_PREFIX})
set(CMAKE_PREFIX_PATH "/jetsonfs/opt/ros/melodic")
set(CMAKE_PREFIX_PATH "/jetsonfs/opt/ros/melodic/lib")
set(CMAKE_PREFIX_PATH "/jetsonfs/opt/ros/melodic/share")
set(CMAKE_FIND_USE_PACKAGE_ROOT_PATH FALSE)
set(CMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH FALSE)
set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH FALSE)
set(CMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY FALSE)
set(CMAKE_FIND_USE_CMAKE_SYSTEM_PATH FALSE)
list(APPEND CMAKE_PREFIX_PATH "/jetsonfs/jetsontoolchain/aarch64-linux-gnu/libc/")
list(APPEND CMAKE_PREFIX_PATH "/jetsonfs/opt/ros/melodic/")
list(APPEND CMAKE_PREFIX_PATH "/jetsonfs/lib/aarch64-linux-gnu/")
list(APPEND CMAKE_PREFIX_PATH "/jetsonfs/jetsontoolchain/aarch64-linux-gnu/include/c++/7.3.1/")
list(APPEND CMAKE_LIBRARY_PATH "/jetsonfs/usr/lib/aarch64-linux-gnu/")
list(APPEND CMAKE_LIBRARY_PATH "/jetsonfs/opt/ros/melodic/lib/")
list(APPEND CMAKE_LIBRARY_PATH "/jetsonfs/jetsontoolchain/aarch64-linux-gnu/libc/lib")
list(APPEND CMAKE_LIBRARY_PATH "/jetsonfs/jetsontoolchain/aarch64-linux-gnu/libc/usr/lib")
list(APPEND CMAKE_LIBRARY_PATH "/jetsonfs/opt/ros/melodic/lib/")
list(APPEND CMAKE_FRAMEWORK_PATH "/jetsonfs/usr/lib/aarch64-linux-gnu/")
list(APPEND CMAKE_FRAMEWORK_PATH "/jetsonfs/opt/ros/melodic/lib/")
list(APPEND CMAKE_PREFIX_PATH "/jetsonfs/jetsontoolchain/aarch64-linux-gnu/lib64/")
list(APPEND CMAKE_PREFIX_PATH "/jetsonfs/opt/ros/melodic/")
link_directories("/jetsonfs/jetsontoolchain/aarch64-linux-gnu/libc/")
link_directories("/jetsonfs/opt/ros/melodic/")
# link_directories("/jetsonfs/jetsontoolchain/aarch64-linux-gnu/libc/lib")
# link_directories("/jetsonfs/jetsontoolchain/aarch64-linux-gnu/libc/usr/lib")

set(BOOST_INCLUDEDIR                "/jetsonfs/usr/include")
set(BULLET_INCLUDE_DIR              "/jetsonfs/usr/include/bullet")
set(SDL_INCLUDE_DIR                 "/jetsonfs/usr/include")
set(SDL_IMAGE_INCLUDE_DIR           "/jetsonfs/usr/include")
set(PYTHON_INCLUDE_DIR              "/jetsonfs/usr/include/python3.6m")
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
set(RT_LIBRARY                      ${JETSON_TOOLCHAIN_PATH}../aarch64-linux-gnu/libc/usr/lib/librt.so)

set(CMAKE_C_FLAGS                   "-std=gnu++11 -Wno-psabi -fdata-sections -ffunction-sections -Wno-deprecated-declarations -Wl,--gc-sections -Wl,-rpath-link,/jetsonfs/jetsontoolchain/aarch64-linux-gnu/libc/lib  -Wl,-rpath-link,/jetsonfs/usr/lib/aarch64-linux-gnu/  -Wl,-rpath-link,/jetsonfs/lib/aarch64-linux-gnu/" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS                 "${CMAKE_C_FLAGS} -I/jetsontoolchain/aarch64-linux-gnu/include/c++/7.3.1/ -std=c++11 -Wno-write-strings" CACHE INTERNAL "")

set(EXTRA_LIBRARY_PATHS             "-L")

set(CMAKE_C_FLAGS_DEBUG             "-Os -g ${EXTRA_LIBRARY_PATHS}" CACHE INTERNAL "")
set(CMAKE_C_FLAGS_RELEASE           "-Os ${EXTRA_LIBRARY_PATHS} -DNDEBUG" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_C_FLAGS_DEBUG}" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_C_FLAGS_RELEASE}" CACHE INTERNAL "")

set(CMAKE_LIBRARY_ARCHITECTURE      "aarch64-linux-gnu")
set(ENABLE_PRECOMPILED_HEADERS      "OFF")
set(CMAKE_NO_SYSTEM_FROM_IMPORTED   "ON")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)