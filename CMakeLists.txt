# gltf2md5 -- CMake build script
#
# Generate a Visual Studio 2022 solution:
#   generate_vs2022.bat            (Windows convenience wrapper)
#
# Or directly:
#   cmake -B build -G "Visual Studio 17 2022" -A x64
#   cmake --build build --config Release
#
# Linux / macOS / WSL:
#   cmake -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build -j

cmake_minimum_required(VERSION 3.15)
project(gltf2md5
    VERSION   1.0.0
    LANGUAGES C CXX
    DESCRIPTION "glTF 2.0 to idTech4 MD5 mesh/anim converter"
)

# ---- Language standards ----------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---- Default to Release for single-config generators -----------------------

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Build type (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        Debug Release RelWithDebInfo MinSizeRel)
endif()

# ---- Source files ----------------------------------------------------------

set(GLTF2MD5_SOURCES
    src/gltf2md5.cpp
    src/mikktspace.c
)

set(GLTF2MD5_HEADERS
    src/cgltf.h
    src/mikktspace.h
)

# ---- Target ----------------------------------------------------------------

add_executable(gltf2md5
    ${GLTF2MD5_SOURCES}
    ${GLTF2MD5_HEADERS}
)

target_include_directories(gltf2md5 PRIVATE src)

# ---- Per-compiler tuning ---------------------------------------------------

if(MSVC)
    target_compile_options(gltf2md5 PRIVATE
        /W3             # reasonable warning level
        /MP             # multi-processor compilation
        /permissive-    # strict conformance
    )
    target_compile_definitions(gltf2md5 PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _CRT_NONSTDC_NO_WARNINGS
    )
    # Avoid relying on vcredist by linking the runtime statically.
    # Comment this block out if you prefer the dynamic CRT.
    set_property(TARGET gltf2md5 PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    target_compile_options(gltf2md5 PRIVATE
        -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
    )
    target_link_libraries(gltf2md5 PRIVATE m)
endif()

# ---- Visual Studio project layout & startup --------------------------------

if(MSVC)
    source_group("Source Files"  FILES src/gltf2md5.cpp src/mikktspace.c)
    source_group("Header Files"  FILES src/cgltf.h src/mikktspace.h)

    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        PROPERTY VS_STARTUP_PROJECT gltf2md5)
endif()

# ---- Output goes to project-root /bin/<config>/ ----------------------------

set_target_properties(gltf2md5 PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY                "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG          "${CMAKE_SOURCE_DIR}/bin/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE        "${CMAKE_SOURCE_DIR}/bin/Release"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_SOURCE_DIR}/bin/RelWithDebInfo"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     "${CMAKE_SOURCE_DIR}/bin/MinSizeRel"
)

# ---- Status print ----------------------------------------------------------

message(STATUS "gltf2md5 ${PROJECT_VERSION}  (CXX=${CMAKE_CXX_COMPILER_ID})")
