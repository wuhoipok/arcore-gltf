# Sets the minimum version of CMake required to build your native library.
# This ensures that a certain set of CMake features is available to
# your build.

cmake_minimum_required(VERSION 3.4.1)

# Specifies a library name, specifies whether the library is STATIC or
# SHARED, and provides relative paths to the source code. You can
# define multiple libraries by adding multiple add_library() commands,
# and CMake builds them for you. When you build your app, Gradle
# automatically packages shared libraries with your APK.

add_library( # Specifies the name of the library.
        tiny_gltf

        # Sets the library as a shared library.
        SHARED

        # Provides a relative path to your source file(s).
        com_google_ar_core_examples_java_common_samplerender_tiny_gltf_loader.cpp)

find_library( # Sets the name of the path variable.
        android-lib

        # Specifies the name of the NDK library that
        # you want CMake to locate.
        android )

target_link_libraries( # set the target
        tiny_gltf

        # the library to link
        android)