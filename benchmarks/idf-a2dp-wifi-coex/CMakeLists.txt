cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(s31_a2dp_wifi_coex_benchmark)

# ESP-IDF's current Bluedroid sources trigger GCC 15's newer strncpy
# truncation diagnostic. Keep the warning visible without editing upstream.
idf_build_set_property(COMPILE_OPTIONS
    "-Wno-error=stringop-truncation" APPEND)
