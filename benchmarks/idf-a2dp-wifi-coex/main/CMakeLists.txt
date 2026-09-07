set(IDF_COEX_EXAMPLE_DIR
    "$ENV{IDF_PATH}/examples/bluetooth/bluedroid/coex/a2dp_gatts_coex/main")
set(IDF_COEX_MAIN "${IDF_COEX_EXAMPLE_DIR}/main.c")

# Reuse the official S31 Bluedroid A2DP + BLE GATT setup verbatim, but rename
# its entry point so this benchmark can add Wi-Fi and measurement tasks.
if(NOT CMAKE_SCRIPT_MODE_FILE)
    set_source_files_properties("${IDF_COEX_MAIN}"
        PROPERTIES COMPILE_DEFINITIONS "app_main=coex_bt_app_main")
endif()

idf_component_register(
    SRCS
        "coex_benchmark.c"
        "coex_trace.c"
        "bt_app_av.c"
        "${IDF_COEX_MAIN}"
    PRIV_REQUIRES
        bt
        esp_event
        esp_netif
        esp_wifi
        nvs_flash
        lwip
        bt_app_core_utils
        bredr_app_common_utils
        a2dp_sink_common_utils
        a2dp_sink_int_codec_utils
        avrcp_common_utils
        avrcp_metadata_utils
        avrcp_abs_vol_utils
    INCLUDE_DIRS "." "${IDF_COEX_EXAMPLE_DIR}")

# Observe the closed coexistence ABI without changing its policy.  The wrappers
# only count and print the first calls, then pass every argument through.
target_link_options(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=coex_enable"
    "-Wl,--wrap=coex_disable"
    "-Wl,--wrap=coex_wifi_request"
    "-Wl,--wrap=coex_wifi_release"
    "-Wl,--wrap=coex_bt_request"
    "-Wl,--wrap=coex_bt_release")
