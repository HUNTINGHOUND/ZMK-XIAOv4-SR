if(CONFIG_ZMK_BEHAVIOR_MSC_TEMP_LAYER)
  zephyr_library()
  zephyr_library_sources(src/behavior_msc_temp_layer.c)
endif()
