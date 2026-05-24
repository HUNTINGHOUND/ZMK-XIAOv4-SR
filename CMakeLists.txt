if(CONFIG_ZMK_BEHAVIOR_MSC_TEMP_LAYER)
  zephyr_library_named(behavior_msc_temp_layer)

  # Pull in ZMK app headers (zmk/keymap.h, zmk/behavior.h, etc.).
  if(EXISTS ${ZEPHYR_BASE}/../zmk/app/include)
    zephyr_library_include_directories(${ZEPHYR_BASE}/../zmk/app/include)
  endif()

  zephyr_library_sources(src/behavior_msc_temp_layer.c)

  if(TARGET zmk)
    zephyr_library_link_libraries(zmk)
  endif()
endif()
