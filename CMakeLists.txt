# Add our behavior source directly to the `app` target so it links into
# the ZMK application binary alongside zmk_keymap_layer_activate/_deactivate
# (which are static-linked into app, not exported as a separate library).
#
# The behavior only makes sense on the half that owns the keymap. ZMK's
# app CMakeLists gates keymap.c (and by extension zmk_keymap_layer_*) on
# `(NOT ZMK_SPLIT) OR ZMK_SPLIT_ROLE_CENTRAL`. Compiling our behavior on
# the peripheral half fails at link time because those symbols are absent.
# Mirror that condition here.
if((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    target_sources_ifdef(CONFIG_ZMK_BEHAVIOR_MSC_TEMP_LAYER app PRIVATE
        src/behavior_msc_temp_layer.c)
endif()
