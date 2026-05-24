# Add our behavior source directly to the `app` target so it links into
# the ZMK application binary alongside zmk_keymap_layer_activate/_deactivate
# (which are static-linked into app, not exported as a separate library).
# This is the same pattern used by infused-kim/kb_zmk_ps2_mouse_trackpoint_driver.
target_sources_ifdef(CONFIG_ZMK_BEHAVIOR_MSC_TEMP_LAYER app PRIVATE
    src/behavior_msc_temp_layer.c)
