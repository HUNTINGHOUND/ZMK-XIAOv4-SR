/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Mouse-scroll behavior that also keeps a temporary keymap layer alive.
 *
 * Each invocation:
 *   1. Decodes the param into a (dx, dy) pair using the same MOVE_X/MOVE_Y
 *      encoding that &msc accepts (so SCRL_UP, SCRL_DOWN, SCRL_LEFT,
 *      SCRL_RIGHT all work).
 *   2. Emits the corresponding INPUT_REL_WHEEL or INPUT_REL_HWHEEL event.
 *   3. Activates the configured keymap layer (if not already active) and
 *      reschedules its disable timer to `time-ms` from now.
 *
 * This is a self-contained reimplementation of just enough of &msc to
 * produce a single scroll tick per invocation, paired with the temp-layer
 * activation logic. We don't share state with ZMK's built-in &msc — that
 * would mean reaching into private state of another translation unit.
 */

#define DT_DRV_COMPAT zmk_behavior_msc_temp_layer

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include <dt-bindings/zmk/pointing.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define MAX_LAYERS ZMK_KEYMAP_LAYERS_LEN

struct msc_temp_layer_config {
    uint8_t layer;
    uint16_t time_ms;
};

struct msc_temp_layer_data {
    const struct device *dev;
    struct k_work_delayable disable_work;
    bool layer_active;
};

static struct msc_temp_layer_data *active_data_for_disable;

static void disable_layer_cb(struct k_work *work) {
    struct k_work_delayable *dw = k_work_delayable_from_work(work);
    struct msc_temp_layer_data *data = CONTAINER_OF(dw, struct msc_temp_layer_data, disable_work);
    const struct msc_temp_layer_config *cfg = data->dev->config;

    if (data->layer_active) {
        LOG_DBG("Deactivating layer %d (timer expired)", cfg->layer);
        zmk_keymap_layer_deactivate(cfg->layer);
        data->layer_active = false;
    }
}

static int msc_temp_layer_init(const struct device *dev) {
    struct msc_temp_layer_data *data = dev->data;
    data->dev = dev;
    data->layer_active = false;
    k_work_init_delayable(&data->disable_work, disable_layer_cb);
    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct msc_temp_layer_data *data = dev->data;
    const struct msc_temp_layer_config *cfg = dev->config;

    int16_t dx = MOVE_X_DECODE(binding->param1);
    int16_t dy = MOVE_Y_DECODE(binding->param1);

    /* Emit one scroll event per invocation. The wrapper sensor-rotate-var
     * fires us once per encoder click, which matches one wheel tick.
     */
    if (dy != 0) {
        input_report_rel(dev, INPUT_REL_WHEEL, dy, true, K_NO_WAIT);
    }
    if (dx != 0) {
        input_report_rel(dev, INPUT_REL_HWHEEL, dx, true, K_NO_WAIT);
    }

    /* (Re-)activate the layer and arm/rearm the disable timer. */
    if (cfg->layer < MAX_LAYERS) {
        if (!data->layer_active) {
            LOG_DBG("Activating layer %d", cfg->layer);
            zmk_keymap_layer_activate(cfg->layer);
            data->layer_active = true;
        }
        k_work_reschedule(&data->disable_work, K_MSEC(cfg->time_ms));
    }

    (void)active_data_for_disable;
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    /* The sensor-rotate-var "presses" then immediately releases via tap-ms.
     * The release is a no-op for us — release timing is handled by the
     * disable work-delayable, not the binding lifecycle.
     */
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api msc_temp_layer_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define MSC_TEMP_LAYER_INST(n)                                                                     \
    static struct msc_temp_layer_data msc_temp_layer_data_##n = {};                                \
    static const struct msc_temp_layer_config msc_temp_layer_config_##n = {                        \
        .layer = DT_INST_PROP(n, layer),                                                           \
        .time_ms = DT_INST_PROP(n, time_ms),                                                       \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, msc_temp_layer_init, NULL, &msc_temp_layer_data_##n,                \
                            &msc_temp_layer_config_##n, POST_KERNEL,                               \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &msc_temp_layer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MSC_TEMP_LAYER_INST)
