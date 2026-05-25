/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Sensor binding: emits a wheel event AND keeps a temp layer alive on
 * EVERY raw sensor event (not just trigger crossings).
 *
 * Why: ZMK encoders use `triggers-per-rotation` to decimate raw sensor
 * ticks. With a 500ms keep-alive timer driven by the trigger callback,
 * slow continuous rotation can have triggers >500ms apart, which lets
 * the layer drop between them. Once the layer drops, subsequent
 * rotations route to the base-layer binding (often volume/page-up-down)
 * instead of our scroll behavior, and the user can't recover without
 * re-activating the layer some other way.
 *
 * Fix: hook accept_data, which the dispatcher calls per raw tick on
 * every layer (regardless of which layer is active for trigger). On
 * any non-zero raw value, reschedule our disable timer. The trigger
 * crossing still fires the wheel event + reactivates the layer if
 * needed via process().
 */

#define DT_DRV_COMPAT zmk_behavior_msc_temp_layer

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include <zmk/virtual_key_position.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <dt-bindings/zmk/pointing.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define MAX_LAYERS ZMK_KEYMAP_LAYERS_LEN
#define MAX_SENSORS 4   /* matches sensor-rotate-common's array */

struct msc_temp_layer_config {
    uint8_t layer;
    uint16_t time_ms;
};

struct msc_temp_layer_data {
    const struct device *dev;
    struct k_work_delayable disable_work;
    /* Per-(sensor, layer) accumulator state, mirroring sensor-rotate-common. */
    struct sensor_value remainder[MAX_SENSORS][MAX_LAYERS];
    int triggers[MAX_SENSORS][MAX_LAYERS];
    /* Timestamp (ms uptime) when the layer last transitioned to active.
     * Used as a "scroll mode anchor": encoder rotations within a grace
     * window after this time will keep the layer alive (re-activating
     * it if some other mechanism, like trackball's &zip_temp_layer,
     * deactivated it). Rotations OUTSIDE the grace window won't
     * re-activate, preserving base-layer volume behavior.
     */
    int64_t layer_activated_at_ms;
};

static void disable_layer_cb(struct k_work *work) {
    struct k_work_delayable *dw = k_work_delayable_from_work(work);
    struct msc_temp_layer_data *data = CONTAINER_OF(dw, struct msc_temp_layer_data, disable_work);
    const struct msc_temp_layer_config *cfg = data->dev->config;

    zmk_keymap_layer_id_t id = zmk_keymap_layer_index_to_id(cfg->layer);
    bool was_active = zmk_keymap_layer_active(id);
    LOG_INF("[msc_layer] disable_cb fires; layer_active(%d)=%d", cfg->layer, was_active);
    if (was_active) {
        zmk_keymap_layer_deactivate(id);
    }
    /* Reset the activation anchor so a future encoder rotation on the
     * base layer doesn't see a stale "in grace window" and re-enter
     * scroll mode unprompted. */
    data->layer_activated_at_ms = 0;
}

/* Singleton pointer for the event listener to find our data. There's
 * exactly one instance of this behavior in the device tree. */
static struct msc_temp_layer_data *g_data;

static int msc_temp_layer_init(const struct device *dev) {
    struct msc_temp_layer_data *data = dev->data;
    data->dev = dev;
    k_work_init_delayable(&data->disable_work, disable_layer_cb);
    g_data = data;
    return 0;
}

/* Track when the managed layer last became active. We use the timestamp
 * (rather than a boolean) because the layer can flip on/off multiple
 * times during a scroll session (e.g. trackball-driven activation +
 * deactivation), and we want to give the user a grace window where
 * encoder rotation re-activates the layer to keep scrolling. */
static int msc_layer_state_listener(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    if (ev == NULL || g_data == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    const struct msc_temp_layer_config *cfg = g_data->dev->config;
    if (ev->layer == cfg->layer && ev->state) {
        g_data->layer_activated_at_ms = k_uptime_get();
        LOG_INF("[msc_layer] layer %d activated at %lld",
                cfg->layer, g_data->layer_activated_at_ms);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(msc_layer_state, msc_layer_state_listener);
ZMK_SUBSCRIPTION(msc_layer_state, zmk_layer_state_changed);

/*
 * accept_data: called per raw sensor event by the dispatcher, for every
 * layer (regardless of active state). Use it to:
 *   1. Reschedule the layer-disable timer on any non-zero motion.
 *   2. Accumulate ticks the same way sensor-rotate-common does, so we
 *      can decide when to fire a trigger in process().
 */
static int sensor_binding_accept_data(struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event,
                                       const struct zmk_sensor_config *sensor_config,
                                       size_t channel_data_size,
                                       const struct zmk_sensor_channel_data *channel_data) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct msc_temp_layer_data *data = dev->data;
    const struct msc_temp_layer_config *cfg = dev->config;

    if (channel_data_size == 0) {
        return 0;
    }

    int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);
    if (sensor_index >= MAX_SENSORS) {
        return -EINVAL;
    }
    if (event.layer >= MAX_LAYERS) {
        return -EINVAL;
    }

    const struct sensor_value value = channel_data[0].value;

    /* Any non-zero raw motion: extend the scroll session if we're in it.
     *
     * Gating logic:
     *   - currently_active: layer is on right now → we're definitely
     *     in scroll mode; rearm timer.
     *   - layer_activated_at_ms recent: layer was on within `time_ms`
     *     ago, but something just deactivated it (likely trackball's
     *     own timer firing). Re-activate to continue scroll session.
     *
     * On every keep-alive, update `layer_activated_at_ms = now` so the
     * grace window slides forward with continued user activity. Without
     * this, long scroll sessions exit the grace window relative to the
     * original activation timestamp.
     *
     * Cold rotations on base layer (layer never activated, anchor=0):
     * gating fails, no re-activation, encoder fires base-layer volume.
     */
    if (cfg->layer < MAX_LAYERS && (value.val1 != 0 || value.val2 != 0)) {
        zmk_keymap_layer_id_t id = zmk_keymap_layer_index_to_id(cfg->layer);
        int64_t now = k_uptime_get();
        bool currently_active = zmk_keymap_layer_active(id);

        /* If layer is active right now, refresh the activity anchor so
         * the grace window slides forward with continued rotation. This
         * is critical: trackball's deactivation timer is shorter than a
         * typical scroll session, so when trackball finally deactivates,
         * we need a recent anchor to know the user is mid-scroll. */
        if (currently_active) {
            data->layer_activated_at_ms = now;
        }

        bool recently_active =
            data->layer_activated_at_ms > 0 &&
            (now - data->layer_activated_at_ms) < cfg->time_ms;

        if (currently_active || recently_active) {
            k_work_reschedule(&data->disable_work, K_MSEC(cfg->time_ms));
            if (!currently_active) {
                LOG_INF("[msc_layer] re-activating layer %d (recent scroll activity)",
                        cfg->layer);
                zmk_keymap_layer_activate(id);
                data->layer_activated_at_ms = now;
            }
        }
    }

    /* Accumulate, mirroring sensor-rotate-common's logic. */
    int triggers;
    if (value.val1 == 0) {
        triggers = value.val2;
    } else {
        struct sensor_value remainder = data->remainder[sensor_index][event.layer];
        remainder.val1 += value.val1;
        remainder.val2 += value.val2;
        if (remainder.val2 >= 1000000 || remainder.val2 <= -1000000) {
            remainder.val1 += remainder.val2 / 1000000;
            remainder.val2 %= 1000000;
        }
        int trigger_degrees = 360 / sensor_config->triggers_per_rotation;
        triggers = remainder.val1 / trigger_degrees;
        remainder.val1 %= trigger_degrees;
        data->remainder[sensor_index][event.layer] = remainder;
    }

    data->triggers[sensor_index][event.layer] = triggers;
    return 0;
}

/*
 * process: called by the dispatcher after accept_data, with mode = TRIGGER
 * for the highest-active layer (until OPAQUE). DISCARD for lower layers.
 *
 * If TRIGGER and we have accumulated triggers, emit one wheel event per
 * trigger and ensure the layer is active. If DISCARD, clear our trigger
 * counter (matching sensor-rotate-common's behavior) and return TRANSPARENT.
 */
static int sensor_binding_process(struct zmk_behavior_binding *binding,
                                  struct zmk_behavior_binding_event event,
                                  enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct msc_temp_layer_data *data = dev->data;
    const struct msc_temp_layer_config *cfg = dev->config;

    int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);
    if (sensor_index >= MAX_SENSORS || event.layer >= MAX_LAYERS) {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->triggers[sensor_index][event.layer] = 0;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    int triggers = data->triggers[sensor_index][event.layer];
    if (triggers == 0) {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }
    data->triggers[sensor_index][event.layer] = 0;

    /* Pick the per-direction param. param1 = CW, param2 = CCW (matches
     * &inc_dec_kp / &mouse_scroll convention).
     */
    uint32_t param;
    if (triggers > 0) {
        param = binding->param1;
    } else {
        triggers = -triggers;
        param = binding->param2;
    }

    int16_t dx = MOVE_X_DECODE(param);
    int16_t dy = MOVE_Y_DECODE(param);

    LOG_INF("[msc_layer] TRIGGER count=%d dx=%d dy=%d", triggers, dx, dy);

    /* Emit one wheel event per trigger. */
    for (int i = 0; i < triggers; i++) {
        if (dy != 0) {
            input_report_rel(dev, INPUT_REL_WHEEL, dy, true, K_NO_WAIT);
        }
        if (dx != 0) {
            input_report_rel(dev, INPUT_REL_HWHEEL, dx, true, K_NO_WAIT);
        }
    }

    /* Re-activate the layer if it was deactivated by another mechanism. */
    if (cfg->layer < MAX_LAYERS) {
        zmk_keymap_layer_id_t id = zmk_keymap_layer_index_to_id(cfg->layer);
        if (!zmk_keymap_layer_active(id)) {
            LOG_INF("[msc_layer] re-activating layer %d", cfg->layer);
            zmk_keymap_layer_activate(id);
        }
        /* Belt-and-suspenders: rearm timer here too in case a trigger
         * arrived without prior accept_data (shouldn't happen, but harmless).
         */
        k_work_reschedule(&data->disable_work, K_MSEC(cfg->time_ms));
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api msc_temp_layer_driver_api = {
    .sensor_binding_accept_data = sensor_binding_accept_data,
    .sensor_binding_process = sensor_binding_process,
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
