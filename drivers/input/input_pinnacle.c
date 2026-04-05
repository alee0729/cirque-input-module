#define DT_DRV_COMPAT cirque_pinnacle

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/logging/log.h>

#include "input_pinnacle.h"

LOG_MODULE_REGISTER(pinnacle, CONFIG_INPUT_LOG_LEVEL);

static int pinnacle_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                             const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    return config->seq_read(dev, addr, buf, len);
}
static int pinnacle_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    return config->write(dev, addr, val);
}

// Now that we are counting ZIDLEs it would be very bad to miss one.  But in my testing I see that happen (rarely - once every
// couple of days of usage).  The fact that the current irq system is edge triggered probably isn't great for this reason.
// But for now just have the touch controller emit NUM_ZIDLE_PAD extra idles
#define NUM_ZIDLE  3
#define NUM_ZIDLE_PAD 2

// Adaptive sample rate: reduce rate after this many consecutive low-movement samples
#define PINNACLE_ADAPTIVE_IDLE_THRESHOLD 10
// Position delta (in Pinnacle units) below which movement is considered "idle"
#define PINNACLE_ADAPTIVE_MOVE_THRESHOLD 5

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int pinnacle_i2c_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    return i2c_burst_read_dt(&config->bus.i2c, PINNACLE_READ | addr, buf, len);
}

static int pinnacle_i2c_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    return i2c_reg_write_byte_dt(&config->bus.i2c, PINNACLE_WRITE | addr, val);
}

#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int pinnacle_spi_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    uint8_t tx_buffer[len + 3], rx_dummy[3];
    tx_buffer[0] = PINNACLE_READ | addr;
    memset(&tx_buffer[1], PINNACLE_AUTOINC, len + 2);

    const struct spi_buf tx_buf[2] = {
        {
            .buf = tx_buffer,
            .len = len + 3,
        },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = 1,
    };
    struct spi_buf rx_buf[2] = {
        {
            .buf = rx_dummy,
            .len = 3,
        },
        {
            .buf = buf,
            .len = len,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = 2,
    };
    int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    return ret;
}

static int pinnacle_spi_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    uint8_t tx_buffer[2] = {PINNACLE_WRITE | addr, val};
    uint8_t rx_buffer[2];

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = 2,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    const struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = 2,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    const int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    if (ret < 0) {
        LOG_ERR("spi ret: %d", ret);
    }

    if (rx_buffer[1] != PINNACLE_FILLER) {
        LOG_ERR("bad ret val %d - %d", rx_buffer[0], rx_buffer[1]);
        return -EIO;
    }

    k_usleep(50);

    return ret;
}
#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int set_int(const struct device *dev, const bool en) {
    const struct pinnacle_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->dr,
                                              en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }

    return ret;
}

static int pinnacle_clear_status(const struct device *dev) {
    int ret = pinnacle_write(dev, PINNACLE_STATUS1, 0);
    if (ret < 0) {
        LOG_ERR("Failed to clear STATUS1 register: %d", ret);
    }

    return ret;
}

static int pinnacle_era_read(const struct device *dev, const uint16_t addr, uint8_t *val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        ret = -EIO;
        goto out;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        ret = -EIO;
        goto out;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_READ);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        ret = -EIO;
        goto out;
    }

    uint8_t control_val;
    int poll_count = 0;
    do {
        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            ret = -EIO;
            goto out;
        }
        if (++poll_count > 100) {
            LOG_ERR("ERA read timed out (control=0x%02x)", control_val);
            ret = -ETIMEDOUT;
            goto out;
        }
        k_usleep(50);
    } while (control_val != 0x00);

    ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_VALUE, val, 1);
    if (ret < 0) {
        LOG_ERR("Failed to read ERA value (%d)", ret);
        ret = -EIO;
    }

out:
    pinnacle_clear_status(dev);
    set_int(dev, true);
    return ret;
}

static int pinnacle_era_write(const struct device *dev, const uint16_t addr, uint8_t val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_VALUE, val);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA value (%d)", ret);
        ret = -EIO;
        goto out;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        ret = -EIO;
        goto out;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        ret = -EIO;
        goto out;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        ret = -EIO;
        goto out;
    }

    uint8_t control_val;
    int poll_count = 0;
    do {
        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            ret = -EIO;
            goto out;
        }
        if (++poll_count > 100) {
            LOG_ERR("ERA write timed out (control=0x%02x)", control_val);
            ret = -ETIMEDOUT;
            goto out;
        }
        k_usleep(50);
    } while (control_val != 0x00);

out:
    pinnacle_clear_status(dev);
    set_int(dev, true);
    return ret;
}

static void pinnacle_send_rel(const struct device *dev, int8_t dx, int8_t dy) {
    const struct pinnacle_config *config = dev->config;
    struct pinnacle_data *data = dev->data;

    bool must_send = false;

    uint8_t btn = data->last_btn;
    if (!config->no_taps && (btn || data->btn_cache)) {
        for (int i = 0; i < 3; i++) {
            uint8_t btn_val = btn & BIT(i);
            if (btn_val != (data->btn_cache & BIT(i))) {
                input_report_key(dev, INPUT_BTN_0 + i, btn_val ? 1 : 0, false, K_FOREVER);
                must_send = true;
            }
        }
    }

    data->btn_cache = btn;
    bool is_touching = (data->last_z > 0);
    bool touch_changed = false;
    if(is_touching) {
        must_send = true;

        if(data->num_z_idle > 0) { // we just recently had z idles
            touch_changed = true;
            data->num_z_idle = 0;
            dx = 0;
            dy = 0; // starting a new press, must reset deltas
        }
    } else {
        data->num_z_idle++;
        if(data->num_z_idle == NUM_ZIDLE) {
            touch_changed = true;
        }
        dx = 0;
        dy = 0;
    }

    LOG_DBG("Rel move: touch_changed=%d z=%d dx=%d dy=%d", touch_changed, data->last_z, dx, dy);

    if(touch_changed)
    {
        // Finalize the input event only if we have something to report
        input_report_key(dev, INPUT_BTN_TOUCH, is_touching ? 1 : 0, false, K_FOREVER);
        must_send = true;
    }

    if(must_send) {
        input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER); 
    }
}

static void pinnacle_send_abs(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    struct pinnacle_data *data = dev->data;
    int16_t x = data->last_x;
    int16_t y = data->last_y;
    int8_t z = data->last_z;

    uint8_t btn = data->last_btn;
    if (!config->no_taps && (btn || data->btn_cache)) {
        for (int i = 0; i < 3; i++) {
            uint8_t btn_val = btn & BIT(i);
            if (btn_val != (data->btn_cache & BIT(i))) {
                input_report_key(dev, INPUT_BTN_0 + i, btn_val ? 1 : 0, false, K_FOREVER);
            }
        }
    }

    data->btn_cache = btn;

    if (z > 0) {
        if (x < config->absolute_mode_clamp_min_x) {
            x = config->absolute_mode_clamp_min_x;
        } else if (x > config->absolute_mode_clamp_max_x) {
            x = config->absolute_mode_clamp_max_x;
        }
        if (y < config->absolute_mode_clamp_min_y) {
            y = config->absolute_mode_clamp_min_y;
        } else if (y > config->absolute_mode_clamp_max_y) {
            y = config->absolute_mode_clamp_max_y;
        }

        // scale to be in the configured interval
        x = ((x - config->absolute_mode_clamp_min_x) * config->absolute_mode_scale_to_width) / (config->absolute_mode_clamp_max_x - config->absolute_mode_clamp_min_x);
        y = ((y - config->absolute_mode_clamp_min_y) * config->absolute_mode_scale_to_height) / (config->absolute_mode_clamp_max_y - config->absolute_mode_clamp_min_y);

        input_report_abs(dev, INPUT_ABS_X, x, false, K_FOREVER);
        input_report_abs(dev, INPUT_ABS_Y, y, true, K_FOREVER);
    }

    return;
}

static int pinnacle_read_abs(const struct device *dev) {
    uint8_t packet[6];
    int ret;
    ret = pinnacle_seq_read(dev, PINNACLE_2_2_PACKET0, packet, 6);
    if (ret < 0) {
        LOG_ERR("read packet: %d", ret);
        return ret;
    }
    struct pinnacle_data *data = dev->data;
    // TODO: Enable SW3-SW5 as well
    data->last_btn = packet[0] &
                  (PINNACLE_PACKET0_BTN_PRIM | PINNACLE_PACKET0_BTN_SEC | PINNACLE_PACKET0_BTN_AUX);
    uint8_t x_low = packet[2];
    uint8_t y_low = packet[3];
    uint8_t xy_high = packet[4];
    data->last_x = ((xy_high & 0x0F) << 8) | x_low;
    data->last_y = ((xy_high & 0xF0) << 4) | y_low;
    data->last_z = (uint8_t)(packet[5] & 0x1F);

    LOG_DBG("button: %d, x: %d y: %d z: %d", data->last_btn, data->last_x, data->last_y, data->last_z);
    return 0;
}

static void pinnacle_report_data_abs(const struct device *dev) {
    int ret = pinnacle_read_abs(dev);
    if (ret == 0) {
        pinnacle_send_abs(dev);
    }
}

static void pinnacle_report_data_abs_rel(const struct device *dev) {
    struct pinnacle_data *data = dev->data;
    int16_t old_x = data->last_x;
    int16_t old_y = data->last_y;

    int ret = pinnacle_read_abs(dev);

    if (ret == 0) {
        int16_t dx = data->last_x - old_x;
        int16_t dy = data->last_y - old_y;
        const struct pinnacle_config *config = dev->config;

        dx /= config->abs_rel_divisor;
        dy /= config->abs_rel_divisor;

        pinnacle_send_rel(dev, (int8_t) dx, (int8_t) dy);
    }
}

static void pinnacle_report_data_rel(const struct device *dev) {
    uint8_t packet[3];
    int ret;
    ret = pinnacle_seq_read(dev, PINNACLE_2_2_PACKET0, packet, 3);
    if (ret < 0) {
        LOG_ERR("read packet: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 3, "Pinnacle Packets");

    struct pinnacle_data *data = dev->data;
    data->last_btn = packet[0] &
                  (PINNACLE_PACKET0_BTN_PRIM | PINNACLE_PACKET0_BTN_SEC | PINNACLE_PACKET0_BTN_AUX);

    int8_t dx = (int8_t)packet[1];
    int8_t dy = (int8_t)packet[2];

    if (packet[0] & PINNACLE_PACKET0_X_SIGN) {
        WRITE_BIT(dx, 7, 1);
    }
    if (packet[0] & PINNACLE_PACKET0_Y_SIGN) {
        WRITE_BIT(dy, 7, 1);
    }

    // always claim touch changed
    data->last_z = 1;
    pinnacle_send_rel(dev, (int8_t) dx, (int8_t) dy);
}

static void pinnacle_work_cb(struct k_work *work) {
    struct pinnacle_data *data = CONTAINER_OF(work, struct pinnacle_data, work);
    const struct device *dev = data->dev;
    const struct pinnacle_config *config = dev->config;

    if (config->absolute_mode) {
        pinnacle_report_data_abs(dev);
    } else if (config->abs_rel_divisor) {
        pinnacle_report_data_abs_rel(dev);
    } else {
        pinnacle_report_data_rel(dev);
    }

    // Always clear status and re-enable interrupt, regardless of whether the
    // report functions above found valid data. Previously this was done inside
    // pinnacle_send_abs/pinnacle_send_rel, but early-return paths (no SW_DR
    // flag, 0xFF packet, read error) would skip re-enabling the interrupt,
    // permanently disabling data-ready notifications.
    pinnacle_clear_status(dev);

    set_int(dev, true);

    // Adaptive sample rate: only meaningful in modes that track absolute position
    if (config->adaptive_sample_rate && (config->absolute_mode || config->abs_rel_divisor)) {
        int16_t dx = data->last_x - data->prev_x;
        int16_t dy = data->last_y - data->prev_y;
        data->prev_x = data->last_x;
        data->prev_y = data->last_y;

        if (dx > -PINNACLE_ADAPTIVE_MOVE_THRESHOLD && dx < PINNACLE_ADAPTIVE_MOVE_THRESHOLD &&
            dy > -PINNACLE_ADAPTIVE_MOVE_THRESHOLD && dy < PINNACLE_ADAPTIVE_MOVE_THRESHOLD) {
            if (data->activity_counter < PINNACLE_ADAPTIVE_IDLE_THRESHOLD) {
                data->activity_counter++;
            }
            if (data->activity_counter >= PINNACLE_ADAPTIVE_IDLE_THRESHOLD && !data->at_low_rate) {
                pinnacle_write(dev, PINNACLE_SAMPLE, config->low_sample_rate);
                data->at_low_rate = true;
            }
        } else {
            data->activity_counter = 0;
            if (data->at_low_rate) {
                pinnacle_write(dev, PINNACLE_SAMPLE, config->high_sample_rate);
                data->at_low_rate = false;
            }
        }
    }
}

static void pinnacle_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct pinnacle_data *data = CONTAINER_OF(cb, struct pinnacle_data, gpio_cb);

    LOG_DBG("HW DR asserted");
    k_work_submit(&data->work);
}

static int pinnacle_adc_sensitivity_reg_value(enum pinnacle_sensitivity sensitivity) {
    switch (sensitivity) {
    case PINNACLE_SENSITIVITY_1X:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    case PINNACLE_SENSITIVITY_2X:
        return PINNACLE_TRACKING_ADC_CONFIG_2X;
    case PINNACLE_SENSITIVITY_3X:
        return PINNACLE_TRACKING_ADC_CONFIG_3X;
    case PINNACLE_SENSITIVITY_4X:
        return PINNACLE_TRACKING_ADC_CONFIG_4X;
    default:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    }
}

static int pinnacle_tune_edge_sensitivity(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    int ret;

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_X_AXIS_WIDE_Z_MIN, config->x_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set X-Axis Min-Z %d", ret);
        return ret;
    }
    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_Y_AXIS_WIDE_Z_MIN, config->y_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set Y-Axis Min-Z %d", ret);
        return ret;
    }
    return 0;
}

static int pinnacle_set_adc_tracking_sensitivity(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;

    uint8_t val;
    int ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    val &= 0x3F;
    val |= pinnacle_adc_sensitivity_reg_value(config->sensitivity);

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, val);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
    }

    return ret;
}

static int pinnacle_force_recalibrate(const struct device *dev) {
    uint8_t val;
    int ret = pinnacle_seq_read(dev, PINNACLE_CAL_CFG, &val, 1);
    if (ret < 0) {
        LOG_ERR("Failed to get cal config %d", ret);
    }

    val |= 0x01;
    ret = pinnacle_write(dev, PINNACLE_CAL_CFG, val);
    if (ret < 0) {
        LOG_ERR("Failed to force calibration %d", ret);
    }

    int poll_count = 0;
    do {
        ret = pinnacle_seq_read(dev, PINNACLE_CAL_CFG, &val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read cal config during recalibration: %d", ret);
            return ret;
        }
        if (++poll_count > 100) {
            LOG_ERR("Calibration timed out (cal_cfg=0x%02x)", val);
            return -ETIMEDOUT;
        }
        k_msleep(10);
    } while (val & 0x01);

    return ret;
}

int pinnacle_set_sleep(const struct device *dev, bool enabled) {
    uint8_t sys_cfg;
    int ret = pinnacle_seq_read(dev, PINNACLE_SYS_CFG, &sys_cfg, 1);
    if (ret < 0) {
        LOG_ERR("can't read sys config %d", ret);
        return ret;
    }

    if (((sys_cfg & PINNACLE_SYS_CFG_EN_SLEEP) != 0) == enabled) {
        return 0;
    }

    LOG_DBG("Setting sleep: %s", (enabled ? "on" : "off"));
    WRITE_BIT(sys_cfg, PINNACLE_SYS_CFG_EN_SLEEP_BIT, enabled ? 1 : 0);

    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, sys_cfg);
    if (ret < 0) {
        LOG_ERR("can't write sleep config %d", ret);
        return ret;
    }

    return ret;
}


int pinnacle_set_shutdown(const struct device *dev, bool enabled) {
    uint8_t sys_cfg;
    int ret = pinnacle_seq_read(dev, PINNACLE_SYS_CFG, &sys_cfg, 1);
    if (ret < 0) {
        LOG_ERR("can't read sys config %d", ret);
        return ret;
    }

    if (((sys_cfg & PINNACLE_SYS_CFG_SHUTDOWN) != 0) == enabled) {
        LOG_WRN("Shutdown already set to %s", (enabled ? "on" : "off"));
    }

    LOG_DBG("Setting shutdown: %s", (enabled ? "on" : "off"));

    if(enabled) {
        // Disable interrupt before entering shutdown — the DR pin may produce
        // spurious assertions while the oscillator is stopping.
        set_int(dev, false);
    }

    WRITE_BIT(sys_cfg, PINNACLE_SYS_CFG_SHUTDOWN_BIT, enabled ? 1 : 0);

    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, sys_cfg);
    if (ret < 0) {
        LOG_ERR("can't write shutdown config %d", ret);
        return ret;
    }

    if (!enabled) {
        pinnacle_clear_status(dev);
        ret = set_int(dev, true);
    }

    return ret;
}

static int pinnacle_set_feed_enable(const struct device *dev, bool enable) {
    uint8_t feedcfg;
    int ret = pinnacle_seq_read(dev, PINNACLE_FEED_CFG1, &feedcfg, 1);
    if (ret < 0) {
        return ret;
    }
    if (enable) {
        feedcfg |= PINNACLE_FEED_CFG1_EN_FEED;
    } else {
        feedcfg &= ~PINNACLE_FEED_CFG1_EN_FEED;
    }
    return pinnacle_write(dev, PINNACLE_FEED_CFG1, feedcfg);
}

// Re-programs all Pinnacle registers from config. Called during init and PM resume.
// Does not touch one-time init state (GPIO callbacks, work queue, dev pointer).
static int pinnacle_configure(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    struct pinnacle_data *data = dev->data;
    int ret;

    // Disable data feed before changing configuration registers to prevent glitches.
    // After a software reset the feed is already disabled, but this is needed
    // when pinnacle_configure() is called from the PM resume path.
    ret = pinnacle_set_feed_enable(dev, false);
    if (ret < 0) {
        LOG_ERR("Failed to disable feed for reconfiguration: %d", ret);
        return ret;
    }

    // In pure absolute mode the driver doesn't count z-idle packets for lift
    // detection, so honour the DT value (default 0) to suppress unnecessary MCU
    // wakeups. Relative and abs-rel modes need NUM_ZIDLE+pad for lift detection.
    uint8_t z_idle_count = (config->absolute_mode && !config->abs_rel_divisor)
                               ? config->idle_packets_count
                               : (NUM_ZIDLE + NUM_ZIDLE_PAD);
    ret = pinnacle_write(dev, PINNACLE_Z_IDLE, z_idle_count);
    if (ret < 0) {
        LOG_ERR("can't write Z_IDLE %d", ret);
        return ret;
    }

    ret = pinnacle_set_adc_tracking_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
        return ret;
    }

    ret = pinnacle_tune_edge_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to tune edge sensitivity %d", ret);
        return ret;
    }

    ret = pinnacle_force_recalibrate(dev);
    if (ret < 0) {
        LOG_ERR("Failed to force recalibration %d", ret);
        return ret;
    }

    ret = pinnacle_set_sleep(dev, config->sleep_en);
    if (ret < 0) {
        return ret;
    }

    ret = pinnacle_write(dev, PINNACLE_SLEEP_INTERVAL, config->sleep_interval);
    if (ret < 0) {
        LOG_ERR("Failed to write sleep interval %d", ret);
        return ret;
    }

    ret = pinnacle_write(dev, PINNACLE_SLEEP_TIMER, config->sleep_timer);
    if (ret < 0) {
        LOG_ERR("Failed to write sleep timer %d", ret);
        return ret;
    }

    // Restore full sample rate and reset adaptive rate state on every configure
    if (config->adaptive_sample_rate) {
        ret = pinnacle_write(dev, PINNACLE_SAMPLE, config->high_sample_rate);
        if (ret < 0) {
            LOG_ERR("Failed to write sample rate %d", ret);
            return ret;
        }
        data->at_low_rate = false;
        data->activity_counter = 0;
    }

    uint8_t feed_cfg2 = PINNACLE_FEED_CFG2_EN_IM | PINNACLE_FEED_CFG2_EN_BTN_SCRL;
    if (config->no_taps) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_DIS_TAP;
    }
    if (config->no_secondary_tap) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_DIS_SEC;
    }
    if (config->no_glide_extend) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_DIS_GE;
    }
    if (config->no_scroll) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_DIS_SCRL;
    }
    if (config->rotate_90) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_ROTATE_90;
    }
    ret = pinnacle_write(dev, PINNACLE_FEED_CFG2, feed_cfg2);
    if (ret < 0) {
        LOG_ERR("can't write FEED_CFG2 %d", ret);
        return ret;
    }

    uint8_t feed_cfg1 = PINNACLE_FEED_CFG1_EN_FEED;
    if (config->absolute_mode || config->abs_rel_divisor) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_ABS_MODE;
        LOG_DBG("Using absolute mode");
    } else {
        LOG_DBG("Using relative mode");
    }
    if (config->x_invert) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_INV_X;
    }
    if (config->y_invert) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_INV_Y;
    }
    ret = pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);
    if (ret < 0) {
        LOG_ERR("can't write FEED_CFG1 %d", ret);
    }
    return ret;
}

static int pinnacle_init(const struct device *dev) {
    struct pinnacle_data *data = dev->data;
    const struct pinnacle_config *config = dev->config;
    int ret;

    // If a supply GPIO is configured, ensure it drives power on at boot
    if (config->supply_gpio.port != NULL) {
        ret = gpio_pin_configure_dt(&config->supply_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure supply GPIO: %d", ret);
            return ret;
        }
    }

    uint8_t fw_id[2];
    ret = pinnacle_seq_read(dev, PINNACLE_FW_ID, fw_id, 2);
    if (ret < 0) {
        LOG_ERR("Failed to get the FW ID %d", ret);
    }

    LOG_DBG("Found device with FW ID: 0x%02x, Version: 0x%02x", fw_id[0], fw_id[1]);

    k_msleep(10);
    ret = pinnacle_write(dev, PINNACLE_STATUS1, 0); // Clear CC
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    k_usleep(50);
    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, PINNACLE_SYS_CFG_RESET);
    if (ret < 0) {
        LOG_ERR("can't reset %d", ret);
        return ret;
    }
    k_msleep(20);

    uint8_t default_sleep_interval;
    if (pinnacle_seq_read(dev, PINNACLE_SLEEP_INTERVAL, &default_sleep_interval, 1) == 0) {
        LOG_DBG("Default sleep interval: %d", default_sleep_interval);
    }

    data->dev = dev;

    pinnacle_clear_status(dev);

    gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, pinnacle_gpio_cb, BIT(config->dr.pin));
    ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    k_work_init(&data->work, pinnacle_work_cb);

    ret = pinnacle_configure(dev);
    if (ret < 0) {
        return ret;
    }

    set_int(dev, true);

#if IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
    pm_device_runtime_enable(dev);
#endif

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int pinnacle_pm_action(const struct device *dev, enum pm_device_action action) {
    const struct pinnacle_config *config = dev->config;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        pinnacle_set_feed_enable(dev, false);
        pinnacle_set_shutdown(dev, true);
        // 3.1: Cut VDD entirely if a supply GPIO is wired up (achieves true 0 µA)
        if (config->supply_gpio.port != NULL) {
            gpio_pin_set_dt(&config->supply_gpio, 0);
        }
        return 0;

    case PM_DEVICE_ACTION_RESUME: {
        int ret;

        if (config->supply_gpio.port != NULL) {
            // Power was gated — restore VDD and wait for POR
            gpio_pin_set_dt(&config->supply_gpio, 1);
            k_msleep(50);
        } else {
            // Device is in shutdown (not power-gated). Clear shutdown
            // bit directly so the oscillator starts, then wait for
            // startup before issuing the software reset below.
            ret = pinnacle_write(dev, PINNACLE_SYS_CFG, 0x00);
            if (ret < 0) {
                LOG_ERR("Failed to clear shutdown: %d", ret);
                return ret;
            }
            k_msleep(10);
        }

        // Software reset — matches init sequence (lines 822–834).
        // Restores all registers to POR defaults (also clears shutdown
        // bit as a side-effect in the power-gated path).
        ret = pinnacle_write(dev, PINNACLE_STATUS1, 0);
        if (ret < 0) {
            LOG_ERR("Resume: failed to clear STATUS1: %d", ret);
            return ret;
        }
        k_usleep(50);
        ret = pinnacle_write(dev, PINNACLE_SYS_CFG, PINNACLE_SYS_CFG_RESET);
        if (ret < 0) {
            LOG_ERR("Resume: failed to issue reset: %d", ret);
            return ret;
        }
        k_msleep(20);

        // Verify device is alive (matches init FW ID check)
        uint8_t fw_id[2];
        ret = pinnacle_seq_read(dev, PINNACLE_FW_ID, fw_id, 2);
        if (ret < 0) {
            LOG_ERR("Resume: device not responding after reset: %d", ret);
            return ret;
        }
        LOG_DBG("Resume: FW ID 0x%02x, Version 0x%02x", fw_id[0], fw_id[1]);

        pinnacle_clear_status(dev);

        ret = pinnacle_configure(dev);
        if (ret < 0) {
            LOG_ERR("Failed to configure device on resume: %d", ret);
            return ret;
        }

        // Ensure interrupts are enabled after configure, matching init.
        // pinnacle_configure() uses ERA read/write which toggle interrupts
        // internally; if any ERA op partially failed, interrupts could be
        // left disabled.
        set_int(dev, true);
        return 0;
    }

    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define PINNACLE_INST(n)                                                                           \
    static struct pinnacle_data pinnacle_data_##n;                                                 \
    static const struct pinnacle_config pinnacle_config_##n = {                                    \
        COND_CODE_1(DT_INST_ON_BUS(n, i2c),                                                        \
                    (.bus = {.i2c = I2C_DT_SPEC_INST_GET(n)}, .seq_read = pinnacle_i2c_seq_read,   \
                     .write = pinnacle_i2c_write),                                                 \
                    (.bus = {.spi = SPI_DT_SPEC_INST_GET(n,                                        \
                                                         SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |    \
                                                             SPI_TRANSFER_MSB | SPI_MODE_CPHA,     \
                                                         0)},                                      \
                     .seq_read = pinnacle_spi_seq_read, .write = pinnacle_spi_write)),             \
        .rotate_90 = DT_INST_PROP(n, rotate_90),                                                   \
        .x_invert = DT_INST_PROP(n, x_invert),                                                     \
        .y_invert = DT_INST_PROP(n, y_invert),                                                     \
        .sleep_en = DT_INST_PROP(n, sleep),                                                        \
        .no_taps = DT_INST_PROP(n, no_taps),                                                       \
        .no_secondary_tap = DT_INST_PROP(n, no_secondary_tap),                                     \
        .no_glide_extend = DT_INST_PROP(n, no_glide_extend),                                      \
        .no_scroll = DT_INST_PROP(n, no_scroll),                                                   \
        .absolute_mode = DT_INST_PROP(n, absolute_mode),                                           \
        .abs_rel_divisor = DT_INST_PROP(n, abs_rel_divisor),                                       \
        .idle_packets_count = DT_INST_PROP(n, idle_packets_count),                                 \
        .sleep_interval = DT_INST_PROP(n, sleep_interval),                                        \
        .sleep_timer = DT_INST_PROP(n, sleep_timer),                                              \
        .adaptive_sample_rate = DT_INST_PROP(n, adaptive_sample_rate),                            \
        .low_sample_rate = DT_INST_PROP(n, low_sample_rate),                                      \
        .high_sample_rate = DT_INST_PROP(n, high_sample_rate),                                    \
        .absolute_mode_scale_to_width = DT_INST_PROP(n, absolute_mode_scale_to_width),             \
        .absolute_mode_scale_to_height = DT_INST_PROP(n, absolute_mode_scale_to_height),           \
        .absolute_mode_clamp_min_x = DT_INST_PROP(n, absolute_mode_clamp_min_x),                   \
        .absolute_mode_clamp_max_x = DT_INST_PROP(n, absolute_mode_clamp_max_x),                   \
        .absolute_mode_clamp_min_y = DT_INST_PROP(n, absolute_mode_clamp_min_y),                   \
        .absolute_mode_clamp_max_y = DT_INST_PROP(n, absolute_mode_clamp_max_y),                   \
        .x_axis_z_min = DT_INST_PROP_OR(n, x_axis_z_min, 5),                                       \
        .y_axis_z_min = DT_INST_PROP_OR(n, y_axis_z_min, 4),                                       \
        .sensitivity = DT_INST_ENUM_IDX_OR(n, sensitivity, PINNACLE_SENSITIVITY_1X),               \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), dr_gpios, {}),                                   \
        .supply_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), supply_gpios, {}),                     \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, pinnacle_pm_action);                                               \
    DEVICE_DT_INST_DEFINE(n, pinnacle_init, PM_DEVICE_DT_INST_GET(n), &pinnacle_data_##n,          \
                          &pinnacle_config_##n, POST_KERNEL, CONFIG_INPUT_PINNACLE_INIT_PRIORITY,  \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(PINNACLE_INST)
