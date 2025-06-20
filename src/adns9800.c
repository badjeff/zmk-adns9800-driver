/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT avago_adns9800

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zmk/keymap.h>
#include <stdlib.h> //for abs()
#include <zephyr/sys/util.h> // for CLAMP
#include "adns9800.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adns9800, CONFIG_ADNS9800_LOG_LEVEL);

/* SROM firmware meta-data, defined in adns9800_piv.c */
extern const size_t adns9800_firmware_length;
extern const uint8_t adns9800_firmware_data[];

/* sensor initialization steps definition */
// init is done in non-blocking manner (i.e., async), a delayable work is defined for this job
// see adns9800_init and adns9800_async_init)

enum async_init_step {
    ASYNC_INIT_STEP_POWER_UP,         // power up reset
    ASYNC_INIT_STEP_FW_LOAD_START,    // clear motion registers, disable REST mode, 
                                      // enable SROM register
    ASYNC_INIT_STEP_FW_LOAD_CONTINUE, // start SROM download
    ASYNC_INIT_STEP_FW_LOAD_VERIFY,   // verify SROM pid and fid, enable REST mode
    ASYNC_INIT_STEP_CONFIGURE,        // set cpi and donwshift time (run, rest1, rest2)
    ASYNC_INIT_STEP_ENABLE_LASER,     // laser on
    ASYNC_INIT_STEP_COUNT             // end flag
};

// delay (ms) in between steps
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10 + CONFIG_ADNS9800_INIT_POWER_UP_EXTRA_DELAY_MS,
    [ASYNC_INIT_STEP_FW_LOAD_START] = 50,    // required in spec
    [ASYNC_INIT_STEP_FW_LOAD_CONTINUE] = 10, // required in spec
    [ASYNC_INIT_STEP_FW_LOAD_VERIFY] = 10,
    [ASYNC_INIT_STEP_CONFIGURE] = 1,
    [ASYNC_INIT_STEP_ENABLE_LASER] = 1,
};

static int adns9800_async_init_power_up(const struct device *dev);
static int adns9800_async_init_fw_load_start(const struct device *dev);
static int adns9800_async_init_fw_load_continue(const struct device *dev);
static int adns9800_async_init_fw_load_verify(const struct device *dev);
static int adns9800_async_init_configure(const struct device *dev);
static int adns9800_async_init_enable_laser(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = adns9800_async_init_power_up,
    [ASYNC_INIT_STEP_FW_LOAD_START] = adns9800_async_init_fw_load_start,
    [ASYNC_INIT_STEP_FW_LOAD_CONTINUE] = adns9800_async_init_fw_load_continue,
    [ASYNC_INIT_STEP_FW_LOAD_VERIFY] = adns9800_async_init_fw_load_verify,
    [ASYNC_INIT_STEP_CONFIGURE] = adns9800_async_init_configure,
    [ASYNC_INIT_STEP_ENABLE_LASER] = adns9800_async_init_enable_laser,
};

static int ___read_reg___(const struct spi_dt_spec *spi, uint8_t reg, uint8_t *buf, size_t size) {
    int err = 0;
    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    /* Write register address. */
    const struct spi_buf tx_buf = { .buf = &reg, .len = 1 };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    err = spi_write_dt(spi, &tx);
    if (err) {
        LOG_ERR("Reg read failed on SPI write");
        return err;
    }

    k_usleep(T_SRAD);

    /* Read register value. */
    struct spi_buf rx_buf = { .buf = buf, .len = size };
    const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };
    err = spi_read_dt(spi, &rx);
    if (err) {
        LOG_ERR("Reg read failed on SPI read");
        return err;
    }

    k_usleep(T_SRX);
    return err;
}

static int ___write_reg___(const struct spi_dt_spec *spi, uint8_t reg, uint8_t val) {
    int err = 0;
    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    uint8_t buf[] = { SPI_WRITE_BIT | reg, val };
    const struct spi_buf tx_buf = { .buf = buf, .len = 2 };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    err = spi_write_dt(spi, &tx);
    if (err) {
        LOG_ERR("Reg write failed on SPI write");
        return err;
    }

    k_usleep(T_SWX);
    return err;
}

static int ___cs_rel___(const struct spi_dt_spec *spi) {
    k_usleep(T_NCS_SCLK);
    return spi_release_dt(spi);
}

static int adns9800_read_reg(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err = 0;
    const struct avago_config *config = dev->config;
    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = ___read_reg___(&config->spi, reg, buf, 1);
    ___cs_rel___(&config->spi);

    return err;
}

static int adns9800_write_reg(const struct device *dev, uint8_t reg, uint8_t val) {
    int err = 0;
    const struct avago_config *config = dev->config;
    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = ___write_reg___(&config->spi, reg, val);
    ___cs_rel___(&config->spi);

    return err;
}

static int adns9800_motion_burst_read(const struct device *dev, uint8_t *buf, 
                                      size_t burst_size) {
    int err = 0;
    struct avago_data *data = dev->data;
    const struct avago_config *config = dev->config;
    __ASSERT_NO_MSG(burst_size <= ADNS9800_MAX_BURST_SIZE);

    if (!data->last_read_burst) {
        err = ___write_reg___(&config->spi, ADNS9800_REG_MOTION_BURST, 0x00);
        if (err) {
            ___cs_rel___(&config->spi);
            return err;
        }
    }

    err = ___read_reg___(&config->spi, ADNS9800_REG_MOTION_BURST, buf, burst_size);
    ___cs_rel___(&config->spi);

    k_usleep(T_BEXIT);
    data->last_read_burst = true;

    return err;
}

static int adns9800_burst_write(const struct device *dev, uint8_t reg, const uint8_t *buf, 
                                size_t size) {
    int err;
    struct avago_data *data = dev->data;
    const struct avago_config *config = dev->config;
    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    /* Write address of burst register */
    uint8_t write_buf = ADNS9800_REG_SROM_LOAD_BURST | 0x80;
    struct spi_buf tx_buf = {.buf = &write_buf, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->spi, &tx);
    if (err) {
        LOG_ERR("Burst write failed on SPI write");
        ___cs_rel___(&config->spi);
        return err;
    }

    k_usleep(T_BRSEP);

    /* Write data */
    for (size_t i = 0; i < size; i++) {
        write_buf = buf[i];
        err = spi_write_dt(&config->spi, &tx);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            break;
        }
        k_usleep(T_BRSEP);
    }

    ___cs_rel___(&config->spi);

    k_usleep(T_BEXIT);

    data->last_read_burst = false;
    return err;
}

static int set_cpi(const struct device *dev, uint32_t cpi) {
    /* Set resolution with CPI step of 50 cpi
     * 0x01: 50 cpi (minimum cpi)
     * :
     * 0xA4: 8200 cpi (maximum cpi)
     */

    if ((cpi > ADNS9800_MAX_CPI) || (cpi < ADNS9800_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    /* Convert CPI to register value */
    uint8_t value = CLAMP((cpi / ADNS9800_STP_CPI), 0x01, 0xA4);

    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    int err = adns9800_write_reg(dev, ADNS9800_REG_CONFIG1, value);
    if (err) {
        LOG_ERR("Failed to change CPI");
    }

    return err;
}

/* unit: ms */
static int set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    /* Set downshift time in ms:
     * - Run downshift time (from Run to Rest1 mode), default: 500ms
     * - Rest 1 downshift time (from Rest1 to Rest2 mode), default: 9.92 s
     * - Rest 2 downshift time (from Rest2 to Rest3 mode), default: ~10 min
     */
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case ADNS9800_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = ADNS9800_REG_RUN_DOWNSHIFT * 10 ms
         */
        maxtime = 2550; // 255 * 10;
        mintime = 10;
        break;

    case ADNS9800_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = ADNS9800_REG_RUN_DOWNSHIFT
         *                        * 16 * Rest1 rate (default 20 ms)
         */
        maxtime = 81600; // 255 * 320;
        mintime = 320;
        break;

    case ADNS9800_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = ADNS9800_REG_REST2_DOWNSHIFT
         *                        * 32 * Rest2 rate (default 100 ms)
         */
        maxtime = 816000; // 255 * 3200;
        mintime = 3200;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range", time);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    /* Convert time to register value */
    uint8_t value = time / mintime;

    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = adns9800_write_reg(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

/* set sampling rate in each mode (in ms) */
static int set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    /* Set sample time for the Rest1-Rest3 modes.
     * Values above 0x09B0 will trigger internal watchdog reset.
     */
    uint32_t maxtime = 0x9B0;
    uint32_t mintime = 1;

    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range", sample_time);
        return -EINVAL;
    }

    LOG_INF("Set sample time to %u ms", sample_time);

    /* The sample time is (reg_value + 1) * 10ms. */
    uint8_t value = (sample_time / 10) - 1;

    int err = adns9800_write_reg(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

static int set_rest_modes(const struct device *dev, uint8_t reg_addr, bool enable) {
    uint8_t value;
    int err = adns9800_read_reg(dev, reg_addr, &value);

    if (err) {
        LOG_ERR("Failed to read Config2 register");
        return err;
    }

    WRITE_BIT(value, ADNS9800_REST_EN_POS, enable);

    LOG_INF("%sable rest modes", (enable) ? ("En") : ("Dis"));
    err = adns9800_write_reg(dev, reg_addr, value);

    if (err) {
        LOG_ERR("Failed to set rest mode");
    }

    return err;
}

static int adns9800_async_init_fw_load_start(const struct device *dev) {
    int err = 0;

    /* Read from registers 0x02-0x06 regardless of the motion pin state. */
    for (uint8_t reg = 0x02; (reg <= 0x06) && !err; reg++) {
        uint8_t buf[1];
        err = adns9800_read_reg(dev, reg, buf);
    }

    if (err) {
        LOG_ERR("Cannot read from data registers");
        return err;
    }

    k_usleep(50);

    // verify product id before upload fw
    uint8_t product_id;
    err = adns9800_read_reg(dev, ADNS9800_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }
    LOG_DBG("Optical chip product ID: 0x%x", product_id);

    if (product_id != ADNS9800_PRODUCT_ID) {
        LOG_ERR("Invalid product id!");
        return -EIO;
    }

    // set the configuration_IV register in 3k firmware mode
    // bit 1 = 1 for 3k mode, other bits are reserved 
    err = adns9800_write_reg(dev, ADNS9800_REG_CONFIG4, 0x02);
    if (err) {
        LOG_ERR("Cannot set the configuration_IV register in 3k firmware mode");
        return err;
    }

    /* Write 0x1D in SROM_enable register to initialize the operation */
    err = adns9800_write_reg(dev, ADNS9800_REG_SROM_ENABLE, 0x1D);
    if (err) {
        LOG_ERR("Cannot initialize SROM");
        return err;
    }

    return err;
}

static int adns9800_async_init_fw_load_continue(const struct device *dev) {
    int err;

    LOG_INF("Uploading optical sensor firmware...");

    /* Write 0x18 to SROM_enable to start SROM download */
    err = adns9800_write_reg(dev, ADNS9800_REG_SROM_ENABLE, 0x18);
    if (err) {
        LOG_ERR("Cannot start SROM download");
        return err;
    }

    /* Write SROM file into SROM_Load_Burst register.
     * Data must start with SROM_Load_Burst address.
     */
    err = adns9800_burst_write(
        dev,
        ADNS9800_REG_SROM_LOAD_BURST,
        adns9800_firmware_data,
        adns9800_firmware_length
        );
    if (err) {
        LOG_ERR("Cannot write firmware to sensor");
    }

    LOG_INF("Uploaded optical sensor firmware.");

    return err;
}

static int adns9800_async_init_fw_load_verify(const struct device *dev) {
    int err;

    uint8_t product_id;
    err = adns9800_read_reg(dev, ADNS9800_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }
    LOG_DBG("Optical chip product ID: 0x%x", product_id);

    uint8_t fw_id;
    err = adns9800_read_reg(dev, ADNS9800_REG_SROM_ID, &fw_id);
    if (err) {
        LOG_ERR("Cannot obtain firmware id");
        return err;
    }
    LOG_DBG("Optical chip firmware ID: 0x%x", fw_id);

    // verify the ID before any other register reads or writes
    if (product_id != ADNS9800_PRODUCT_ID) {
        LOG_ERR("Invalid product id!");
        return -EIO;
    }
    if (fw_id != ADNS9800_FIRMWARE_ID) {
        LOG_ERR("Chip is not running from SROM!");
        return -EIO;
    }

    return err;
}

static void set_interrupt(const struct device *dev, const bool en) {
    const struct avago_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("Cannot set interrupt: %s", en ? "ENABLE" : "DISABLE");
    }
}

static int adns9800_async_init_power_up(const struct device *dev) {
    const struct avago_config *config = dev->config;
    const struct spi_dt_spec *spi = &config->spi;
    const struct gpio_dt_spec cs_gpio = spi->config.cs.gpio;

    // ensure that the SPI port is reset
    // Drive NCS high, and then low to reset the SPI port.
    // - ACTIVE_LOW, set false to high.
    gpio_pin_set_dt(&cs_gpio, GPIO_OUTPUT_ACTIVE);
    k_usleep(1000); // Wait for 1ms
    gpio_pin_set_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);
    k_usleep(1000); // Wait for 1ms

    return adns9800_write_reg(dev, ADNS9800_REG_POWER_UP_RESET, ADNS9800_POWERUP_CMD_RESET);
}

static int adns9800_async_init_configure(const struct device *dev) {
    int err;
    const struct avago_config *config = dev->config;

    err = set_cpi(dev, config->cpi);

    if (!err) {
        err = set_downshift_time(dev, ADNS9800_REG_RUN_DOWNSHIFT,
                                 CONFIG_ADNS9800_RUN_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = set_downshift_time(dev, ADNS9800_REG_REST1_DOWNSHIFT,
                                 CONFIG_ADNS9800_REST1_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = set_downshift_time(dev, ADNS9800_REG_REST2_DOWNSHIFT,
                                 CONFIG_ADNS9800_REST2_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = set_sample_time(dev, ADNS9800_REG_REST1_RATE,
                              CONFIG_ADNS9800_REST1_RATE_MS);
    }

    if (!err) {
        err = set_sample_time(dev, ADNS9800_REG_REST2_RATE,
                              CONFIG_ADNS9800_REST2_RATE_MS);
    }

    if (!err) {
        err = set_sample_time(dev, ADNS9800_REG_REST3_RATE,
                              CONFIG_ADNS9800_REST3_RATE_MS);
    }

    if (!err) {
        err = set_rest_modes(dev, ADNS9800_REG_CONFIG2, true);
    }

    return err;
}

static int adns9800_async_init_enable_laser(const struct device *dev) {
    int err;

    // Spec:
    // Enable laser by setting Forced_Disable bit (Bit-0) 
    // of LASER_CTRL0 register (address 0x20) to 0.

    // Actual:
    // enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
    // reading the actual value of the register is important because the real
    // default value is different from what is said in the datasheet, and if you
    // change the reserved bytes (like by writing 0x00...) it would not work.

    uint8_t laser_ctrl0;
    err = adns9800_read_reg(dev, ADNS9800_REG_LASER_CTRL0, &laser_ctrl0);
    if (err) {
        LOG_ERR("Cannot read LASER_CTRL0");
        return err;
    }

    LOG_INF("Setting LASER to %u (reg value 0x%x)", laser_ctrl0, laser_ctrl0 & 0xf0);

    err = adns9800_write_reg(dev, ADNS9800_REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );  
    if (err) {
        LOG_ERR("Cannot configure LASER_CTRL0");
        return err;
    }

    return err;
}

static void adns9800_async_init(struct k_work *work) {
    struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
    struct avago_data *data = CONTAINER_OF(work_delayable, struct avago_data, init_work);
    const struct device *dev = data->dev;

    LOG_DBG("ADNS9800 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("ADNS9800 initialization failed in step %d", data->async_init_step);
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true; // sensor is ready to work
            LOG_INF("ADNS9800 initialized");
            set_interrupt(dev, true);
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

static int adns9800_report_data(const struct device *dev) {
    struct avago_data *data = dev->data;
    const struct avago_config *config = dev->config;
    uint8_t buf[ADNS9800_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    static int64_t dx = 0;
    static int64_t dy = 0;

#if CONFIG_ADNS9800_REPORT_INTERVAL_MIN > 0
    static int64_t last_smp_time = 0;
    static int64_t last_rpt_time = 0;
    int64_t now = k_uptime_get();
#endif

    int err = adns9800_motion_burst_read(dev, buf, ADNS9800_BURST_SIZE);
    if (err) {
        return err;
    }
    // LOG_HEXDUMP_DBG(buf, ADNS9800_BURST_SIZE, "buf");

    int16_t x = ((int16_t)sys_get_le16(&buf[ADNS9800_DX_POS]));
    int16_t y = ((int16_t)sys_get_le16(&buf[ADNS9800_DY_POS]));
    LOG_DBG("x/y: %d/%d", x, y);

// #ifdef ADNS9800_SQUAL_POS
//     int8_t squal = buf[ADNS9800_SQUAL_POS];
//     LOG_DBG("motion_burst_read, X: 0x%x 0x%x, Y: 0x%x 0x%x, %d, %d, %d", 
//         buf[ADNS9800_DX_POS+1], buf[ADNS9800_DX_POS],
//         buf[ADNS9800_DY_POS+1], buf[ADNS9800_DY_POS],
//         x, y, squal);
// #else
//     LOG_DBG("motion_burst_read, X: 0x%x 0x%x, Y: 0x%x 0x%x, %d, %d", 
//         buf[ADNS9800_DX_POS+1], buf[ADNS9800_DX_POS],
//         buf[ADNS9800_DY_POS+1], buf[ADNS9800_DY_POS],
//         x, y);
// #endif

#if IS_ENABLED(CONFIG_ADNS9800_SWAP_XY)
    int16_t a = x;
    x = y;
    y = a;
#endif
#if IS_ENABLED(CONFIG_ADNS9800_INVERT_X)
    x = -x;
#endif
#if IS_ENABLED(CONFIG_ADNS9800_INVERT_Y)
    y = -y;
#endif

#if CONFIG_ADNS9800_REPORT_INTERVAL_MIN > 0
    // purge accumulated delta, if last sampled had not been reported on last report tick
    if (now - last_smp_time >= CONFIG_ADNS9800_REPORT_INTERVAL_MIN) {
        dx = 0;
        dy = 0;
    }
    last_smp_time = now;
#endif

    // accumulate delta until report in next iteration
    dx += x;
    dy += y;

#if CONFIG_ADNS9800_REPORT_INTERVAL_MIN > 0
    // strict to report inerval
    if (now - last_rpt_time < CONFIG_ADNS9800_REPORT_INTERVAL_MIN) {
        return 0;
    }
#endif

    // divide to report value
    int16_t rx = (int16_t)CLAMP(dx, INT16_MIN, INT16_MAX);
    int16_t ry = (int16_t)CLAMP(dy, INT16_MIN, INT16_MAX);
    bool have_x = rx != 0;
    bool have_y = ry != 0;

    if (have_x || have_y) {
#if CONFIG_ADNS9800_REPORT_INTERVAL_MIN > 0
        last_rpt_time = now;
#endif
        dx = 0;
        dy = 0;
        if (have_x) {
            input_report(dev, config->evt_type, config->x_input_code, rx, !have_y, K_NO_WAIT);
        }
        if (have_y) {
            input_report(dev, config->evt_type, config->y_input_code, ry, true, K_NO_WAIT);
        }
    }

    return err;
}

static void adns9800_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct avago_data *data = CONTAINER_OF(cb, struct avago_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    set_interrupt(dev, false);
    k_work_submit(&data->trigger_work);
}

static void adns9800_work_callback(struct k_work *work) {
    struct avago_data *data = CONTAINER_OF(work, struct avago_data, trigger_work);
    const struct device *dev = data->dev;
    adns9800_report_data(dev);
    set_interrupt(dev, true);
}

static int adns9800_init_irq(const struct device *dev) {
    int err;
    struct avago_data *data = dev->data;
    const struct avago_config *config = dev->config;

    // check readiness of irq gpio pin
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // init the irq pin
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    // setup and add the irq callback associated
    gpio_init_callback(&data->irq_gpio_cb, adns9800_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    return err;
}

static int adns9800_init(const struct device *dev) {
    struct avago_data *data = dev->data;
    const struct avago_config *config = dev->config;
    int err;

    if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("%s is not ready", config->spi.bus->name);
		return -ENODEV;
	}

    // check readiness of cs gpio pin and init it to inactive
    const struct gpio_dt_spec cs_gpio = config->spi.config.cs.gpio;
    if (!device_is_ready(cs_gpio.port)) {
        LOG_ERR("SPI CS device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure SPI CS GPIO");
        return err;
    }

    // init device pointer
    data->dev = dev;

    // init trigger handler work
    k_work_init(&data->trigger_work, adns9800_work_callback);

    // init irq routine
    err = adns9800_init_irq(dev);
    if (err) {
        return err;
    }

    // Setup delayable and non-blocking init jobs, including following steps:
    // 1. power reset
    // 2. clear motion registers
    // 3. srom firmware download and checking
    // 4. eable rest mode
    // 5. set cpi and downshift time and sample rate
    // The sensor is ready to work (i.e., data->ready=true after the above steps are finished)
    k_work_init_delayable(&data->init_work, adns9800_async_init);

    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

static int adns9800_attr_set(const struct device *dev, enum sensor_channel chan,
                            enum sensor_attribute attr, const struct sensor_value *val) {
    struct avago_data *data = dev->data;
    int err;

    if (unlikely(chan != SENSOR_CHAN_ALL)) {
        return -ENOTSUP;
    }

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    switch ((uint32_t)attr) {
    case ADNS9800_ATTR_CPI:
        err = set_cpi(dev, ADNS9800_SVALUE_TO_CPI(*val));
        break;

    case ADNS9800_ATTR_REST_ENABLE:
        err = set_rest_modes(dev, ADNS9800_REG_CONFIG2, ADNS9800_SVALUE_TO_BOOL(*val));
        break;

    case ADNS9800_ATTR_RUN_DOWNSHIFT_TIME:
        err = set_downshift_time(dev, ADNS9800_REG_RUN_DOWNSHIFT, ADNS9800_SVALUE_TO_TIME(*val));
        break;

    case ADNS9800_ATTR_REST1_DOWNSHIFT_TIME:
        err = set_downshift_time(dev, ADNS9800_REG_REST1_DOWNSHIFT, ADNS9800_SVALUE_TO_TIME(*val));
        break;

    case ADNS9800_ATTR_REST2_DOWNSHIFT_TIME:
        err = set_downshift_time(dev, ADNS9800_REG_REST2_DOWNSHIFT, ADNS9800_SVALUE_TO_TIME(*val));
        break;

    case ADNS9800_ATTR_REST1_SAMPLE_TIME:
        err = set_sample_time(dev, ADNS9800_REG_REST1_RATE, ADNS9800_SVALUE_TO_TIME(*val));
        break;

    case ADNS9800_ATTR_REST2_SAMPLE_TIME:
        err = set_sample_time(dev, ADNS9800_REG_REST2_RATE, ADNS9800_SVALUE_TO_TIME(*val));
        break;

    case ADNS9800_ATTR_REST3_SAMPLE_TIME:
        err = set_sample_time(dev, ADNS9800_REG_REST3_RATE, ADNS9800_SVALUE_TO_TIME(*val));
        break;

    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }

    return err;
}

static const struct sensor_driver_api adns9800_driver_api = {
    .attr_set = adns9800_attr_set,
};

#define ASND9800_SPI_MODE (SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB |    \
                           SPI_OP_MODE_MASTER | SPI_HOLD_ON_CS | SPI_LOCK_ON)

#define ASND9800_DEFINE(n)                                                                         \
    static struct avago_data data##n;                                                              \
    static const struct avago_config config##n = {                                                 \
        .spi = SPI_DT_SPEC_INST_GET(n, ASND9800_SPI_MODE, 0),                                      \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                                       \
        .evt_type = DT_PROP(DT_DRV_INST(n), evt_type),                                             \
        .x_input_code = DT_PROP(DT_DRV_INST(n), x_input_code),                                     \
        .y_input_code = DT_PROP(DT_DRV_INST(n), y_input_code),                                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, adns9800_init, NULL, &data##n, &config##n, POST_KERNEL,               \
                          CONFIG_INPUT_ADNS9800_INIT_PRIORITY, &adns9800_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ASND9800_DEFINE)
