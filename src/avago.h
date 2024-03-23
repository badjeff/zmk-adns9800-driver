#ifndef ZEPHYR_INCLUDE_AVAGO_H_
#define ZEPHYR_INCLUDE_AVAGO_H_

/**
 * @file avago.h
 *
 * @brief Common header file for all optical motion sensor by AVAGO
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* device data structure */
struct avago_data {
    const struct device          *dev;

    int16_t                      x;
    int16_t                      y;
    uint32_t                     curr_cpi;

    struct gpio_callback         irq_gpio_cb; // motion pin irq callback
    struct k_work                trigger_work; // realtrigger job

    struct k_work_delayable      init_work; // the work structure for delayable init steps
    int                          async_init_step;

    bool                         ready; // whether init is finished successfully
    bool                         last_read_burst;
    int                          err; // error code during async init
};

// device config data structure
struct avago_config {
    struct gpio_dt_spec irq_gpio;
    struct spi_dt_spec bus;
    struct gpio_dt_spec cs_gpio;
    uint8_t evt_type;
    uint8_t x_input_code;
    uint8_t y_input_code;
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_AVAGO_H_ */
