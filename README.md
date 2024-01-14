ADNS9800 driver implementation for ZMK with at least Zephyr 3.5

This work is based on [ufan's zmk pixart sensor drivers](https://github.com/ufan/zmk/tree/support-trackpad) and [inorichi's zmk-pmw3610-driver](https://github.com/inorichi/zmk-pmw3610-driver).

## Installation

Only GitHub actions builds are covered here. Local builds are different for each user, therefore it's not possible to cover all cases.

Include this project on your ZMK's west manifest in `config/west.yml`:

```yml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/petejohanson
    - name: badjeff
      url-base: https://github.com/badjeff
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: feat/pointers-move-scroll
      import: app/west.yml
    - name: zmk-adns9800-driver
      remote: badjeff
      revision: main
  self:
    path: config
```

Then, edit your `build.yml` to look like this:

```yml
on: [workflow_dispatch]

jobs:
  build:
    uses: petejohanson/zmk/.github/workflows/build-user-config.yml@core/zephyr-3.5-update
```

Now, update your `board.overlay` adding the necessary bits (update the pins for your board accordingly):

```dts
&pinctrl {
    spi0_default: spi0_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                <NRF_PSEL(SPIM_MISO, 0, 17)>;
        };
    };

    spi0_sleep: spi0_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                <NRF_PSEL(SPIM_MISO, 0, 17)>;
            low-power-enable;
        };
    };
};

&spi0 {
    status = "okay";
    compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi0_default>;
    pinctrl-1 = <&spi0_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 20 GPIO_ACTIVE_LOW>;

    trackball: trackball@0 {
        status = "okay";
        compatible = "avago,adns9800";
        reg = <0>;
        spi-max-frequency = <4000000>;
        irq-gpios = <&gpio0 6 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        evt-type = <INPUT_EV_REL>;
        x-input-code = <INPUT_REL_X>;
        y-input-code = <INPUT_REL_Y>;
    };
};
```

Now enable the driver config in your `<shield>.config` file (read the Kconfig file to find out all possible options):

```conf
CONFIG_SPI=y
CONFIG_INPUT=y
CONFIG_ZMK_MOUSE=y
CONFIG_ADNS9800=y
# CONFIG_ADNS9800_LOG_LEVEL_DBG=y
# CONFIG_ADNS9800_CPI=600
# CONFIG_ADNS9800_CPI_DIVIDOR=1
# CONFIG_ADNS9800_REPORT_INTERVAL_MIN=22
# CONFIG_ADNS9800_ORIENTATION_90=y
# CONFIG_ADNS9800_ORIENTATION_180=y
# CONFIG_ADNS9800_ORIENTATION_270=y
```
