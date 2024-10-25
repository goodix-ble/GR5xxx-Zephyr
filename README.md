# Goodix GR5xxx BLE SoC Zephyr Porting

## Introduction

This repository is for adding [Zephyr](https://github.com/zephyrproject-rtos/zephyr) support for Goodix GR5xxx BLE SoC series.

Current supported zephyr version: **v3.7.0**

Current supported SoCs:
 - [ ] **GR551x**
 - [x] **GR5525**
 - [ ] **GR5526**
 - [ ] **GR533x**

## Quick Start

> Before using zephyr, please make sure all dependencies required by Zephyr is correctly installed. Refer to [Developing with Zephyr - Getting Started Guides](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) for detailed information.

```sh
# Use gr5xxx-zephyr as west manifest repo
west init -m https://github.com/goodix-ble/gr5xxx-zephyr
west update

# Add "GProgrammer" as a flash runner in order to use `west flash` to load firmware into the SoC. This is optional.
west install-gp-runner <PATH_TO_WHERE_GPROGRAMMER_INSTALLED>

# Set toolchain via environment variable
# Consider adding these 2 env into rc file
set ZEPHYR_TOOLCHAIN_VARIANT="gnuarmemb"
set GNUARMEMB_TOOLCHAIN_PATH="<PATH_TO_WHERE_GNUARMEMB_INSTALLED>"

# Build blinky example for GR5525 Starter Kit
west build -b gr5525_sk zephyr/samples/basic/blinky

# Load firmware into GR5525 Starter Kit
west flash
```
