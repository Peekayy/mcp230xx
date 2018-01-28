
# Deprecated !
A more complete implementation was introduced in kernel source around June 2017

# mcp230xx
Device driver kernel module for mcp23008 &amp; mcp23017 (i²C GPIO expanders)

This is a slight modification of [gpio-mcp23s08](https://github.com/torvalds/linux/blob/master/drivers/gpio/gpio-mcp23s08.c) linux kernel module. Original source code supports both SPI and I²C but doesn't seem to behave well when both protocols are activated.
So I remove all SPI specific code and made an I²C only version.

# Module usage

We'll consider the mcp230xx chip is actually well hooked on an I²C bus.
This module requires i2c-dev to be loaded.
```shell
modprobe i2c-dev
```

And the module itself.
```shell
modprobe gpio-mcp23xx
```

Register the device.
```shell
echo [chip-address] > /sys/bus/i2c/devices/[i2c-bus-number]/new_device
```
Where *chip-address* is the address of the chip on the i²c bus. (0x20 if all 3 address pins are shorted to GND)
And *i2c-bus-number* is the i2c bus identifier on your machine.

This should register a new folder in /sys/class/gpio named `gpiochipX`. Where X is the base gpio address of this chip.
In this folder the following files give important information :
  * *base* : gives the starting gpio address
  * *ngpio* : gives the number of gpio pins handled by this device
  * *label* : the chip name (should contain either mcp23008 or mcp23017)

Finally we need to register the gpio device :
```shell
  echo [gpio-address] > /sys/class/gpio/epxport
```
Here *gpio-address* is **base + [0..ngpio]**
This creates a new folder in /sys/class/gpio named gpio[gpio-address].
Voilà !
