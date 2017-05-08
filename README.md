# mcp230xx
Device driver kernel module for mcp23008 &amp; mcp23017 (i²C GPIO expanders)

This is a slight modification of [gpio-mcp23s08](https://github.com/torvalds/linux/blob/master/drivers/gpio/gpio-mcp23s08.c) linux kernel module. Original source code supports both SPI and I²C but doesn't seem to behave well when both protocols are activated.
So I remove all SPI specific code and made an I²C only version.
