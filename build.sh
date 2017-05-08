#! /bin/bash

set -e
make
gzip gpio-mcp23s08.ko
cp gpio-mcp23s08.ko.gz /usr/lib/modules/`uname -r`/extramodules
depmod
