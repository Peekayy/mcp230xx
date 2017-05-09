#!/bin/sh

function unregisterPins {
	if [[ -d gpiopins ]]; then
		for link in $(find gpiopins -type l -exec readlink {} +;); do
			echo ${link:20} > /sys/class/gpio/unexport
		done
		rm -r gpiopins
	fi
	echo "done"
}

function registerPins {
	if [[ -d gpiopins ]]; then
		rm -r gpiopins
	fi
	mkdir gpiopins
	cd gpiopins
	for dir in $(ls -d /sys/class/gpio/gpiochip*); do
		chipLabel=$(cat $dir/label)
		if [[ $chipLabel == mcp230* ]]; then
			base=$(cat $dir/base)
			ngpio=$(cat $dir/ngpio)
			i2cDev=$(cd $dir/device/driver/;ls -d *-*)
			mkdir $i2cDev
			cd $i2cDev
			for ((i=0;i<ngpio;i++)); do
				portName=$(echo -e "GP\x$((41+i/8))$((i%8))")
				gpioNumber=$((base+i))
				if [[ -e /sys/class/gpio/gpio$gpioNumber ]]; then
					echo $gpioNumber > /sys/class/gpio/unexport
				fi
				echo $gpioNumber > /sys/class/gpio/export
				ln -s /sys/class/gpio/gpio$gpioNumber $portName
				echo "$i2cDev : Added $portName->gpio$gpioNumber"
			done
		fi
	done
}

function register {
	if [[ -z $1 ]] || [[ -z $2 ]] || [[ -z $3 ]]; then
		echo "Usage : manageMcp230xx.sh register <mcp23008|mcp23017> <i2cbus> <address>"
	else
		echo $1 $3 > /sys/bus/i2c/devices/$2/new_device
	fi
}

function unregister {
	if [[ -z $1 ]] || [[ -z $2 ]]; then
		echo "Usage : manageMcp230xx.sh unregister <i2cbus> <address>"
	else
		echo $2 > /sys/bus/i2c/devices/$1/delete_device
	fi
}


function build {
	make
	gzip gpio-mcp230xx.ko
	cp gpio-mcp230xx.ko.gz /usr/lib/modules/`uname -r`/extramodules
	depmod
	make clean
}

case $1 in
	registerPins)
		registerPins
	;;
	unregisterPins)
		unregisterPins
	;;
	register)
		register $2 $3 $4
	;;
	unregister)
		unregister $2 $3
	;;
	build)
		build
	;;
	*)
		echo "Usage : manageMcp230xx.sh <registerPins|unregisterPins|build|register|unregister>"
	;;
esac

