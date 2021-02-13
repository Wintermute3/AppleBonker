all:	controlbox

controlbox: controlbox.c protocol.h
	gcc controlbox.c -o controlbox `pkg-config --libs --cflags libusb-1.0`
