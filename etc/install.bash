#!/usr/bin/env bash
# This script installs the Polhemus tracker udev rules and firmware

sudo apt-get install libusb-1 fxload
sudo mkdir /usr/local/share/PolhemusUsb 
sudo cp usbfw/* /usr/local/share/PolhemusUsb/
sudo cp 90-Polhemus_trkr.rules /etc/udev/rules.d/
