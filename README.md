# electric_emu_ros2
Template repo for our future robot-specific projects.

## Installation
`git clone --recurse-submodules https://github.com/cornellev/electric-emu-brain.git`  
`cd electric_emu_ros2 && sudo ./install.sh`

## Docker
`docker build -t cev-brain-ros2 .`

## Development

#### Formatting
If you're not using the DevContainer, please make sure to install `clang-format`, ideally version 19! This will help keep the formatting of our C++ code formatted consistently. Python code is formatted by `black`. The DevContainer will install the `Black` extension for you, which ships with `black`, but otherwise you'll need to install it in VSCode or whatever other IDE you're using.

## UART SETUP ON PI
`sudo raspi-config`  
In InterfaceOptions/SerialPort disable the login shell and enable the serial port hardware.

`sudo nano /boot/config.txt`
Add the following:  
```
enable_uart=1
dtoverlay=disable-bt
```  
and save.  
  
`sudo systemctl disable serial-getty@ttyS0.service  `
  
Reboot.  

Connect to the correct GPIO pins for UART communication on the PI,
then try reading and writing data to /dev/serial0 or some other /dev/serial* device.