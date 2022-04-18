# RollstuhlServoSteuerung
clone ethercat Master from
https://gitlab.com/etherlab.org/ethercat.git
works with stock kernel on raspberry pi 4

/usr/local/etc/ethercat.conf:
MASTER0_DEVICE="eth0"
DEVICE_MODULES="generic"
TODO add support for the rpi4 nic

systemctl start ethercat
should then load ec_master and ec_generic
and generate /dev/EtherCAT0
add the following line to /etc/udev/rules.d
KERNEL=="EtherCAT[0 -9]*", MODE="0664", GROUP="users"

