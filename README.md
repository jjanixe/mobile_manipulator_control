# Scripts to control husky+UR5 integrated robot via Python

## Before running the code
Turn on UR5 BEFORE running roslaunch!! (for some reason, if roslaunch is executed first, UR5 shuts down...)
```
roslaunch roas_bringup bringup.launch
```

change the permission to access the serial port
```
# dmesg | grep tty and check if which port is connected to the gripper
sudo chown husky /dev/ttyUSB0
```

## Running the code
```
python main.py --use_keyboard
```