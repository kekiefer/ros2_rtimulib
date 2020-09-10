# ros2_rtimulib
Ros2 package for interfacing with RTIMULib sensors

---

## ROS2 Package Info

### Dependencies
* RTIMULib (c++ and python) found [here](https://github.com/mattanimation/RTIMULib2)

## Setup
for now, RTIMULib should be installed via the instructions of the readme located: `Linux/python`

* change any setttings in the `config.yaml` and if need be
* the `RTIMULib.ini` file should be fine minus the need to calibrate the magnitometer

### Calibration

See [this Link](https://github.com/RTIMULib/RTIMULib2/blob/master/Calibration.pdf) for details

using RTIMULibCal:
* run the tool
* move the device around
* save .ini file


## Usage

can use the launch file
`ros2 launch ros2_rtimulib rtimulib.launch.py`

or run a the node directly
`ros2 run ros2_rtimulib imu_node`

## Thanks

This package is heavily derived from the wonderful [ros2_berrygps package](https://github.com/mattanimation/ros2_berrygps) for the BerryGPS hat.