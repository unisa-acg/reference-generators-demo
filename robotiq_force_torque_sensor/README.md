# robotiq_ft_sensor_hardware

This package provides a wrapper for the Robotiq Force/Torque (F/T) sensor driver.
It enables communication with the sensor hardware and real-time collection of force and torque data, publishing the sensor data to ROS2 topics for visualization.

This package uses the sensor driver available at the following [link](https://github.com/unisa-acg/rq_fts_ros2_driver).
Please refer to the corresponding [Readme](https://github.com/unisa-acg/rq_fts_ros2_driver/blob/main/README.md)
for more information.

- The [`launch`](./launch/) folder contains the [`demo_robotiq_ft_sensor.launch.py`](./launch/demo_robotiq_ft_sensor.launch.py) file that loads the real sensor plugin and starts a  `plotjuggler` node to graphically display the measured wrenches.

- The [`config`](./config/) folder contains the [`robotiq_force_torque_measurements_plot.xml`](./config/robotiq_force_torque_measurements_plot.xml), a configuration file needed for the `Plotjuggler` layout.

## Getting started

### Using the driver

In order to use the driver, you must have access to the virtual serial COM port; to do so, add your username to the `dialout` group and reboot your machine:

```bash
sudo usermod -a -G dialout $USER
sudo shutdown -r now
```

Once the software setup is complete, don't forget to set up your hardware architecture as well: connect the USB cable to your machine, attach the USB-to-RS45 converter to the sensor, and plug the converter in a socket to power it.
You should see a solid blue LED light on your sensor's base.

Finally, you can check if the sensor is available in your devices:

```bash
ls -l /dev/ | grep ttyUSB
```

By default, the device name is set to `ttyUSB0`.
If it differs, make sure to pass the output of the following command as the value for the `ftdi_id` parameter in the file [`robotiq_hardware.launch.py`](../../rq_fts_ros2_driver/robotiq_ft_sensor_hardware/launch/robotiq_hardware.launch.py).

### Compiling

Please refer [the root Readme](../README.md#getting-started) of this repo to know how to install the source dependencies needed by this package via `wstool`.
Then, you can install system dependencies by running the following command from the root of your `colcon` workspace:

```bash
rosdep update && rosdep install --from-paths src --ignore-src --rosdistro humble
```

## Launch demos

The [demo](./launch/demo_robotiq_ft_sensor.launch.py) launch file in this package starts the Robotiq force-torque sensor hardware and displays a real-time plot of the measurements captured by the sensor.

Make sure you are within the colcon workspace and, once the dependencies have been installed, to launch a demo of the real Robotiq sensor is necessary to build the package:

```bash
colcon build --packages-up-to robotiq_force_torque_sensor
source install/setup.bash
```

To launch the demo, execute the following command:

```bash
ros2 launch robotiq_force_torque_sensor demo_robotiq_ft_sensor.launch.py
```
