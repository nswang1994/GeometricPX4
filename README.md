# Summary
This repository contains the flight controller, and the fast finite-time stable extended state observer (FFTS-ESO) for a quadrotor subjected to complex disturbances. The implementation is based on the open-source autopilot [PX4](https://github.com/PX4/PX4-Autopilot), v1.13.2. The FFTS-ESO is implemented onto the module mc\_pos\_control and mc\_rate\_control for translational
and rotational motions, respectively.
# Citation
TBD
# Installation
This installation guide was tested with Ubuntu 20.04
Clone the repository:
    git clone -b Geometric-FFTS-ESO http://github/nswang1994/GeometricPX4
Install the PX4 [toolchain](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html):
    bash ./GeometricPX4/Tools/setup/ubuntu.sh
# Test
The test can be conducted in the SITL simulation by excuting the following command:
    make px4_sitl gazebo
# Other information
-The default parameters in the implementation is designed for 3DR-IRIS in Gazebo simulation.
-The flight experiment is conducted based on Holybro X500.
TBD
# Experiment video
TBD
