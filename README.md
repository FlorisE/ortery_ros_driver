# A ROS 2 Driver to control Ortery turntables

_Work in progress_

This package wraps the `OTADCommand.exe` utility for controlling Ortery turntables (e.g. Ortery PhotoCapture 360) and is hence meant to be used in ROS 2 for Windows.

To use it make sure that the folder containing `OTADCommand.exe` is added to the `PATH` environment variable.

Currently implemented ROS 2 services:
* GetDeviceCount: Returns the number of devices.
* GetDeviceInfo (int id): Returns the information for device _id_ (id is enumerated from 0)

To be added:
* GetDevicesInfo: Returns a list of device information for all connected devices
* ...
