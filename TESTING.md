# RADAR Testing and Validation

This repo now documents validation in two buckets: hardware-tested subsystem behavior and implemented software that was not revalidated after hardware access ended.

## Hardware-Tested Subsystems

Validated on the physical prototype:
- teleoperation command path
- USB camera publishing
- pan-tilt servo control
- MAX30102 sensing and vitals publication

## Public Code Worth Inspecting

If you are reviewing engineering depth rather than only README polish, the most useful files are:
- `src/radar_vitals/radar_vitals/pulse_ox_node.py`
- `src/radar_pan_tilt/radar_pan_tilt/pan_tilt_node.py`
- `src/radar_pan_tilt/radar_pan_tilt/pan_tilt_joy_node.py`
- `src/radar_camera/radar_camera/camera_node.py`
- `gui/main.cpp`

## What Is Still Missing

The repo does not yet include:
- quantitative latency measurements
- side-by-side comparison against a reference pulse oximeter
- automated tests around the device drivers or GUI
- end-to-end demo recording of the integrated system

Those gaps are documentation and validation gaps, not reasons to overstate what was already proven.
