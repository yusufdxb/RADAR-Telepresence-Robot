# Validation Plan

This project no longer has continuous hardware access, so the right public posture is: preserve what was hardware-tested, document what remains software-only, and define the exact validation that would close the gap if hardware is available again.

## Hardware-Tested Behaviors To Report Clearly

| Subsystem | Publicly defensible claim |
|---|---|
| Teleop | joystick/operator input was translated into velocity commands on the prototype |
| Video | onboard USB camera published ROS 2 image topics |
| Pan-tilt | servo node drove camera orientation through ROS 2 commands |
| Vitals | MAX30102 data acquisition and vitals publishing worked on the prototype |

## Metrics Worth Adding Next

| Area | Metric | Method |
|---|---|---|
| Teleop | command latency | timestamp operator command vs motion response |
| Video | stream update rate | average FPS or frame interval over a short run |
| Vitals | refresh rate | average update interval of processed vitals topic |
| Vitals | reference agreement | compare pulse / SpO2 against a known pulse oximeter |

## Suggested Mini Dataset

| Test | Trials | Notes |
|---|---|---|
| forward / stop / rotate teleop commands | 10 each | repeat with operator at normal remote station |
| camera pan sweep | 10 | verify no missed servo commands |
| pulse reading spot checks | 10 samples | compare to reference device |
| SpO2 spot checks | 10 samples | compare to reference device |

## Evidence To Capture

- short screen recording of the Qt GUI receiving live video and vitals
- screenshot of topic list while the system is running
- one table comparing vitals output to a reference device
- one short note on failure cases such as sensor dropout or stale video

## Publication Rule

If hardware access does not return, do not fake a validation section. Keep the measurement plan public and the validated-scope statement narrow.
