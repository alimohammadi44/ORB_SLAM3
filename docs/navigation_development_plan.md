# Mono SLAM Navigation Development Plan

## Goal

Develop a repeatable monocular and monocular-inertial SLAM workflow for GPS-denied navigation experiments using ORB-SLAM3 as the core SLAM engine.

## Phase 1: Build and baseline validation

- Build ORB-SLAM3 on the laptop.
- Validate with a known dataset before using phone data.
- Save build logs and runtime logs.
- Confirm that monocular and monocular-inertial examples run.

## Phase 2: Dataset testing

Use EuRoC first because it includes synchronized camera and IMU data. This helps separate ORB-SLAM3 build problems from phone data problems.

Minimum outputs:

- Successful trajectory output.
- Runtime log.
- ATE or drift evaluation where ground truth is available.

## Phase 3: Phone data recording

For Android or phone-based experiments, record:

- Image frames.
- Image timestamps.
- Gyroscope samples.
- Accelerometer samples.
- Sensor timestamps from the same clock source if possible.

Avoid starting with real-time processing. First record data, convert it, and test offline.

## Phase 4: Calibration

Required calibration items:

- Camera intrinsic parameters.
- Distortion model.
- IMU noise parameters.
- Camera-to-IMU extrinsic transform.
- Time offset between camera and IMU.

This phase is critical. Poor calibration usually causes mono-inertial SLAM to fail even when the code is correct.

## Phase 5: Android-to-ORB-SLAM3 conversion

Create conversion scripts that generate a clean input structure:

```text
android_a52_sequence/
  images/
  timestamps.txt
  imu.csv
  settings.yaml
```

The converter should check:

- Monotonic timestamps.
- Missing frames.
- IMU sample rate.
- Image frame rate.
- Time overlap between images and IMU.

## Phase 6: Ground truth and validation

Outdoor tests can use GPS as approximate ground truth. Indoor tests should use one of:

- AprilTag markers.
- Measured path length.
- Motion capture, if available.
- Known start/end location constraints.

## Phase 7: Navigation integration

After offline SLAM is stable, add navigation outputs:

- Current pose.
- Trajectory.
- Local map points.
- Tracking state.
- Failure/reset detection.

Do not connect SLAM directly to control until tracking loss and scale failures are understood.

## Reliability checklist

- Build is repeatable from a clean clone.
- EuRoC monocular works.
- EuRoC mono-inertial works.
- Phone data format is documented.
- Calibration file is version controlled.
- Results are stored with dataset name, date, and settings file.
- Tracking failures are logged.
