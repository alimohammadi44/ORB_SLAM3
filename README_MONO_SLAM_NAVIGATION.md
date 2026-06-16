# Mono SLAM Navigation Development

This repository is used as the core SLAM engine for monocular and monocular-inertial navigation experiments. It is based on ORB-SLAM3, but the project goal is practical laptop development, dataset testing, and later Android/phone camera plus IMU experiments.

## Recommended local folder name

Use `mono_slam` locally even though the GitHub repository name is `ORB_SLAM3`:

```bash
mkdir -p ~/slam_ws
cd ~/slam_ws
git clone https://github.com/alimohammadi44/ORB_SLAM3.git mono_slam
cd mono_slam
```

## Development target

The preferred target for navigation is not pure monocular only. For metric navigation, use monocular-inertial SLAM:

```text
Monocular camera + IMU -> Mono-inertial ORB-SLAM3
```

Pure monocular SLAM can estimate the shape of the trajectory, but it does not provide reliable metric scale without additional information.

## Build on Ubuntu laptop

Install common dependencies:

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake pkg-config \
  libopencv-dev libeigen3-dev \
  libglew-dev libgl1-mesa-dev libglu1-mesa-dev \
  libboost-all-dev libssl-dev \
  python3-numpy
```

Build Pangolin separately before building this repository.

Then build ORB-SLAM3:

```bash
cd ~/slam_ws/mono_slam
chmod +x build.sh
./build.sh 2>&1 | tee build_log.txt
```

The build script defaults to two CPU jobs to reduce terminal crashes and RAM problems. To use more cores:

```bash
JOBS=4 ./build.sh 2>&1 | tee build_log.txt
```

## First validation sequence

Do not start with phone data. First test a known dataset:

1. Build the project successfully.
2. Run a EuRoC monocular example.
3. Run a EuRoC monocular-inertial example.
4. Only after that, prepare phone camera and IMU data.

## Project-specific work still needed

- Camera intrinsic calibration for the phone camera.
- IMU noise model and axis verification.
- Camera-to-IMU extrinsic calibration.
- Timestamp synchronization between images and IMU.
- Data conversion from Android logs to ORB-SLAM3 input format.
- Ground-truth comparison using GPS outdoors, AprilTags indoors, or measured trajectories.

## Useful local structure

```text
~/slam_ws/
  mono_slam/
  datasets/
    euroc/
    android_a52/
  results/
    euroc_test/
    android_a52_test/
```

## License note

The core ORB-SLAM3 project is GPLv3. Academic research use is straightforward with proper citation, but commercial closed-source integration needs careful license review.
