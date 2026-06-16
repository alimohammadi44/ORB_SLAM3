# Laptop Installation Notes

These notes are for developing the ORB-SLAM3-based mono SLAM project on an Ubuntu laptop.

## 1. Clone using local project name

```bash
mkdir -p ~/slam_ws
cd ~/slam_ws
git clone https://github.com/alimohammadi44/ORB_SLAM3.git mono_slam
cd mono_slam
```

## 2. Install dependencies

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake pkg-config \
  libopencv-dev libeigen3-dev \
  libglew-dev libgl1-mesa-dev libglu1-mesa-dev \
  libboost-all-dev libssl-dev \
  python3-numpy
```

## 3. Install Pangolin

```bash
cd ~/slam_ws
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout v0.6
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
sudo make install
sudo ldconfig
```

## 4. Build mono_slam

```bash
cd ~/slam_ws/mono_slam
chmod +x build.sh
./build.sh 2>&1 | tee build_log.txt
```

If the terminal crashes or the laptop is slow, use one job:

```bash
JOBS=1 ./build.sh 2>&1 | tee build_log.txt
```

If the build succeeds, check:

```bash
chmod +x scripts/check_installation.sh
./scripts/check_installation.sh
```

## 5. Clean and rebuild

```bash
rm -rf build
rm -rf Thirdparty/DBoW2/build
rm -rf Thirdparty/g2o/build
rm -rf Thirdparty/Sophus/build
JOBS=1 ./build.sh 2>&1 | tee build_log.txt
```

## 6. Recommended validation order

1. Confirm the project builds.
2. Run EuRoC monocular.
3. Run EuRoC monocular-inertial.
4. Only then prepare Android or phone data.

## 7. Common problem: modern CMake

Newer CMake versions may reject old `cmake_minimum_required` settings used in ORB-SLAM3 third-party packages. The current `build.sh` passes `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` to reduce this problem.
