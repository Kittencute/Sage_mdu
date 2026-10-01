# Nav2 MPPI CPU Fix

## Problem

TerraScout Nav2 crashed with:

```text
nav2_container exited: signal SIGILL
```

CPU:

```text
Intel Xeon E5-1660 v2
```

Check CPU instructions:

```bash
grep -m1 '^flags' /proc/cpuinfo | grep -oE '\b(avx2|avx|fma)\b'
```

Output:

```text
avx
```

The CPU supports AVX but not AVX2/FMA.

The ROS Jazzy Nav2 MPPI binaries contained AVX2/FMA instructions.

## 1. Check Nav2 Version

```bash
apt list --installed 2>/dev/null | grep nav2-mppi
```

The version fixed on this machine was:

```text
nav2_mppi_controller 1.3.13
```

## 2. Get Matching Nav2 Source

```bash
mkdir -p ~/nav2_cpu_fix
cd ~/nav2_cpu_fix

git clone --branch 1.3.13 \
  https://github.com/ros-navigation/navigation2.git

cd navigation2
```

## 3. Patch MPPI

Edit:

```text
nav2_mppi_controller/CMakeLists.txt
```

Replace:

```cmake
if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()

if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()
```

with:

```cmake
add_compile_options(-mno-avx2 -mno-fma)
```

## 4. Build MPPI

```bash
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select nav2_mppi_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The rebuilt libraries are:

```text
install/nav2_mppi_controller/lib/libmppi_controller.so
install/nav2_mppi_controller/lib/libmppi_critics.so
```

## 5. Build Patched SAGE Image

```bash
mkdir -p /tmp/nav2_mppi_cpu_fix

cp install/nav2_mppi_controller/lib/libmppi_controller.so \
  /tmp/nav2_mppi_cpu_fix/

cp install/nav2_mppi_controller/lib/libmppi_critics.so \
  /tmp/nav2_mppi_cpu_fix/
```

Create `/tmp/nav2_mppi_cpu_fix/Dockerfile`:

```dockerfile
FROM sage-sim:latest

COPY libmppi_controller.so /opt/ros/jazzy/lib/libmppi_controller.so
COPY libmppi_critics.so /opt/ros/jazzy/lib/libmppi_critics.so
```

Build:

```bash
cd /tmp/nav2_mppi_cpu_fix

docker build -t sage-sim-mppi-fix:latest .
```

Keep the original image:

```bash
docker tag sage-sim:latest sage-sim-original:latest
```

Use the patched image:

```bash
docker tag sage-sim-mppi-fix:latest sage-sim:latest
```

## 6. Test

```bash
cd ~/sage_ws_mx
make up
```

Nav2 should start without:

```text
SIGILL
```

The MPPI controller should configure and activate normally.

## Restore Original Image

If needed:

```bash
docker tag sage-sim-original:latest sage-sim:latest
```

## Important

This fixes the Nav2 MPPI CPU compatibility problem.

It is separate from the NVIDIA/GPU setup documented in:

```text
NVIDIA_SETUP.md
```