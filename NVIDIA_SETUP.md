# NVIDIA GPU / Docker Setup for SAGE

This document describes the working NVIDIA configuration for this SAGE simulation machine.

## Hardware

### GPU

NVIDIA Quadro K2000

- VRAM: 2 GB
- NVIDIA driver: `470.256.02`
- CUDA version reported by `nvidia-smi`: `11.4`

Check with:

    nvidia-smi

Working output should show:

    NVIDIA-SMI 470.256.02
    Driver Version: 470.256.02
    GPU: Quadro K2000

---

## Operating System / Kernel

The working kernel is:

    6.8.0-142-generic

Check with:

    uname -r

The NVIDIA 470 driver works with this kernel.

A newer 7.0/HWE kernel was previously tested and caused NVIDIA DKMS/build
problems on this machine.

For this system, keep the working Ubuntu GA 6.8 kernel unless there is a
specific reason to change it.

---

# Desktop / Display Configuration

The working desktop session is:

    X11

Check with:

    echo $XDG_SESSION_TYPE

Expected:

    x11

## NVIDIA DRM Modesetting

Check:

    sudo cat /sys/module/nvidia_drm/parameters/modeset

Expected:

    N

The working configuration does NOT enable NVIDIA DRM modesetting.

---

## GRUB Configuration

Check:

    grep GRUB_CMDLINE_LINUX_DEFAULT /etc/default/grub

Expected:

    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"

Do NOT add:

    nvidia-drm.modeset=1

to the current working configuration.

---

# Important: Wayland

Wayland was tested with the Quadro K2000 and NVIDIA 470 driver.

To try to enable Wayland, NVIDIA DRM modesetting was temporarily enabled with:

    nvidia-drm.modeset=1

This caused display/login problems including:

- black screen when attempting a Wayland login
- problems returning to the normal graphical session
- green/purple graphical artifacts

The change was reverted.

The known-good configuration is:

    X11
    NVIDIA 470.256.02
    nvidia_drm modeset = N
    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"

Do not switch this machine to Wayland unless there is a very good reason and
a recovery plan is available.

---

# NVIDIA Container Toolkit

SAGE simulation requires GPU access from Docker containers.

The installed NVIDIA Container Toolkit version is:

    1.20.1

Check with:

    dpkg -l | grep -E 'nvidia-container|libnvidia-container'

Installed packages include:

    libnvidia-container-tools
    libnvidia-container1
    nvidia-container-toolkit
    nvidia-container-toolkit-base

---

# Docker Runtime

Check Docker runtimes with:

    docker info | grep -E 'Runtimes|Default Runtime'

Working configuration:

    Runtimes: runc io.containerd.runc.v2 nvidia
    Default Runtime: runc

The important part is that:

    nvidia

appears in the available Docker runtimes.

The default Docker runtime does NOT need to be NVIDIA.

SAGE's generated Docker configuration requests the NVIDIA runtime/GPU where
required.

---

# Verify SAGE Is Using the GPU

Start SAGE normally:

    make up

Then in another terminal run:

    nvidia-smi

When the simulation is running, Gazebo/SAGE processes should appear in the
NVIDIA process list.

For example, the working system showed processes similar to:

    /usr/lib/xorg/Xorg
    /usr/bin/gnome-shell
    .../render/worlds/forest.sdf
    gz sim -g -v 1

This confirms that the simulation is using the Quadro K2000.

---

# Current Known-Good Configuration

The complete working configuration is:

    Ubuntu:                   24.04 LTS
    Kernel:                   6.8.0-142-generic
    GPU:                      NVIDIA Quadro K2000
    GPU VRAM:                 2 GB
    NVIDIA Driver:            470.256.02
    NVIDIA reported CUDA:     11.4
    Desktop:                  X11
    NVIDIA DRM modeset:       N
    GRUB:                     quiet splash
    NVIDIA Container Toolkit: 1.20.1
    Docker NVIDIA runtime:    available
    Docker default runtime:   runc

---

# Troubleshooting

## Check NVIDIA driver

    nvidia-smi

If this fails, investigate the host NVIDIA driver before debugging SAGE.

---

## Check kernel

    uname -r

Known working version:

    6.8.0-142-generic

---

## Check desktop session

    echo $XDG_SESSION_TYPE

Expected:

    x11

---

## Check NVIDIA DRM modesetting

    sudo cat /sys/module/nvidia_drm/parameters/modeset

Expected:

    N

---

## Check GRUB

    grep GRUB_CMDLINE_LINUX_DEFAULT /etc/default/grub

Expected:

    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"

---

## Check Docker NVIDIA runtime

    docker info | grep -E 'Runtimes|Default Runtime'

Expected to include:

    nvidia

---

## Check NVIDIA Container Toolkit

    dpkg -l | grep -E 'nvidia-container|libnvidia-container'

The NVIDIA Container Toolkit packages should be installed.

---

# Relationship to the Nav2 Fix

The NVIDIA fix and Nav2 MPPI fix solve two different problems.

## GPU / Simulation

The Quadro K2000 requires the working NVIDIA configuration:

    Quadro K2000
        |
        +-- NVIDIA 470.256.02
        +-- Linux kernel 6.8
        +-- X11
        +-- NVIDIA DRM modeset disabled
        +-- NVIDIA Container Toolkit
        +-- Docker NVIDIA runtime
        |
        +--> SAGE / Gazebo GPU acceleration

## Nav2 / CPU

The Nav2 problem was caused by CPU instruction compatibility.

CPU:

    Intel Xeon E5-1660 v2

The CPU supports:

    AVX

but does not support:

    AVX2
    FMA

The ROS Jazzy Nav2 MPPI binaries contained AVX2/FMA instructions and caused:

    nav2_container exited: signal SIGILL

The MPPI controller was rebuilt with:

    -mno-avx2
    -mno-fma

See:

    NAV2_CPU_FIX.md

for the Nav2-specific fix.

---

# Important

This machine now has a known-good NVIDIA configuration.

Do not unnecessarily change:

- NVIDIA driver
- Linux kernel
- X11/Wayland configuration
- NVIDIA DRM modesetting
- GRUB NVIDIA parameters
- NVIDIA Container Toolkit

when debugging unrelated SAGE or ROS problems.

A failure in Nav2 does not automatically mean there is a GPU problem.

Likewise, a Gazebo/display problem does not automatically mean there is a
Nav2 problem.