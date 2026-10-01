# Nav2 MPPI SIGILL Fix — Xeon E5-1660 v2

## Problem

SAGE simulation started normally, but the TerraScout Nav2 container crashed when the
Nav2 MPPI controller was initialized.

The important error was:

    [terrascout1/nav2_container] exited: signal SIGILL

This also caused the rest of the SAGE project, including `fleet-router`, to shut down.

The fleet router itself was not the cause of the failure.

---

## Hardware

CPU:

    Intel(R) Xeon(R) CPU E5-1660 v2 @ 3.70GHz

Checking CPU instruction support:

    grep -m1 '^flags' /proc/cpuinfo | grep -oE '\b(avx2|avx|fma)\b'

Result:

    avx

This CPU supports AVX, but does NOT support:

- AVX2
- FMA

---

## Root Cause

The installed ROS 2 Jazzy Nav2 MPPI package was:

    ros-jazzy-nav2-mppi-controller 1.3.13

The Nav2 MPPI libraries were:

    /opt/ros/jazzy/lib/libmppi_controller.so
    /opt/ros/jazzy/lib/libmppi_critics.so

Disassembling the original libraries showed AVX2/FMA instructions such as:

    vinserti128
    vfmadd132sd
    vfmadd213ps

The Xeon E5-1660 v2 cannot execute these instructions.

This explains the:

    SIGILL

`SIGILL` means the program attempted to execute an illegal/unsupported CPU instruction.

---

## Why Nav2 Was Built This Way

The matching Navigation2 source was checked out at version:

    1.3.13

In:

    nav2_mppi_controller/CMakeLists.txt

the build checks whether the compiler supports these flags:

    check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
    check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

and then enables them:

    if(COMPILER_SUPPORTS_AVX2)
      add_compile_options(-mavx2)
    endif()

    if(COMPILER_SUPPORTS_FMA)
      add_compile_options(-mfma)
    endif()

The important issue is that this checks whether the COMPILER understands AVX2/FMA.

It does NOT check whether the CPU that will run Nav2 supports AVX2/FMA.

Therefore the resulting MPPI libraries can contain instructions that this machine
cannot execute.

---

# Fix

## 1. Get Navigation2 1.3.13

The source used for the fix was placed in:

    ~/nav2_cpu_fix/navigation2

The source version must match the installed Nav2 version:

    1.3.13

---

## 2. Modify the MPPI CMake configuration

Edit:

    nav2_mppi_controller/CMakeLists.txt

Remove/disable:

    if(COMPILER_SUPPORTS_AVX2)
      add_compile_options(-mavx2)
    endif()

    if(COMPILER_SUPPORTS_FMA)
      add_compile_options(-mfma)
    endif()

Replace them with:

    # CPU compatibility: Xeon E5-1660 v2 supports AVX, but not AVX2/FMA.
    add_compile_options(-mno-avx2 -mno-fma)

Other compiler optimization settings were left unchanged.

---

## 3. Rebuild only the MPPI controller

From:

    ~/nav2_cpu_fix/navigation2

run:

    source /opt/ros/jazzy/setup.bash

    colcon build \
      --packages-select nav2_mppi_controller \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

The build should produce:

    install/nav2_mppi_controller/lib/libmppi_controller.so
    install/nav2_mppi_controller/lib/libmppi_critics.so

---

## 4. Check the rebuilt libraries

The rebuilt libraries were checked for the AVX2/FMA instructions that appeared in
the original binaries:

    for f in \
      install/nav2_mppi_controller/lib/libmppi_controller.so \
      install/nav2_mppi_controller/lib/libmppi_critics.so
    do
      echo "===== $f ====="
      objdump -d -M intel "$f" |
        grep -Ei '\bvfmadd|\bvinserti128|\bvpbroadcast|\bvpaddq.*ymm|\bvpmul.*ymm' |
        head -30
    done

For the fixed build, nothing was printed below the two library headings.

---

# Testing With SAGE

The normal SAGE simulation image was:

    sage-sim:latest

The rebuilt libraries were copied into:

    /tmp/nav2_mppi_cpu_fix/

The temporary Dockerfile was:

    FROM sage-sim:latest

    COPY libmppi_controller.so /opt/ros/jazzy/lib/libmppi_controller.so
    COPY libmppi_critics.so /opt/ros/jazzy/lib/libmppi_critics.so

Build the test image:

    docker build \
      -t sage-sim-mppi-fix:latest \
      /tmp/nav2_mppi_cpu_fix

---

## Preserve the Original SAGE Image

Before testing, preserve the original image:

    docker tag sage-sim:latest sage-sim-original:latest

Then make the patched image temporarily become `sage-sim:latest`:

    docker tag sage-sim-mppi-fix:latest sage-sim:latest

Verify:

    docker image inspect \
      sage-sim:latest \
      sage-sim-mppi-fix:latest \
      sage-sim-original:latest \
      --format '{{.RepoTags}} -> {{.Id}}'

Then start SAGE normally:

    make up

---

# Result

Before the fix, Nav2 died while initializing MPPI:

    nav2_container exited: signal SIGILL

After rebuilding MPPI without AVX2/FMA, Nav2 successfully created the MPPI controller:

    Created internal controller for rotation shimming:
    FollowPath of type nav2_mppi_controller::MPPIController

It then successfully configured MPPI:

    Configured MPPI Controller: FollowPath

And successfully activated it:

    Activated MPPI Controller: FollowPath

Nav2 continued starting and reported:

    Managed nodes are active

The TerraScout mission server then reached:

    ready for missions

Therefore the AVX2/FMA incompatibility was the cause of the Nav2 SIGILL crash.

---

# Important

The current solution is a TEST/PROOF solution.

The fixed libraries were manually placed into a derived Docker image.

A permanent solution should build the patched `nav2_mppi_controller` as part of
the SAGE Docker build so that the fix is reproducible after:

    docker image prune
    make build

or when setting up SAGE on another machine with the same CPU limitations.

Do not rely permanently on files stored under:

    /tmp/nav2_mppi_cpu_fix

Also do not manually modify the host ROS installation under:

    /opt/ros/jazzy/

The permanent fix should be implemented through the SAGE Docker build.