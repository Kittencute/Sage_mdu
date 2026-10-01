# NVIDIA Setup for SAGE

Working system:

```text
Ubuntu 24.04
GPU: NVIDIA Quadro K2000
Driver: 470.256.02
Kernel: 6.8.0-142-generic
NVIDIA Container Toolkit: 1.20.1
```

## 1. Install NVIDIA 470

```bash
sudo apt install nvidia-driver-470-server
```

## 2. Install Kernel 6.8

NVIDIA 470 failed to build with the 7.0/HWE kernel.

```bash
sudo apt install linux-generic
sudo reboot
```

Verify:

```bash
uname -r
nvidia-smi
```

Working kernel:

```text
6.8.0-142-generic
```

## 3. Remove 7.0/HWE Kernel

Only do this after booting successfully into 6.8.

```bash
sudo apt remove \
  linux-image-generic-hwe-24.04 \
  linux-headers-generic-hwe-24.04 \
  linux-image-7.0.0-31-generic \
  linux-image-7.0.0-34-generic \
  linux-headers-7.0.0-31-generic \
  linux-headers-7.0.0-34-generic \
  linux-modules-7.0.0-31-generic \
  linux-modules-7.0.0-34-generic
```

## 4. Install NVIDIA Container Toolkit

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor --yes \
  -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt-get install -y nvidia-container-toolkit
```

Configure Docker:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

## 5. Verify

```bash
nvidia-smi
docker info | grep -E 'Runtimes|Default Runtime'
```

Expected Docker configuration:

```text
Runtimes: runc io.containerd.runc.v2 nvidia
Default Runtime: runc
```

## 6. Test SAGE

```bash
cd ~/sage_ws_mx
make up
```

In another terminal:

```bash
nvidia-smi
```

Gazebo/SAGE processes should appear in the NVIDIA process list.

## Nav2

The Nav2 `SIGILL` CPU problem is separate from NVIDIA.

See:

```text
NAV2_CPU_FIX.md
```