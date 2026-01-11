# OpenCNC HMI - Linux Embedded Platform

Qt6 HMI running on Wayland for embedded CNC control panels. Supports minimal Linux distributions optimized for fast boot and real-time performance.

## Recommended Platforms

| Platform | Boot Time | Image Size | RT Latency | Complexity |
|----------|-----------|------------|------------|------------|
| **Buildroot** | ~3s | ~100MB | <50µs | Medium |
| **Yocto** | ~5s | ~200MB | <50µs | High |
| **Arch Minimal** | ~8s | ~500MB | <100µs | Low |

## Hardware Targets

### Primary: LattePanda Iota
- Intel N100 quad-core, 8GB RAM
- 2× 2.5GbE (dedicated CNC network)
- HDMI/USB for touch panel
- 88×75mm form factor

### Alternative: Raspberry Pi 5
- BCM2712 quad-core, 4/8GB RAM
- 1× GbE
- DSI/HDMI display
- Lower cost, less compute headroom

## Quick Start: Arch Linux Minimal

Fastest path to a working system for development.

### 1. Base Install

```bash
# Boot Arch ISO, partition disk
cfdisk /dev/nvme0n1
# Create: 512MB EFI, rest ext4

mkfs.fat -F32 /dev/nvme0n1p1
mkfs.ext4 /dev/nvme0n1p2

mount /dev/nvme0n1p2 /mnt
mount --mkdir /dev/nvme0n1p1 /mnt/boot

# Minimal base install
pacstrap -K /mnt base linux-rt linux-firmware intel-ucode \
    networkmanager openssh sudo

genfstab -U /mnt >> /mnt/etc/fstab
arch-chroot /mnt
```

### 2. RT Kernel & Boot

```bash
# Install RT kernel
pacman -S linux-rt linux-rt-headers

# Bootloader
bootctl install

cat > /boot/loader/entries/arch-rt.conf << EOF
title   OpenCNC (RT Kernel)
linux   /vmlinuz-linux-rt
initrd  /intel-ucode.img
initrd  /initramfs-linux-rt.img
options root=/dev/nvme0n1p2 rw quiet isolcpus=2,3 nohz_full=2,3
EOF
```

### 3. Wayland + Qt6

```bash
# Minimal Wayland compositor
pacman -S sway foot qt6-base qt6-wayland qt6-declarative

# CNC user with RT privileges
useradd -m -G video,input cnc
echo "cnc ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/cnc

# RT limits
cat > /etc/security/limits.d/99-cnc-rt.conf << EOF
cnc    soft    rtprio    99
cnc    hard    rtprio    99
cnc    soft    memlock   unlimited
cnc    hard    memlock   unlimited
EOF
```

### 4. Auto-Start HMI

```bash
# /home/cnc/.config/sway/config
output * bg #000000 solid_color
exec /opt/opencnc/bin/opencnc-hmi

# Disable screen blanking
exec swaymsg "output * dpms on"
```

```bash
# Auto-login and start sway
mkdir -p /etc/systemd/system/getty@tty1.service.d/
cat > /etc/systemd/system/getty@tty1.service.d/autologin.conf << EOF
[Service]
ExecStart=
ExecStart=-/usr/bin/agetty --autologin cnc --noclear %I \$TERM
EOF

# .bash_profile
if [ -z "$WAYLAND_DISPLAY" ] && [ "$XDG_VTNR" -eq 1 ]; then
    exec sway
fi
```

## Buildroot Configuration

Minimal image (~100MB) with fast boot (~3s).

### External Tree Setup

```bash
# Clone OpenCNC Buildroot external
git clone https://github.com/OpenCNC/buildroot-opencnc.git
cd buildroot-opencnc

# Download Buildroot
wget https://buildroot.org/downloads/buildroot-2024.02.tar.gz
tar xf buildroot-2024.02.tar.gz

# Configure
cd buildroot-2024.02
make BR2_EXTERNAL=../external opencnc_lattepanda_defconfig

# Build (takes 30-60 min first time)
make -j$(nproc)

# Output: output/images/sdcard.img
```

### Key Buildroot Options

```makefile
# Target
BR2_x86_64=y
BR2_x86_goldmont_plus=y  # Intel N100

# Kernel
BR2_LINUX_KERNEL=y
BR2_LINUX_KERNEL_CUSTOM_VERSION=y
BR2_LINUX_KERNEL_CUSTOM_VERSION_VALUE="6.6"
BR2_LINUX_KERNEL_PATCH="$(BR2_EXTERNAL)/patches/linux-rt"

# Qt6 + Wayland
BR2_PACKAGE_QT6=y
BR2_PACKAGE_QT6BASE_GUI=y
BR2_PACKAGE_QT6BASE_WIDGETS=y
BR2_PACKAGE_QT6BASE_EGLFS=y
BR2_PACKAGE_QT6WAYLAND=y
BR2_PACKAGE_QT6DECLARATIVE=y

# Wayland compositor
BR2_PACKAGE_WESTON=y
BR2_PACKAGE_WESTON_KIOSK=y

# Network
BR2_PACKAGE_DHCPCD=y
BR2_SYSTEM_DHCP="eth0"
```

### OpenCNC Package

```makefile
# package/opencnc-hmi/opencnc-hmi.mk
OPENCNC_HMI_VERSION = main
OPENCNC_HMI_SITE = https://github.com/OpenCNC/realtime-cnc.git
OPENCNC_HMI_SITE_METHOD = git
OPENCNC_HMI_DEPENDENCIES = qt6base qt6wayland qt6declarative

define OPENCNC_HMI_BUILD_CMDS
    cd $(@D) && $(TARGET_MAKE_ENV) \
        cmake -B build -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE=$(HOST_DIR)/share/buildroot/toolchainfile.cmake \
        -DCMAKE_BUILD_TYPE=Release
    $(TARGET_MAKE_ENV) ninja -C $(@D)/build
endef

define OPENCNC_HMI_INSTALL_TARGET_CMDS
    $(INSTALL) -D -m 0755 $(@D)/build/opencnc-hmi $(TARGET_DIR)/opt/opencnc/bin/opencnc-hmi
endef

$(eval $(generic-package))
```

## Yocto Configuration

Enterprise-grade, highly customizable.

### Layer Setup

```bash
# Clone Poky and required layers
git clone -b scarthgap git://git.yoctoproject.org/poky
git clone -b scarthgap git://git.openembedded.org/meta-openembedded
git clone -b scarthgap git://git.yoctoproject.org/meta-intel
git clone https://github.com/pmodwrc/meta-qt6.git

# Initialize build
source poky/oe-init-build-env build-opencnc
```

### local.conf

```bash
MACHINE = "intel-corei7-64"
DISTRO = "poky"

# RT Kernel
PREFERRED_PROVIDER_virtual/kernel = "linux-yocto-rt"

# Qt6 Wayland
IMAGE_INSTALL:append = " \
    qtbase \
    qtwayland \
    qtdeclarative \
    weston \
    weston-init \
    opencnc-hmi \
"

# Minimal image
IMAGE_FEATURES = "splash"
DISTRO_FEATURES:remove = "x11 pulseaudio bluetooth nfs"
```

### OpenCNC Recipe

```bitbake
# meta-opencnc/recipes-apps/opencnc-hmi/opencnc-hmi_git.bb
SUMMARY = "OpenCNC HMI Application"
LICENSE = "PolyForm-Noncommercial-1.0.0"
LIC_FILES_CHKSUM = "file://LICENSE;md5=..."

SRC_URI = "git://github.com/OpenCNC/realtime-cnc.git;branch=main;protocol=https"
SRCREV = "${AUTOREV}"
S = "${WORKDIR}/git"

inherit cmake qt6-cmake

DEPENDS = "qtbase qtwayland qtdeclarative"

do_install() {
    install -d ${D}/opt/opencnc/bin
    install -m 0755 ${B}/opencnc-hmi ${D}/opt/opencnc/bin/
}

FILES:${PN} = "/opt/opencnc"
```

## RT Kernel Tuning

### Boot Parameters

```bash
# /boot/loader/entries/*.conf or GRUB
isolcpus=2,3            # Reserve cores for RT tasks
nohz_full=2,3           # Disable timer ticks on isolated cores
rcu_nocbs=2,3           # Offload RCU callbacks
processor.max_cstate=1  # Disable deep C-states
intel_idle.max_cstate=1
nosoftlockup            # Disable soft lockup detector
nowatchdog              # Disable NMI watchdog
```

### Sysctl Settings

```bash
# /etc/sysctl.d/99-rt.conf
kernel.sched_rt_runtime_us = -1
vm.swappiness = 0
kernel.nmi_watchdog = 0
kernel.hung_task_timeout_secs = 0
```

### IRQ Affinity

```bash
#!/bin/bash
# Move all IRQs to cores 0-1, leave 2-3 for RT application
for irq in /proc/irq/*/smp_affinity; do
    echo 3 > "$irq" 2>/dev/null
done

# Move kernel threads
for pid in $(pgrep -f "ksoftirqd|kworker|migration"); do
    taskset -p 0x3 $pid 2>/dev/null
done
```

## Network Configuration

### Dedicated CNC Network

```bash
# /etc/systemd/network/10-cnc.network
[Match]
Name=eth0

[Network]
Address=192.168.4.1/24

[Link]
RequiredForOnline=no
```

The ESP32-P4 controller should be configured with static IP `192.168.4.10`.

### Firewall (optional)

```bash
# Allow only CNC protocol
nft add table inet cnc
nft add chain inet cnc input { type filter hook input priority 0 \; }
nft add rule inet cnc input iif eth0 tcp dport 5000 accept
nft add rule inet cnc input iif eth0 udp dport 5001 accept
nft add rule inet cnc input iif eth0 drop
```

## Testing RT Performance

### Cyclictest

```bash
# Install rt-tests
pacman -S rt-tests  # Arch
# or build from source for Buildroot/Yocto

# Run cyclictest (should show <100µs max latency)
cyclictest -l100000 -m -Sp90 -i200 -h400 -q

# Expected output on LattePanda Iota with RT kernel:
# T: 0 Min:    1 Act:    3 Avg:    4 Max:   45
# T: 1 Min:    1 Act:    4 Avg:    4 Max:   52
# T: 2 Min:    1 Act:    2 Avg:    3 Max:   38
# T: 3 Min:    1 Act:    3 Avg:    3 Max:   41
```

### Network Latency

```bash
# From HMI to ESP32-P4
ping -c 1000 -i 0.001 192.168.4.10 | tail -1
# Expected: rtt min/avg/max = 0.1/0.3/1.2 ms
```

## Directory Structure

```
hmi/
├── linux/
│   ├── README.md           # This file
│   ├── buildroot/
│   │   ├── external/       # BR2_EXTERNAL tree
│   │   └── configs/        # defconfig files
│   ├── yocto/
│   │   └── meta-opencnc/   # Yocto layer
│   └── arch/
│       └── install.sh      # Arch setup script
├── include/
│   └── opencnc_hmi.h       # Header-only library
└── README.md               # Main HMI documentation
```
