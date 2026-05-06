# Raspberry Pi Setup

## Enable UART on a Pi 5
Edit config.txt
```bash
sudo nano /boot/firmware/config.txt
```

Find the uart enable option and edit:
```bash
enable_uart=1
dtoverlay=uart0-pi5
```

Also disable serial console:
```bash
sudo nano /boot/firmware/cmdline.txt
```

Remove anything like:
```bash
console=serial0,115200
console=ttyAMA0,115200
```
Save the file and reboot.

## Verify
```
ls -l /dev/ttyAMA0
```

You should see something like:
```bash
crw-rw---- 1 root dialout 204, 64 Mar 10 14:02 /dev/ttyAMA0
```

The current bridge runtime uses:

```text
/dev/ttyAMA0 @ 200000 baud
```

## Enable the Pi Camera Service

The Pi CSI camera runs on native Ubuntu 25.10 and is exposed to Docker through a
v4l2loopback device. Run this once from the repository root on the Raspberry Pi
host:

```bash
./ros2_ws/host_camera/install.sh
```

The installer sets up:

```text
/dev/video10
```

as the Project NUEVO virtual camera, installs the native Ubuntu camera packages
(`libcamera-tools`, `rpicam-apps-lite` or `rpicam-apps`, v4l2loopback, and
ffmpeg), and enables `nuevo-pi-camera-feed.service` so the feed starts
automatically at boot and restarts after failures.

After the ROS2 container is running, verify both host and Docker access:

```bash
./ros2_ws/host_camera/check.sh
```

Expected test images:

```text
/tmp/nuevo_camera_check/host_loopback.jpg
/tmp/nuevo_camera_check/docker_loopback.jpg
```

Useful service commands:

```bash
sudo systemctl status nuevo-pi-camera-feed
sudo systemctl restart nuevo-pi-camera-feed
sudo journalctl -u nuevo-pi-camera-feed -f
```
