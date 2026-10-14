# 3D Lidar Scanner mobile host

Cross-platform React Native app for iOS and Android. It replaces the ROS/PC
host during capture: the phone discovers the Nano ESP32 over UDP, streams
RPLIDAR and tilt data directly, turns each scan ray into an XYZ point, and
saves a standard ASCII `.ply` point cloud in the app's document storage.
The existing ROS post-processing and mesh-generation workflow remains
available for desktop use; mesh reconstruction and multi-scan registration
are intentionally not performed on the phone.

## Development build

This app uses native UDP sockets and Wi-Fi network joining, so it needs an
Expo development build rather than Expo Go.

Requires Node.js 20.19 or newer (Node 22 recommended; see `.nvmrc`). Ubuntu's
`apt` `nodejs` package is too old and fails with `SyntaxError: Unexpected
token '?'`. Install a current version, for example with nvm:

```sh
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
# restart the shell, then:
cd mobile
nvm install   # reads .nvmrc
node --version
```

Then:

```sh
npm install
npx expo prebuild
npx expo run:ios
# or
npx expo run:android
```

Use macOS with Xcode to build/run iOS. Android Studio and an Android SDK are
needed for Android. The app requests local network access, camera access for
optional scan photos, and Wi-Fi permission when joining the scanner access
point. Native development builds are required on both platforms.

## Scanner network setup

1. Compile/upload `firmware/tilt_stage/tilt_stage.ino` to the Nano ESP32.
   Create `firmware/tilt_stage/wifi_config.h` from the example for the initial
   home network and fallback access-point credentials.
2. On a home network, connect the phone to the same Wi-Fi as the scanner and
   use **Discover nearby scanner**.
3. Away from home, join the fallback AP from **Scanner setup**. Enter its SSID
   and password there; the fallback remains enabled when home Wi-Fi is saved.
4. To provision a home network, enter its SSID and password in the app and
   choose **Save to scanner**. The ESP32 stores these values in NVS and tries
   them on startup. The phone does not persist the Wi-Fi password.

The firmware and app use the bridge's CRC-16 UDP frames on board port `4210`
and phone port `4211`. The app uses the existing tilt text commands and
RPLIDAR A1 standard scan stream. Keep this local, untrusted networks are not
appropriate for the bridge's unauthenticated LiDAR/control protocol.

## Capture and files

The angle selector defaults to −45° and +45°. The duration slider ranges from
5 to 120 seconds and sets tilt speed proportionally. The stage firmware
limits scanning to 10°/s; durations shorter than the selected angular span
allows are raised to the physically achievable minimum. At the default 90°
span this minimum is 9 seconds.

After capture, the user can optionally take a reference photo, save the point
cloud, or discard it. Saved `.ply` files can be shared using the iOS or Android
system share sheet (including AirDrop or email). The app keeps scans and
optional photos in its private document directory. For final meshing or
registration, share the point cloud to a desktop post-processing pipeline or
cloud service.

The mobile point conversion follows `tilt_coordinator/tilt_math.py`: LiDAR
azimuth is offset by 90° and the tilt stage rotates the ray about the Y axis.
