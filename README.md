# op3_webots
This is [Webots](https://github.com/cyberbotics/webots) Simulator environment for RoboCup.

Some models of ball, soccer goal, etc.... in this package refer to the following URL.
https://github.com/RoboCup-Humanoid-TC/hlvs_webots

## How to Install

### 1. Install Webots
Download the latest release from following URL.

https://github.com/cyberbotics/webots/releases/download/R2023b/webots_2023b_amd64.deb

```bash
sudo dpkg -i webots_2023b_amd64.deb
```

### 2. Clone Packages and Build
```bash
mkdir -p ~/webots_ws/src && cd ~/webots_ws/src
git clone git@github.com:team-re-boot/op3_webots.git
git clone git@github.com:team-re-boot/ROBOTIS-OP3-Common.git
cd ../
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to op3_webots
```

### 3. Run
Launch Webots.
```bash
source ~/webots_ws/install/setup.bash
ros2 launch op3_webots webots_world.launch.py
```
Launch joint_state_publisher_gui if moving joints.
```bash
ros2 launch op3_webots joint_state_publisher_gui.launch.py
```

<div align="center">
  <img src="img/webots.png" width="1000">
</div>
