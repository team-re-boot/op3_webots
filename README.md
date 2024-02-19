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
git clone git@github.com:team-re-boot/op3_webots.git
```

### 3. Run
```bash
cd op3_webots
webots world/robotis_op3.wbt
```

<div align="center">
<img src="img/webots.png" width="1000">
</div>