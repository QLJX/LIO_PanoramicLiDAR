# LiDAR-Inertial Odometry for Self-built Panoramic LiDAR With Dynamic Re-Initialization and Enhanced Point-Plane Matching

**[Work in Progress]** This repository contains the implementation of the LIO framework tailored for our **self-built panoramic LiDAR system**.

> **📢 NOTICE:** The source code will be made publicly available upon the acceptance of our paper.
>
> If you are a reviewer, thank you for your time and interest!

---

### 1. Introduction

This project proposes a robust LiDAR-Inertial Odometry (LIO) framework specifically designed for a **self-built panoramic LiDAR** (spinning 32-line LiDAR). Unlike conventional LIO systems, our device generates an ultra-wide FOV ($360^\circ \times 270^\circ$) but suffers from **extremely low inter-frame overlap** and aggressive rotational motion.

To address these challenges, our system features:
* **Dynamic Re-Initialization:** Capable of recovering from tracking failures *on-the-fly* while the LiDAR is continuously rotating, without requiring a static phase.
* **Enhanced Point-Plane Matching (EPPM):** Exploits historical planar information via a sliding window manager to supplement local constraints in low-overlap scenarios.

<div align="center">
    <img src="doc/fig1_handheld_platform.png" width="90%" />
    <p><strong>Fig. 1.</strong> Handheld platform. (a) Self-built panoramic LiDAR system with an ultra-wide field of view (360° × 270°). (b) Schematic diagram of the spiral scanning trajectory induced by the coupling of LiDAR rotation and platform motion, resulting in extremely limited FOV overlap between consecutive frames.</p>
    <br>
    
    <img src="doc/fig2_framework.png" width="100%" />
    <p><strong>Fig. 2.</strong> Overview of the proposed framework, which consists of four parallel threads: data processing thread in yellow, dynamic re-initialization thread in red, enhanced point-plane matching thread in green, and optimization thread in blue.</p>
</div>

### 2. Key Features

- **Hardware Adaptation:** Tightly coupled LIO tailored for spinning LiDAR kinematics (coupling of motor rotation and platform motion).
- **Robust Initialization:**
    - LMI-constrained gyroscope bias calibration.
    - Linear alignment for gravity and velocity estimation under high-speed rotation.
- **Accuracy:** Outperforms SOTA methods (FAST-LIO2, POINT-LIO, etc.) in aggressive motion and unstructured environments.
- **Versatility:** Validated on **Handheld**, **UGV**, and **UAV** platforms.


### 3. Build & Run (Coming Soon)

#### Prerequisites
* Ubuntu 20.04
* ROS Noetic
* PCL >= 1.8
* Eigen >= 3.3.4

#### Compilation
```bash
cd ~/catkin_ws/src
git clone [https://github.com/YourUsername/RepoName.git](https://github.com/YourUsername/RepoName.git)
cd ..
catkin_make
source devel/setup.bash
