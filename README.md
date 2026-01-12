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

<p align="center">
  <img src="doc/hardware_setup.png" width="90%" />
</p>
<p align="center">
    <em>Fig. 1. Our self-built handheld panoramic LiDAR system.</em>
</p>

<br>

<p align="center">
  <img src="doc/pipeline.png" width="90%" />
</p>
<p align="center">
    <em>Fig. 2. The proposed system framework.</em>
</p>

### 2. Key Features

- **Hardware Adaptation:** Tightly coupled LIO tailored for self-built panoramic LiDAR (coupling of motor rotation and platform motion).
- **Dynamic Re-initialization:**
    - LMI-constrained gyroscope bias calibration.
    - Linear alignment for gravity and velocity estimation under high-speed rotation.
- **Enhanced Point-Plane Matching (EPPM):**
    - **Sliding Window Manager:** Propagates historical planar structures into currently unobserved regions to address the low inter-frame overlap.
    - **Two-Stage Matching:** fuses point-map constraints with point-window constraints to ensure tracking stability in narrow or degenerate environments.
- **Accuracy:** Outperforms SOTA methods (FAST-LIO2, iG-LIO, etc.) in aggressive motion and unstructured environments.
- **Versatility:** Validated on **Handheld**, **UGV**, and **UAV** platforms.

<p align="center">
  <img src="doc/platforms.png" width="90%" />
</p>
<p align="center">
    <em>Fig. 3. Experimental platforms used for evaluation.</em>
</p>

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
```bash

### 4. Experimental Results

[cite_start]We evaluated our framework on the **DUT-Panoramic** dataset against state-of-the-art LIO systems, including **LIO-SAM**, **FAST-LIO**, **FASTER-LIO**, **POINT-LIO**, and **IG-LIO**.

**Quantitative Comparison (RMSE in meters):**

Our method demonstrates superior accuracy, especially in sequences with aggressive rotation and low overlap (e.g., *Indoor_4*, *Outdoor_1*), where other methods suffer from significant drift or tracking failure ("X").

| Sequence | LIO-SAM | FAST-LIO | FASTER-LIO | POINT-LIO | IG-LIO | **Ours** |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Indoor_4** (Stairs) | 1.090 | 1.519 | 0.626 | 0.860 | 0.646 | **0.243** |
| **Indoor_6** (Agile) | 0.938 | X | 2.508 | 2.211 | 3.505 | **0.412** |
| **Outdoor_1** (Long) | 0.968 | X | 9.716 | 3.987 | 5.624 | **0.630** |
| **Outdoor_2** (Rough) | 0.953 | X | X | X | X | **0.127** |

> **Note:** "X" indicates the algorithm failed to complete the sequence.

**Dynamic Re-Initialization Performance:**
[cite_start]We also compared our re-initialization module with **D-LI-Init**[cite: 311]. As shown in the paper (Fig. 7 & 8), our method achieves faster convergence and lower estimation errors for velocity, gravity, and IMU biases under continuous rotation speeds up to 180°/s.

### 5. Acknowledgements

We sincerely appreciate the authors of the following open-source projects for their excellent contributions to the community. Our work is built upon or compared against these state-of-the-art methods:

* **[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)** (T. Shan et al.)
* **[FAST-LIO](https://github.com/hku-mars/FAST_LIO)** (W. Xu et al.)
* **[FASTER-LIO](https://github.com/gaoxiang12/faster-lio)** (C. Bai et al.)
* **[POINT-LIO](https://github.com/hku-mars/Point-LIO)** (D. He et al.)
* **[IG-LIO](https://github.com/Nanyang-Technological-University/IG-LIO)** (Z. Chen et al.)
* **[D-LI-Init](https://github.com/hku-mars/LiDAR_IMU_Init)** (Comparison Baseline for Re-initialization)

---
