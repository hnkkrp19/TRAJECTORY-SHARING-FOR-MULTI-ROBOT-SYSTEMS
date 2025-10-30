# TRAJECTORY-SHARING-FOR-MULTI-ROBOT-SYSTEMS

This repository contains code, experiments, and documentation related to our research project:  
**“Trajectory Sharing for Multi-Robot Systems”**, submitted to YSC2025 at IUH.

> 🔒 **Note**: Full paper and full code are under review and cannot be fully disclosed until publication is complete. This repository includes selected safe-to-share components for academic and portfolio purposes.

---

## 📖 Overview

The project proposes a control strategy for synchronizing the motion of two autonomous robots along a **shared reference trajectory**, despite differences in speed or hardware.

Our system combines:
- **Adaptive Pure Pursuit algorithm** – for look-ahead target point calculation.
- **PID controller** – for minimizing trajectory tracking errors.
- **RF-based wireless communication** – to transmit real-time coordinates from a central robot to a secondary robot.

The proposed system enables:
- Robust trajectory sharing
- Real-time position synchronization
- Error correction despite velocity variance

---

## 🔧 System Architecture

- Two differential-drive robots:
  - **Central robot**: Calculates and follows the reference path, sends trajectory via RF.
  - **Second robot**: Receives trajectory in real-time and adjusts its motion accordingly.
- **Sensors**: Optical encoders for feedback
- **Communication**: SPI-based RF module
- **Controller**: Embedded system (e.g., Raspberry Pi or equivalent)

---

## 🔬 Experimental Setup

Three trajectory types were tested to evaluate performance:
- 10° linear deviation
- 75° curved trajectory
- 187° reverse direction trajectory

Tracking errors were within acceptable limits, with the highest being ~0.2 units in extreme cases.

---

## 👥 Authors & Contribution

- **Nguyen Thanh Dang** – System architecture, algorithm design
- **Huynh Ngoc Kien** – Data handling, communication protocol, code implementation (co-author)
- **Nguyen Pham Ai Doan** – Hardware integration (co-author)
- **Doan Cong Qui** – Evaluation and visualization

> 📬 Contact: kienhuynhngoc.tech@gmail.com
> 📬 Contact: nguyenphamaidoan@gmail.com
