# Autonomous Takeoff & Landing on a ROS2-Integrated UAV

**ROS2 ⟷ Raspberry Pi ⟷ Pixhawk ⟷ pymavlink**

## 📌 Project Overview

This repository demonstrates how to control a **Pixhawk-based UAV** through **ROS2** and **pymavlink**.

The main objective of this project is to understand and validate:

1. **Connections between ROS2, Raspberry Pi, and Pixhawk**
2. Sending and receiving MAVLink commands using **pymavlink**
3. Executing an **autonomous flight routine** (takeoff → hover → land) through a Python script

This repo acts as a learning and testing ground for bridging **ROS2 robotics middleware** with **low-level drone control via MAVLink**.

---

## 📂 Repository Structure

```
Autonomous-takeoff-Land-on-ROS2-integrated-UAV/
│
├── Test_pymavlink.py        # Script to test pymavlink connection with Pixhawk
├── auto_takeoff_land.py     # Script to perform autonomous takeoff, hover, and land
│
├── results/                
├── images/                 
└── README.md
```

---

## 🛠 Hardware & Setup

| Component             | Details                                         |
| --------------------- | ----------------------------------------------- |
| **Flight Controller** | Pixhawk 2.4.8 (PX4 firmware)                    |
| **Onboard Computer**  | Raspberry Pi 4B (Ubuntu 22.04, ROS2 Humble)     |
| **Communication**     | MAVLink protocol via serial/USB                 |

---

## 📊 Description of Tasks

### 1️⃣ Connection Test (`Test_pymavlink.py`)

* Establishes a **MAVLink connection** between the Raspberry Pi (running ROS2) and the Pixhawk.
* Confirms that the drone can:

  * Receive heartbeats
  * Respond to parameter requests
  * Acknowledge command messages

### 2️⃣ Autonomous Flight (`auto_takeoff_land.py`)

* Implements an **autonomous flight routine** using pymavlink:

  1. Arm the drone
  2. Take off to a predefined altitude
  3. Hover for a specified duration
  4. Land safely

---

## 🚀 How to Run

### 1️⃣ Install Dependencies

```bash
pip install pymavlink
pip install mavproxy
```

### 2️⃣ Connect Pixhawk to Raspberry Pi

* Via **TELEM2 → USB-UART** cable, or
* Directly via **USB connection**

Make sure the correct **serial port** (e.g., `/dev/ttyAMA0` or `/dev/ttyUSB0`) is specified in your scripts.

### 3️⃣ Run Test Script

```bash
python Test_pymavlink.py
```

Expected: Heartbeats and confirmation messages from Pixhawk.

### 4️⃣ Run Autonomous Takeoff & Landing

```bash
python auto_takeoff_land.py
```

Expected: UAV arms → takes off to target altitude → hovers → lands.

---
