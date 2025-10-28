# 🤖 DiffDriveRobot  

## 🧭 Overview
**DiffDriveRobot** is the first open-source platform in the *Project Gradus* series,  
implementing a fundamental **two-wheel differential-drive** structure.  

<div align="center">
  <img src="images/diffdrive_front.jpg" width="45%" />
  <img src="images/diffdrive_side.jpg" width="45%" />
  <p><em>▲ Figure 1. Front and side views of the DiffDriveRobot prototype</em></p>
</div>

This robot is designed as a starting point for testing  
control, odometry, and navigation algorithms,  
serving as the foundation for upcoming **Swerve, Quadruped, and Bipedal** robots in the *Project Gradus* roadmap.

---

## 📁 Directory Description

| Folder                          | Description                                                             |
| ------------------------------- | ----------------------------------------------------------------------- |
| **`hardware/design/`**          | Original CAD models and 3D-printable STL files                          |
| **`hardware/assembly/`**        | Assembly guide, wiring diagrams, and commercial parts list              |
| **`hardware/microcontroller/`** | ESP32 / micro-ROS firmware for motor control and encoder handling       |
| **`hardware/bringup/`**         | ROS 2 bring-up scripts and nodes for `/cmd_vel` ↔ `/odom` communication |
| **`simulation/`**               | Gazebo setup for virtual testing                                        |
| **`images/`**                   | Visual resources used in README or documentation                        |
| **`docs/`**                     | Technical notes, system diagrams, and development records               |

> 💡 The `hardware/` directory unifies every step of real-world implementation —
> from **design → assembly → microcontroller → ROS 2 bring-up** —
> ensuring reproducibility and modular hardware development.

---

## ⚙️ Key Features

* 🧩 **ROS 2 Integration:** Fully compatible with ROS 2 Humble and later
* 🧠 **Odometry / Encoder Support:** Real-time velocity and pose estimation
* 🎮 **Velocity Control Interface:** `/cmd_vel` topic for teleop and autonomous control
* 🪶 **Lightweight 3D-Printed Chassis:** Low-cost, easily modifiable body

  * The list of commercial off-the-shelf parts is available under **[Hardware → Assembly](#-commercial-parts-list)**
* 🧱 **Gazebo Simulation Environment:** URDF + world files provided

---

## 🧪 Simulation & Real-World Operation

### 🔹 Real-World Setup

**ESP32 (MCU)**

* Publishes `/odom` topic → encoder-based odometry data
* Subscribes to `/cmd_vel` → velocity commands from workstation

**Laptop / Workstation**

* Publishes `/cmd_vel` → drives the robot
* Subscribes to `/odom` → receives pose feedback for control and visualization

> ⚙️ Firmware under `hardware/microcontroller/` handles the real-time control logic,
> while `hardware/bringup/` contains the ROS 2 nodes and launch files used to operate the robot.

---

### 🔹 Gazebo Simulation (Coming Soon)

Run the robot in Gazebo with:

```bash
ros2 launch diffdrive_robot simulation.launch.py
```

Keyboard teleoperation example:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/cmd_vel
```

---

## 🛠️ Commercial Parts List

*(to be detailed in `hardware/assembly/`)*

| Component      | Model / Link     | Description                   |
| :------------- | :--------------- | :---------------------------- |
| DC Motor       | TBD              | BLDC motors  |
| Encoder        | TBD              | Wheel rotation measurement    |
| Motor Driver   | TB6612 / L298N   | BLDC motor controller |
| Battery        | LiFePO₄ 24 V     | --           |
| Caster Wheel   | TBD              | Front, Rear stabilizer wheel         |

> 🧩 *This list is under continuous update.*

---

## 🚀 Future Development


---


## 📜 License

This project follows an open-source philosophy.
All resources are freely available for research and educational use.
(Specific license details can be found in the `LICENSE` file.)
---

