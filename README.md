# 🤖 ICRA 2026 Tutorial: 3D Tactile Sensor Integration in Robotic Fingers for Smart Manipulation

**Hands-on Tutorial at ICRA 2026**

**Organisers:** Milan Amighi, Constantin Scholz, and Bram Vanderborght  
*(Brubotics – VUB & imec, in collaboration with Melexis)*

---

## 🧠 Overview

Despite skin being our largest sensory organ, touch remains underused in robotics. Tactile sensing is vital for safe and dexterous manipulation, human-robot interaction, and wearable devices, yet most robotic sensors measure only normal pressure. We live among materials with distinct rigidity, friction, weight, and texture, differences that humans intuitively perceive through touch. Robots can already move, see, and hear; the next frontier is enabling them to truly *feel*, so they can grasp delicate and deformable objects while accounting for each object's physical properties.

This hands-on tutorial introduces an affordable, compact, and robust 3D tactile sensor developed by Melexis: the **Tactaxis® 3D force sensor**, which uses automotive-grade Hall-effect technology to measure both normal and shear forces simultaneously.

![Melexis Tactaxis 3D Force Sensor](Pictures/Sensor.png)

<p align="center">
  <img src="Pictures/Touch_sensor.gif" alt="Touch sensor demo">
</p>

Participants will work in small teams to:
- 🔩 Assemble a two-finger gripper ([PincOpen from Pollen Robotics](https://pollen-robotics.github.io/PincOpen/))
- 🖐️ Integrate the Tactaxis® sensor into the fingertips
- 🚀 Deploy the full system on a real robotic manipulation task

Through guided experiments, you will map force signals to interaction dynamics, detect slip, and implement adaptive grasp control that responds to the properties of the object being handled. You will leave with practical know-how, open-source resources, and a grounded understanding of why touch enables safer, more precise, and more versatile robot behavior.

![Full gripper overview](Pictures/Full_gripper.png)



**Concept demonstration (ITF 2025 with imec):** [▶️ Watch on YouTube](https://www.youtube.com/watch?v=10evZqkg7gM&time_continue=62&source_ve_path=MjM4NTE&embeds_referring_euri=https%3A%2F%2Fmech.vub.ac.be%2F)

---

## 📁 Repository Structure

```
.
├── BOM.xlsx                          # Bill of materials: electronics, fasteners, and parts needed to build the gripper, with purchase links
├── Gripper Kit - Assembly guide.pdf  # Step-by-step assembly guide for the full gripper
├── Electronics/                      # Circuit diagrams and electronics documentation
├── Design/                           # STEP and STL files for 3D printing
├── Docker/                           # Dockerfile and scripts to create a containerised ROS 2 environment
└── Codes/                            # ROS 2 packages, python only and motor initialisation scripts
```

---

## ✅ Prerequisites

- Basic familiarity with ROS 2 and Python is helpful but not required
- Docker must be installed on your laptop to use the provided environment
- All hardware kits (3D-printed parts, fasteners, sensors) will be provided on-site

---


## 🚀 Before you start

You need:

- The ESP32-S3 hardware setup powered and connected by USB,
- Python installed on your PC (if you use the host scripts). A Docker image is also available with all the libraries and ROS 2 Humble dependencies pre-installed,
- The correct serial port (e.g. `COM9` on Windows or `/dev/ttyACM0` on Ubuntu), or let the scripts detect it automatically.

### 📦 Install Python dependencies

If you are running the scripts directly on the host (without Docker), install the required Python packages from the root of the repository:

```bash
pip install -r requirements.txt
```

The `requirements.txt` includes: `numpy`, `pyserial`, `crc`, `esptool`. You can also install ROS 2 Humble via the [official installation guide](https://docs.ros.org/en/humble/Installation.html).

### 🔥 Flash the Firmware

If the ESP32 has not been flashed yet, run the flashing script:

```bash
#If you are not using docker
cd ~/ICRA2026_Tutorial_3D-tactile-sensor-integration-in-robotic-fingers-for-smart-manipulation/Codes/python_codes/flash

#If you are using docker (look "Getting started")
cd /ICRA2026_Tutorial_3D-tactile-sensor-integration-in-robotic-fingers-for-smart-manipulation/Codes/python_codes/flash

#To launch the code
python3 flash.py
```

### ⚙️ Motor Initialisation

Run this script first to confirm the serial link is working and bring the motor to its reference position before attaching it to the gripper.

> ⚠️ Make sure the gripper LCD is set to **Connected** mode before running any ROS 2 node or Python script.

```bash
#If you are not using docker
cd ~/ICRA2026_Tutorial_3D-tactile-sensor-integration-in-robotic-fingers-for-smart-manipulation/Codes/python_codes

#If you are using docker (look "Getting started")
cd /ICRA2026_Tutorial_3D-tactile-sensor-integration-in-robotic-fingers-for-smart-manipulation/Codes/python_codes

#To launch the code
python3 Motor_initialisation.py
```

---

## 🏁 Getting Started

> The following code has been tested on Ubuntu 22.04. A Dockerfile is provided to automatically install all required libraries. The Python-only scripts can also be run on Windows, provided the necessary libraries are installed and the correct port is selected.

1. Clone this repository:
   ```bash
   git clone ...
   ```
2. Follow the assembly guide: `Gripper Kit - Assembly guide.pdf` to build the entire gripper.
3. Once the gripper is built, you can start to play with it. Either:
   - 🐳 **With Docker** — build and run the container (includes all ROS 2 and Python dependencies):
     ```bash
     cd ~/ICRA2026_Tutorial_3D-tactile-sensor-integration-in-robotic-fingers-for-smart-manipulation/Docker
     ./buildrun.sh
     ```
     To open an additional shell inside the running container:
     ```bash
     docker exec -it docker3dsensors bash
     ```
   - 🐍 **Without Docker** — install dependencies directly on the host:
     ```bash
     pip install -r requirements.txt
     ```
4. Launch the ROS 2 nodes or the Python-only code as described in the `Codes/` folder.

---

## 🙏 Acknowledgements

This project was made possible with the support of **VLAIO** under the Skinaxis project and the **euRobin EU project**. We would also like to sincerely thank:
- **Melexis**, for making this tutorial possible by providing the Tactaxis® sensors
- **ICRA organisers**, for the opportunity to present to the robotics community
- **Pollen Robotics**, for supporting the project with the PincOpen gripper platform

---

## 📬 Contact

| Organisation | Contact |
|---|---|
| Brubotics (VUB & imec) | [Milan.Francois.T.Amighi@vub.be](mailto:Milan.Francois.T.Amighi@vub.be) |
| Melexis | [tls@melexis.com](mailto:tls@melexis.com) |

---

## 📚 References

- T. Le Signor, N. Dupré and G. F. Close, "A Gradiometric Magnetic Force Sensor Immune to Stray Magnetic Fields for Robotic Hands and Grippers," *IEEE Robotics and Automation Letters*, vol. 7, no. 2, pp. 3070–3076, April 2022. [doi:10.1109/LRA.2022.3146507](https://doi.org/10.1109/LRA.2022.3146507)

- T. Le Signor, N. Dupré, J. Didden, E. Lomakin, G. Close, "Mass-Manufacturable 3D Magnetic Force Sensor for Robotic Grasping and Slip Detection," *Sensors*, 23, 3031, 2023. [doi:10.3390/s23063031](https://doi.org/10.3390/s23063031)

- PincOpen Gripper — Pollen Robotics: [https://pollen-robotics.github.io/PincOpen/](https://pollen-robotics.github.io/PincOpen/)
