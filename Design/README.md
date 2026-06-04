# 🛠️ Design Files

This folder contains all STEP and STL files needed to 3D print and assemble the gripper used in this tutorial. Files are organised into two subfolders:

- 📐 **`step/`** — STEP files for editing or importing into CAD software
- 🧩 **`stl/`** — STL files ready for slicing and printing

---

## 🖨️ Printing Settings

All parts were printed using a **Bambu Lab H2D** printer with the following settings:

| Parameter | Value |
|---|---|
| Infill | 20% |
| Material | PLA (recommended) |

---

## 🧱 Components

### 📦 From the PincOpen Repository (unmodified)

These parts are sourced directly from the [Pollen Robotics PincOpen repository](https://github.com/pollen-robotics/PincOpen/tree/main/cad). Download them there, they are not duplicated here.

| Part | Quantity |
|---|---|
| Internal Rod | 2 |
| External Rod | 2 |
| Distal Rod | 2 |
| Top Plate | 1 |
| Motor Flange | 1 |
| Cam | 1 |
| Driving Rod | 2 |
| Bottom Plate | 1 |

---

### 🔧 Modified from PincOpen (adapted for Tactaxis® sensor integration)

These parts originate from the PincOpen design but have been modified to accommodate the Melexis Tactaxis® 3D force sensors in the fingertips.

| Part | Quantity |
|---|---|
| Removable Tip | 2 |
| Distal Shell | 2 |

---

### ✨ New Components (added for electronics integration)

These parts are fully original additions to the design, providing housing for the PCB and control electronics.

| Part | Quantity |
|---|---|
| Head Housing | 1 |
| Interface Panel for Head Housing | 1 |
| Top Panel for Head Housing | 1 |
| Potentiometer Knob | 1 |

---

### 🎯 Sensor Reference Model

A reference STEP and STL of the Tactaxis® sensor body is included for alignment and integration purposes.

| Part | Quantity |
|---|---|
| Tactaxis Sensor | 2 |

---

## 📡 Tactaxis® Sensor — Info & Dimensions

The **Melexis Tactaxis® 3D force sensor** is a compact, automotive-grade Hall-effect sensor that simultaneously measures normal (Fz) and shear (Fx, Fy) forces. It is designed for integration into robotic fingertips where space and robustness are critical.

| Dimensions | Value |
|---|---|
| Surface | ~24 mm × 18 mm |
| Overall assembly height | ~6 mm (sensor + magnet housing) |
| Number of taxels per sensor | 4 |

> The sensor snaps into the **Removable Tip** cavity. The STEP file `step/Tactaxis_sensor.STEP` represents the sensor body and can be used as a reference when adapting the fingertip geometry.

---

## 📝 Notes

- The STEP files can be opened and modified in any CAD software (e.g. FreeCAD, Fusion 360, SolidWorks, OnShape).
- If you want to adapt the fingertip design for a different sensor, the **Removable Tip** and **Distal Shell** STEP files are the relevant starting points.
- For the full bill of materials including fasteners and electronics, refer to `BOM.xlsx` at the root of the repository.
