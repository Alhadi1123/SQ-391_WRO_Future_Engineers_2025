# WRO Future Engineers 2025 — Main Robot Code

This folder contains all code developed for the **WRO Future Engineers 2025** competition.  
It includes both legacy (old) versions and the final version used in the official robot during the competition.

---

## 📂 Repository Structure

```
📁 Old Version main code/
    └── Legacy implementation of the main robot logic (pre-final version).

📁 Old Version test and calibration/
    └── Experimental scripts for testing and sensor calibration.

📁 main code/catkin_ws/
    ├── src/
    │   ├── obstacle_challenge/
    │   │   └── launch/
    │   │       └── obstacle_run.launch
    │   └── open_challenge/
    │       └── launch/
    │           └── open_run.launch
    └── (ROS workspace used in competition)
```

---

## 🧠 About the Versions

### 🔹 Old Versions
- **Old Version main code** and **Old Version test and calibration** contain early prototypes and test scripts.
- They were used for calibration, component verification, and initial behavior development.

### 🔹 Main Code (Used in Competition)
- Located under:  
  ```
  main code/catkin_ws/
  ```
- This is the **ROS workspace** that runs the robot’s final version, including all control, vision, and navigation nodes.
- It was the version deployed during the **official WRO Future Engineers 2025** competition.

---

## 🚀 How to Run

### 1️⃣ Obstacle Challenge
To launch the full robot system for the **Obstacle Challenge**, run:

```bash
roslaunch obstacle_challenge obstacle_run.launch
```

### 2️⃣ Open Challenge
To launch the full robot system for the **Open Challenge**, run:

```bash
roslaunch open_challenge open_run.launch
```

> 💡 Run these commands inside the ROS workspace:
> ```bash
> cd main\ code/catkin_ws
> source devel/setup.bash
> ```

---

## ⚙️ Notes
- Ensure all dependencies are installed before launching (e.g., sensor drivers, camera node, control node).
- The launch files automatically start all relevant nodes and parameters for each challenge.
