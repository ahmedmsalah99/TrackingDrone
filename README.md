<h1>TRACKING DRONE</h1>

![cover](https://pic.onlinewebfonts.com/thumbnails/icons_665129.svg)

![License](https://img.shields.io/badge/license-AGPLv3-blue.svg)
![C++](https://img.shields.io/badge/language-C++-blue.svg)
![ONNX Runtime](https://img.shields.io/badge/ONNX_Runtime-v1.20.1-brightgreen.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-4.5.5-brightgreen.svg)
![CMake](https://img.shields.io/badge/CMake-3.22.1-blue.svg)

##  Table of Contents

- [ Overview](#-overview)
- [ Features](#-features)
- [ Project Structure](#-project-structure)
- [ Getting Started](#-getting-started)
  - [ Prerequisites](#-prerequisites)
  - [ Build and Run](#-build-and-run)
  - [ Usage](#-usage)
  - [ Params](#-params)
- [ Project Roadmap](#-project-roadmap)
- [ License](#-license)
- [ Acknowledgments](#-acknowledgments)

---

##  Overview

<code> A **ROS2-based** project for object detection and multi-object tracking (**MOT**) using **YOLO** models and **OC-Sort** algorithm, designed for drone applications.
</code>

---

##  Features


* **Object Detection:** Employs **YOLO** models for **real-time** object detection.  
* **Multi-Object Tracking (MOT):** Utilizes **OC-Sort** for robust tracking of detected objects.  
* **ROS2 Integration:** Seamlessly integrates with **ROS2** for robotic applications.  
* **Modular Design:** Components are designed for flexibility and reusability.  
* **Video Stream Processing:** Processes video frames from various sources (**simulated or real-time**).  
* **Configurable Parameters:** Allows **fine-tuning** of detection and tracking parameters.



---

##  Project Structure

```sh
└── TrackingDrone/
    ├── Readme.md
    └── src
        ├── .gitignore
        ├── common_msgs
        ├── detect_and_track_bringup
        ├── detection_and_mot
        ├── moc_video_publisher
        └── video_manager
```

---
##  Getting Started

###  Prerequisites

Before getting started with TrackingDrone, ensure your runtime environment meets the following requirements:

- **ROS2 humble** 
- **OpenCV**
- **ONNX Runtime**
- **Eigen3**


**Build and Run:**

1. Clone the **TrackingDrone** repository:
```sh
❯ git clone --recursive https://github.com/ahmedmsalah99/TrackingDrone.git
```

2. Download the resources from **OneDrive** and place them in the right places:
[🔗 OneDrive Folder](https://1drv.ms/f/c/ba4a36e37e136c1e/Espk4TyxO-hJscBR4_XtCf4BybTYcVzWY7BKRy2OZeNyQw?e=z4Fg1S)

* You can download the suitable **onnx-runtime** for your system from the following link:
[ONNX-RUNTIME](https://github.com/microsoft/onnxruntime/releases)

* Rename the folder to **onnxruntime** and place it in **src/detection_and_mot**

* You can also convert your model on your device to **onnx** and use it instead of the provided model which is going to run faster.

* Also, in **src/detection_and_mot/lib** the **oc-sort** library is there as a shared library. If you need to build it on your own use the modified repo: https://github.com/ahmedmsalah99/OC_SORT

3. **In the work space folder:**
```sh
❯ colcon build
```

4. **Setup the environment:**
```sh
❯ source install/setup.bash
❯ export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
❯ export $(pwd)/src/ros2_shm_msgs/config/shm_fastdds.xml
❯ export RMW_FASTRTPS_USE_QOS_FROM_XML=1
```


5. **Launch the system**
```sh
❯ ros2 launch detect_and_track_bringup bringup.launch.py
```



###  Params
The easiest way to configure the project is to edit parameters in **src/detect_and_track_bringup/config**:


1) **detection_and_mot_params:**

* **detection_fps** is how often you run your detection. Set it to your min. model's fps.

* **delay_time** is the shift in time in past that detections are published with. This depends on the **detection delay + tracking delay time**.

2) **stream_manager_*_params.yaml**

* **target_fps** is the fps your stream manager is going to publish. It's limitng maximum fps.

3) **detection_time_out_params.yaml**

* **detection_time_out** is the time in seconds that the system will wait for a detection before considering it as a timeout.




---
##  Project Roadmap

- [X] **`Task 1`**: <strike>Implement detection and MOT.</strike>
- [ ] **`Task 2`**: Implement Simulation.
- [ ] **`Task 3`**: Implement VIO.
- [ ] **`Task 4`**: Implement Control.
- [ ] **`Task 5`**: Implement Veichle tracking.
- [ ] **`Task 6`**: Implement Veichle position estimation.

---

##  License

This project is protected under the [GNU AGPLv3
](https://choosealicense.com/licenses/agpl-3.0/) License.
---

##  Acknowledgments

- [YOLOs-CPP](https://github.com/Geekgineer/YOLOs-CPP)
- [OC-Sort](https://github.com/noahcao/OC_SORT)
- [ros2_shm_msgs](https://github.com/ZhenshengLee/ros2_shm_msgs)

---