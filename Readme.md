<div align="left" style="position: relative;">
<img src="https://raw.githubusercontent.com/PKief/vscode-material-icon-theme/ec559a9f6bfd399b82bb44393651661b08aaf7ba/icons/folder-markdown-open.svg" align="right" width="30%" style="margin: -20px 0 0 20px;">
<h1>TRACKING DRONE</h1>
<p align="left">
	<em><code>❯ REPLACE-ME</code></em>
</p>
<p align="left">
	<!-- Shields.io badges disabled, using skill icons. --></p>
<p align="left">Built with the tools and technologies:</p>
<p align="left">
	<a href="https://skillicons.dev">
		<img src="https://skillicons.dev/icons?i=cpp,cmake,md">
	</a></p>
</div>
<br clear="right">

##  Table of Contents

- [ Overview](#-overview)
- [ Features](#-features)
- [ Project Structure](#-project-structure)
  - [ Project Index](#-project-index)
- [ Getting Started](#-getting-started)
  - [ Prerequisites](#-prerequisites)
  - [ Installation](#-installation)
  - [ Usage](#-usage)
  - [ Testing](#-testing)
- [ Project Roadmap](#-project-roadmap)
- [ Contributing](#-contributing)
- [ License](#-license)
- [ Acknowledgments](#-acknowledgments)

---

##  Overview

<code>❯ A ROS2-based project for object detection and multi-object tracking (MOT) using YOLO models and OC-Sort algorithm, designed for drone applications.

</code>

---

##  Features

<code>❯ REPLACE-ME</code>

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


###  Project Index
<details open>
	<summary><b><code>TRACKINGDRONE/</code></b></summary>
	<details> <!-- __root__ Submodule -->
		<summary><b>__root__</b></summary>
		<blockquote>
			<table>
			</table>
		</blockquote>
	</details>
	<details> <!-- src Submodule -->
		<summary><b>src</b></summary>
		<blockquote>
			<details>
				<summary><b>detect_and_track_bringup</b></summary>
				<blockquote>
					<table>
					<tr>
						<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detect_and_track_bringup/CMakeLists.txt'>CMakeLists.txt</a></b></td>
						<td><code>❯ REPLACE-ME</code></td>
					</tr>
					</table>
					<details>
						<summary><b>launch</b></summary>
						<blockquote>
							<table>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detect_and_track_bringup/launch/bringup.launch.py'>bringup.launch.py</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							</table>
						</blockquote>
					</details>
					<details>
						<summary><b>config</b></summary>
						<blockquote>
							<table>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detect_and_track_bringup/config/video_manager_params.yaml'>video_manager_params.yaml</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detect_and_track_bringup/config/detection_and_mot_params.yaml'>detection_and_mot_params.yaml</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detect_and_track_bringup/config/stream_manager_sim_params.yaml'>stream_manager_sim_params.yaml</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detect_and_track_bringup/config/stream_manager_real_params.yaml'>stream_manager_real_params.yaml</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detect_and_track_bringup/config/moc_video_publisher_params.yaml'>moc_video_publisher_params.yaml</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							</table>
						</blockquote>
					</details>
				</blockquote>
			</details>
			<details>
				<summary><b>common_msgs</b></summary>
				<blockquote>
					<table>
					<tr>
						<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/common_msgs/CMakeLists.txt'>CMakeLists.txt</a></b></td>
						<td><code>❯ REPLACE-ME</code></td>
					</tr>
					</table>
					<details>
						<summary><b>msg</b></summary>
						<blockquote>
							<table>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/common_msgs/msg/Detections.msg'>Detections.msg</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/common_msgs/msg/Detection.msg'>Detection.msg</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							</table>
						</blockquote>
					</details>
					<details>
						<summary><b>srv</b></summary>
						<blockquote>
							<table>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/common_msgs/srv/ChangeTarget.srv'>ChangeTarget.srv</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							</table>
						</blockquote>
					</details>
				</blockquote>
			</details>
			<details>
				<summary><b>detection_and_mot</b></summary>
				<blockquote>
					<table>
					<tr>
						<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/CMakeLists.txt'>CMakeLists.txt</a></b></td>
						<td><code>❯ REPLACE-ME</code></td>
					</tr>
					</table>
					<details>
						<summary><b>src</b></summary>
						<blockquote>
							<table>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/src/detection_and_mot_node.cpp'>detection_and_mot_node.cpp</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							</table>
						</blockquote>
					</details>
					<details>
						<summary><b>include</b></summary>
						<blockquote>
							<details>
								<summary><b>ocsort</b></summary>
								<blockquote>
									<table>
									<tr>
										<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/include/ocsort/Association.hpp'>Association.hpp</a></b></td>
										<td><code>❯ REPLACE-ME</code></td>
									</tr>
									<tr>
										<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/include/ocsort/lapjv.hpp'>lapjv.hpp</a></b></td>
										<td><code>❯ REPLACE-ME</code></td>
									</tr>
									<tr>
										<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/include/ocsort/KalmanBoxTracker.hpp'>KalmanBoxTracker.hpp</a></b></td>
										<td><code>❯ REPLACE-ME</code></td>
									</tr>
									<tr>
										<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/include/ocsort/OCSort.hpp'>OCSort.hpp</a></b></td>
										<td><code>❯ REPLACE-ME</code></td>
									</tr>
									<tr>
										<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/include/ocsort/Utilities.hpp'>Utilities.hpp</a></b></td>
										<td><code>❯ REPLACE-ME</code></td>
									</tr>
									<tr>
										<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/include/ocsort/KalmanFilter.hpp'>KalmanFilter.hpp</a></b></td>
										<td><code>❯ REPLACE-ME</code></td>
									</tr>
									</table>
								</blockquote>
							</details>
							<details>
								<summary><b>detection_and_mot</b></summary>
								<blockquote>
									<table>
									<tr>
										<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/include/detection_and_mot/point.hpp'>point.hpp</a></b></td>
										<td><code>❯ REPLACE-ME</code></td>
									</tr>
									<tr>
										<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/include/detection_and_mot/YOLO.hpp'>YOLO.hpp</a></b></td>
										<td><code>❯ REPLACE-ME</code></td>
									</tr>
									<tr>
										<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/include/detection_and_mot/nanoflann.hpp'>nanoflann.hpp</a></b></td>
										<td><code>❯ REPLACE-ME</code></td>
									</tr>
									</table>
								</blockquote>
							</details>
						</blockquote>
					</details>
					<details>
						<summary><b>resources</b></summary>
						<blockquote>
							<table>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/resources/coco_mod.names'>coco_mod.names</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/detection_and_mot/resources/coco.names'>coco.names</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							</table>
						</blockquote>
					</details>
				</blockquote>
			</details>
			<details>
				<summary><b>video_manager</b></summary>
				<blockquote>
					<table>
					<tr>
						<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/video_manager/CMakeLists.txt'>CMakeLists.txt</a></b></td>
						<td><code>❯ REPLACE-ME</code></td>
					</tr>
					</table>
					<details>
						<summary><b>src</b></summary>
						<blockquote>
							<table>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/video_manager/src/video_manager_node.cpp'>video_manager_node.cpp</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							</table>
						</blockquote>
					</details>
					<details>
						<summary><b>launch</b></summary>
						<blockquote>
							<table>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/video_manager/launch/video_manager.launch.py'>video_manager.launch.py</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							</table>
						</blockquote>
					</details>
					<details>
						<summary><b>scripts</b></summary>
						<blockquote>
							<table>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/video_manager/scripts/run_video_manager.sh'>run_video_manager.sh</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							</table>
						</blockquote>
					</details>
					<details>
						<summary><b>include</b></summary>
						<blockquote>
							<details>
								<summary><b>video_manager</b></summary>
								<blockquote>
									<table>
									<tr>
										<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/video_manager/include/video_manager/draw.hpp'>draw.hpp</a></b></td>
										<td><code>❯ REPLACE-ME</code></td>
									</tr>
									<tr>
										<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/video_manager/include/video_manager/video_manager.hpp'>video_manager.hpp</a></b></td>
										<td><code>❯ REPLACE-ME</code></td>
									</tr>
									</table>
								</blockquote>
							</details>
						</blockquote>
					</details>
				</blockquote>
			</details>
			<details>
				<summary><b>moc_video_publisher</b></summary>
				<blockquote>
					<table>
					<tr>
						<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/moc_video_publisher/setup.py'>setup.py</a></b></td>
						<td><code>❯ REPLACE-ME</code></td>
					</tr>
					</table>
					<details>
						<summary><b>test</b></summary>
						<blockquote>
							<table>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/moc_video_publisher/test/test_copyright.py'>test_copyright.py</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/moc_video_publisher/test/test_flake8.py'>test_flake8.py</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/moc_video_publisher/test/test_pep257.py'>test_pep257.py</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							</table>
						</blockquote>
					</details>
					<details>
						<summary><b>moc_video_publisher</b></summary>
						<blockquote>
							<table>
							<tr>
								<td><b><a href='https://github.com/ahmedmsalah99/TrackingDrone/blob/master/src/moc_video_publisher/moc_video_publisher/moc_video_publisher_node.py'>moc_video_publisher_node.py</a></b></td>
								<td><code>❯ REPLACE-ME</code></td>
							</tr>
							</table>
						</blockquote>
					</details>
				</blockquote>
			</details>
		</blockquote>
	</details>
</details>

---
##  Getting Started

###  Prerequisites

Before getting started with TrackingDrone, ensure your runtime environment meets the following requirements:

- **Programming Language:** CPP
- **Package Manager:** Cmake


###  Installation

Install TrackingDrone using one of the following methods:

**Build from source:**

1. Clone the TrackingDrone repository:
```sh
❯ git clone https://github.com/ahmedmsalah99/TrackingDrone
```

2. Navigate to the project directory:
```sh
❯ cd TrackingDrone
```

3. Install the project dependencies:


**Using `cmake`** &nbsp; [<img align="center" src="https://img.shields.io/badge/C++-00599C.svg?style={badge_style}&logo=c%2B%2B&logoColor=white" />](https://isocpp.org/)

```sh
❯ cmake . && make
```




###  Usage
Run TrackingDrone using the following command:
**Using `cmake`** &nbsp; [<img align="center" src="https://img.shields.io/badge/C++-00599C.svg?style={badge_style}&logo=c%2B%2B&logoColor=white" />](https://isocpp.org/)

```sh
❯ ./TrackingDrone
```


###  Testing
Run the test suite using the following command:
**Using `cmake`** &nbsp; [<img align="center" src="https://img.shields.io/badge/C++-00599C.svg?style={badge_style}&logo=c%2B%2B&logoColor=white" />](https://isocpp.org/)

```sh
❯ ctest
```


---
##  Project Roadmap

- [X] **`Task 1`**: <strike>Implement feature one.</strike>
- [ ] **`Task 2`**: Implement feature two.
- [ ] **`Task 3`**: Implement feature three.

---

##  Contributing

- **💬 [Join the Discussions](https://github.com/ahmedmsalah99/TrackingDrone/discussions)**: Share your insights, provide feedback, or ask questions.
- **🐛 [Report Issues](https://github.com/ahmedmsalah99/TrackingDrone/issues)**: Submit bugs found or log feature requests for the `TrackingDrone` project.
- **💡 [Submit Pull Requests](https://github.com/ahmedmsalah99/TrackingDrone/blob/main/CONTRIBUTING.md)**: Review open PRs, and submit your own PRs.

<details closed>
<summary>Contributing Guidelines</summary>

1. **Fork the Repository**: Start by forking the project repository to your github account.
2. **Clone Locally**: Clone the forked repository to your local machine using a git client.
   ```sh
   git clone https://github.com/ahmedmsalah99/TrackingDrone
   ```
3. **Create a New Branch**: Always work on a new branch, giving it a descriptive name.
   ```sh
   git checkout -b new-feature-x
   ```
4. **Make Your Changes**: Develop and test your changes locally.
5. **Commit Your Changes**: Commit with a clear message describing your updates.
   ```sh
   git commit -m 'Implemented new feature x.'
   ```
6. **Push to github**: Push the changes to your forked repository.
   ```sh
   git push origin new-feature-x
   ```
7. **Submit a Pull Request**: Create a PR against the original project repository. Clearly describe the changes and their motivations.
8. **Review**: Once your PR is reviewed and approved, it will be merged into the main branch. Congratulations on your contribution!
</details>

<details closed>
<summary>Contributor Graph</summary>
<br>
<p align="left">
   <a href="https://github.com{/ahmedmsalah99/TrackingDrone/}graphs/contributors">
      <img src="https://contrib.rocks/image?repo=ahmedmsalah99/TrackingDrone">
   </a>
</p>
</details>

---

##  License

This project is protected under the [SELECT-A-LICENSE](https://choosealicense.com/licenses) License. For more details, refer to the [LICENSE](https://choosealicense.com/licenses/) file.

---

##  Acknowledgments

- List any resources, contributors, inspiration, etc. here.

---