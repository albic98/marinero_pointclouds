
# MARINERO Pointclouds

Contains .py scripts for handling pointclouds of Marina Punat.

---

## Requirements  

- **ROS 2 Humble** (or compatible ROS 2 distribution)  
- Packages:  
  - `pcl`  
  - `numpy`  
  - `open3d`   
  - `rviz2` 

---

## Installation

Clone the repositroy with all the required dependencies in your ROS2 workspace.

```
  cd workspace_folder/src
  git clone https://github.com/albic98/marinero_pointclouds.git
  colcon build --symlink-install
  source install/setup.bash
```

---

## Getting started

Script for formatting .ply files to .pcd format is `ros2 run ply_pcd_remapper.py`.

Latest script that is used for visualizing pointclouds in RViz is `remapped_segmented_pcd_publisher_thread.py`.

The entire Marina Punat can be visualized using either of the following scripts: 
`remapped_marina_pcd_publisher.py`, `remapped_marina_ply_publisher.py`. 

However, I recommend using `remapped_marina_pcd_publisher.py` as it provides faster visualization.

Before using the pointcloud visualization scripts, update the paths to the .pcd files so they point to the correct location on your system:
```
    zone_A: "/path/to/LIDAR_data/Marina_Punat_zona_A_500K_remapped.pcd",
            "/path/to/LIDAR_data/Marina_Punat_zona_B_15M_remapped.pcd",

    zone_B: "/path/to/LIDAR_data/Marina_Punat_zona_B_500K_remapped.pcd",
            "/path/to/LIDAR_data/Marina_Punat_zona_B_15M_remapped.pcd"

    zone_C: "/path/to/LIDAR_data/Marina_Punat_zona_C_200K_remapped.pcd",
            "/path/to/LIDAR_data/Marina_Punat_zona_C_6M_remapped.pcd"
```
Replace `/path/to/LIDAR_data/` with the absolute or relative path to the directory where you placed the .pcd files.

---

## Usage/Examples

Run the nodes as follows:

#### Segmented pointclouds (zones A, B, C):
```
ros2 run marinero_pointclouds remapped_segmented_pcd_publisher_thread
```

#### Full marina pointcloud:
```
ros2 run marinero_pointclouds remapped_marina_pcd_publisher
```

---

## Script overview

This scripts are not used in the current version of simulation but some of these can be replace the current script that is being used: `remapped_segmented_pcd_publisher_thread.py`

- `01_client_remapped_segmented_pcd_publisher.py` - Client script using a service to publish marina zones depending on robot location

- `01_server_remapped_segmented_pcd_publisher.py` - Server script for segmented zone publishing

- `ply_publisher.py` - Deprecated: publishes a single .ply file.

- `remapped_pcd_publisher.py` - Deprecated: publishes a single .pcd file from the **LIDAR_data** folder

- `remapped_ply_publisher.py` - Deprecated: publishes a single .ply file from the **LIDAR_data** folder.

- `remapped_segmented_pcd_publisher.py` - Similar to `remapped_segmented_pcd_publisher_thread.py`, but less effective as it does not use **threading**

- `remapped_segmented_ply_publisher.py` - Similar to `remapped_segmented_pcd_publisher.py`, but for .ply files

---

## Integration with MARINERO Simulations

The node `remapped_segmented_pcd_publisher_thread.py` is automatically launched through  
`gazebo_simulation.launch.py` in the [marinero_simulations](https://github.com/albic98/marinero_simulations) package.  

You **do not** need to launch it manually when running the full simulation.

---

## Support

The required .pcd files are not included in this repository due to size limitations.

For support, go to https://github.com/CRTA-Lab/marinero_stack.