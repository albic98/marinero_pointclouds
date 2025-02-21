
# Project title

Contains .py scripts for handling pointclouds of Marina Punat.

## Getting started

Script for formatting .ply files to .pcd format is `ply_pcd_remapper.py`.

Latest script that is used for visualizing pointclouds in RViz is `remapped_segmented_pcd_publisher_thread.py`.

The entire Marina Punat can be visualized using either of the following scripts: 
`remapped_marina_pcd_publisher.py`, `remapped_marina_ply_publisher.py`. However, I 
recommend using `remapped_marina_pcd_publisher.py` as it provides faster visualization.


## Support

LIDAR_data files are missing from the repository because they are too large for Github. To access, them write an e-mail to the email written below.

For support, email albert.androsic@fsb.unizg.hr.


## Usage/Examples

Node for segmented visualization of pointclouds for zone A, B and C of Marina Punat:
```
ros2 run marinero_pointclouds remapped_segmented_pcd_publisher_thread
```

Node for visualization of the whole Marina Punat:
```
ros2 run marinero_pointclouds remapped_marina_pcd_publisher
```




