from setuptools import find_packages, setup

package_name = 'marinero_pointclouds'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albert',
    maintainer_email='albert.androsic2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ply_pcd_remapper = marinero_pointclouds.ply_pcd_remapper:main',
            'ply_publisher = marinero_pointclouds.ply_publisher:main',
            'remapped_ply_publisher = marinero_pointclouds.remapped_ply_publisher:main',
            'remapped_marina_ply_publisher = marinero_pointclouds.remapped_marina_ply_publisher:main',
            'remapped_segmented_ply_publisher = marinero_pointclouds.remapped_segmented_ply_publisher:main',
            'remapped_pcd_publisher = marinero_pointclouds.remapped_pcd_publisher:main',
            'remapped_marina_pcd_publisher = marinero_pointclouds.remapped_marina_pcd_publisher:main',
            'remapped_segmented_pcd_publisher = marinero_pointclouds.remapped_segmented_pcd_publisher:main',
            'remapped_segmented_pcd_publisher_thread = marinero_pointclouds.remapped_segmented_pcd_publisher_thread:main',
            '01_client_remapped_segmented_pcd_publisher = marinero_pointclouds.01_client_remapped_segmented_pcd_publisher:main',
            '01_server_remapped_segmented_pcd_publisher = marinero_pointclouds.01_server_remapped_segmented_pcd_publisher:main',
            "count_until_server = marinero_pointclouds.count_until_server:main",
            "count_until_client = marinero_pointclouds.count_until_client:main"
        ],
    },
)
