import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

parameters = []
remappings = []

def launch_setup(context: LaunchContext, *args, **kwargs):
    parameters = [{
        'frame_id': 'base_footprint',
        'use_sim_time': True,
        'subscribe_rgbd': False,
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'wait_imu_to_init': True,
        'queue_size': 30,
        'Mem/IncrementalMemory': 'true',     # Enable SLAM mode
        'RGBD/NeighborLinkRefining': 'true', # Improves map alignment
        'RGBD/ProximityBySpace': 'true',     # Loop closure by proximity
        'Reg/Force3DoF': 'true',             # For ground robots
        'Grid/FromDepth': 'false',           # Not using depth image, only cloud
        'Grid/FromScan': 'false',            # Not using 2D scan
        'Grid/FromScanCloud': 'true',        # Use point cloud
        'Grid/MaxGroundHeight': '0.2',       # Filter ground points
        'Grid/MinClusterSize': '20',         # Filter noise
        'Grid/RangeMin': '0.3',
        'Grid/RangeMax': '5.0',
        'Optimizer/Strategy': '1',           # Use TORO optimizer
        'Rtabmap/TimeThr': '700',            # Optional: drop frames if SLAM gets too slow
    }]

    
    remappings=[('imu', '/imu/data_raw'), ('scan_cloud', '/camera/points')]
    
    return [
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),
    ]


def generate_launch_description():
    return LaunchDescription([        
        OpaqueFunction(function=launch_setup)
    ])