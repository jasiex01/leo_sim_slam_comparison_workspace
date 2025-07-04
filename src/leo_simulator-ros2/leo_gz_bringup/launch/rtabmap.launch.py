from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization == 'True' or localization == 'true'
    icp_odometry = LaunchConfiguration('icp_odometry').perform(context)
    icp_odometry = icp_odometry == 'True' or icp_odometry == 'true'
    
    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':False,
          'subscribe_rgb':False,
          'subscribe_scan':True,
          'approx_sync':True,
          'use_action_for_goal':True,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          'Grid/FromDepth': 'false',       # Use 2D laser instead of depth
          'Grid/MaxObstacleHeight': '0.0', # Ignore height, 2D only
          'Grid/MinClusterSize': '3',
          'Grid/RangeMax': '12.0',         # Match LiDAR max range
          'Grid/MapFrameRate': '1.0',      # Map update rate
          'RGBD/OptimizeMaxError': '0.2',
          'Reg/Force3DoF': 'true',         # Essential for 2D
          'Reg/Strategy': '1',             # ICP
          'ICP/RangeMin': '0.2',
          'ICP/RangeMax': '12.0',
          'ICP/CorrespondenceRatio': '0.2',
          'ICP/MaxCorrespondenceDistance': '0.5',
          'ICP/Iterations': '30',
          'ICP/Epsilon': '0.0001',
          'ICP/PointToPlane': 'false',
    }
    arguments = []
    if localization:
        parameters['Mem/IncrementalMemory'] = 'False'
        parameters['Mem/InitWMWithAllNodes'] = 'True'
    else:
        arguments.append('-d') # This will delete the previous database (~/.ros/rtabmap.db)
               
    remappings=[
          ('scan', '/lidar/laserscan'),
          ('imu', '/imu/data_raw'),]
    if icp_odometry:
        remappings.append(('odom', 'icp_odom'))
    
    return [
        # Nodes to launch
        
        # ICP odometry (optional)
        Node(
            condition=IfCondition(LaunchConfiguration('icp_odometry')),
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[parameters, 
                        {'odom_frame_id':'icp_odom',
                         'guess_frame_id':'odom',
                         'frame_id': 'base_footprint',
                         'subscribe_scan': True,
                         'use_imu': False,
                         'Scan/RangeMin': '0.2',
                         'Scan/RangeMax': '12.0',
                         'Scan/MinAngle': '-3.14',
                         'Scan/MaxAngle': '3.14',
                         'Scan/AngleIncrement': '0.004363',  # e.g. 0.25 deg (for 360°/1024)
                         'Icp/DownsamplingStep': '1'
                         }],
            remappings=remappings),
        
        # SLAM:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=arguments),
    ]

def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),
        
        DeclareLaunchArgument(
            'icp_odometry', default_value='false',
            description='Launch ICP odometry on top of wheel odometry.'),

        OpaqueFunction(function=launch_setup)
    ])