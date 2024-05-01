import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    bringup_dir = get_package_share_directory('nvblox_examples_bringup')

    # Launch Arguments
    launch_realsense_arg = DeclareLaunchArgument(
        'launch_realsense', default_value='True',
        description='Whether to launch the realsense driver')
    run_rqt_arg = DeclareLaunchArgument(
        'run_rqt', default_value='True',
        description='Whether to start rqt_image_view')

    # Realsense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'sensors', 'realsense.launch.py')]),
        condition=IfCondition(LaunchConfiguration('launch_realsense')))


    ######## Optrise imager launch file

    Optris_image = Node(
            package='optris_drivers2',  # Replace with your package name
            executable='optris_imager',
            name='optris_imager',
            parameters=[{'serial': 'your_device_serial'}],  # Replace with your device serial
            output='screen'
        )
    
    # Bag recording
    realsense_topics = [
        '/tf_static',
        '/camera/color/camera_info',
        '/camera/color/image_raw',
        '/camera/realsense_splitter_node/output/depth',
        '/camera/depth/camera_info',
        '/camera/realsense_splitter_node/output/infra_1',
        '/camera/infra1/camera_info',
        '/camera/realsense_splitter_node/output/infra_2',
        '/camera/infra2/camera_info'
    ]
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', " ".join(realsense_topics)],
        shell=True, output='screen')

    # Rqt
    rqt_launch = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        condition=IfCondition(LaunchConfiguration('run_rqt')))

    recording_started_msg =\
        '''\n\n\n
        -----------------------------------------------------
                    BAG RECORDING IS STARTING NOW

                 (make sure the realsense node is up)
        -----------------------------------------------------
        \n\n\n'''
    return LaunchDescription([
        launch_realsense_arg,
        run_rqt_arg,

        # Start the realsense and rqt image view
        Optris_image,
        realsense_launch,
        rqt_launch,

        # Start recording after 10 sec.
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg=recording_started_msg),
                bag_record
            ])])