# Isaac Ros2 Yolov8 Custom Model 

Introduction
======================

This documentation expalins how to run `Yolov8` acceleration and `isaac_ros_visual_slam` in Jetson with `Intel Realsense D400` for a custom Yolov8 Model. 

Steps:
======================

1- We need to prepare the `isaac_ros_yolov8` as shown [here](https://github.com/AbdullahGM1/isaac_ros_yolov8_custom_model).

2- Inside the same container, we colne the required packages for `isaac_ros_visual_slam`

```
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git && git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
```

3- Modify the `isaac_ros_visual_slam_realsense.launch.py` launch file inside `/isaac_ros_visual_slam/launch` to rename the `/camera/color/image_raw` topic to `/image` and the resize of the input to `640x640`. Also, to set the `enable_depth` to true to run the `Detection` input without issues. We can paste the below code inside `isaac_ros_visual_slam_realsense.launch.py`.

```
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    realsense_camera_node = Node(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_color': True,
                'enable_depth': True,
                'depth_module.emitter_enabled': 0,
                'rgb_camera.profile': '1280x720x30', 
                'depth_module.profile': '640x360x60',
                'enable_gyro': True,
                'enable_accel': True,
                'gyro_fps': 200,
                'accel_fps': 200,
                'unite_imu_method': 2
        }],
        remappings=[('color/image_raw', '/image')]
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'camera_link',
                    'input_imu_frame': 'camera_gyro_optical_frame',
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.000244,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.001862,
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 200.0,
                    'img_jitter_threshold_ms': 22.00
                    }],
        remappings=[('stereo_camera/left/image', 'camera/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'camera/infra1/camera_info'),
                    ('stereo_camera/right/image', 'camera/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'camera/infra2/camera_info'),
                    ('visual_slam/imu', 'camera/imu')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ]
    )
    
    image_resize_node_color = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='image_resize_right',
        parameters=[{
                'output_width': 640,
                'output_height': 640,
                'keep_aspect_ratio': True
        }],
        remappings=[
            ('camera_info', '/camera/color/camera_info'),
            ('image', '/camera/color/image_raw'),
            ('resize/camera_info', '/camera/color/camera_info_resize'),
            ('resize/image', '/image')]
    )
    
    resize_launch_container = ComposableNodeContainer(
        name='resize_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            image_resize_node_color
        ],
        output='screen'
    )

    return launch.LaunchDescription([visual_slam_launch_container, realsense_camera_node, resize_launch_container])
```

4- `colcon build` the packages.

Running the system:
======================

1- Run the `yolov8`

```
ros2 launch isaac_ros_yolov8 isaac_ros_yolov8_visualize.launch.py model_file_path:=/workspaces/isaac_ros-dev/src/onnx/drone_detection_v22_Yolov8n_int8.onnx engine_file_path:=/workspaces/isaac_ros-dev/src/onnx/drone_detection_v22_Yolov8n_int8.plan input_binding_names:=['images'] output_binding_names:=['output0'] network_image_width:=640 network_image_height:=640 force_engine_update:=False image_mean:=[0.0,0.0,0.0] image_stddev:=[1.0,1.0,1.0] input_image_width:=640 input_image_height:=640 confidence_threshold:=0.60 nms_threshold:=0.45
```

2- Run the modified `isaac_ros_visual_slam` launch file:
```
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

3- Run `rviz2` we can subscribe to the following topics:

  a- `/visual_slam/tracking/vo_path` - to see the `Slam` path.
  
  b- `/visual_slam/tracking/vo_pose` - to see the `Pose`.
  
  c- `/yolov8_processed_image` - To see `Yolov8` detection. 
