"""
UMRT Foxglove Compressed Video to Image Launch File
"""

"""
Imports
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

"""
Generate Launch Description
"""
def generate_launch_description():

    """
    Parameters
    """
    # Declare arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rover/poe',
        description = 'Namespace for camera.'
    )
    in_foxglove_arg = DeclareLaunchArgument(
        'in_foxglove',
        default_value='encoded_video',
        description = 'Input Foxglove Compressed Video topic.'
    )
    out_raw_arg = DeclareLaunchArgument(
        'out_raw',
        default_value='raw',
        description = 'Output raw image topic.'
    )

    # didnt really apply this
    # qos_profile = QoSProfile(
    #     reliability=ReliabilityPolicy.BEST_EFFORT, # Matches your C++ publisher
    #     history=HistoryPolicy.KEEP_LAST,
    #     depth=1, # Depth 1 as per your C++ publisher
    #     durability=DurabilityPolicy.VOLATILE # Matches your C++ publisher
    # )

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    in_foxglove = LaunchConfiguration('in_foxglove')
    out_raw = LaunchConfiguration('out_raw')

    """
    Nodes
    """
    # Image transport republish node for decoding
    foxglove_decoder_node = Node(
        name='image_foxglove_to_raw',
        namespace=namespace,
        package='image_transport',
        executable='republish',
        remappings=[
            ('in/foxglove', in_foxglove),
            ('out', out_raw),
        ],
        arguments=['foxglove', 'raw'],
        parameters=[{
        'qos_overrides./in/foxglove.subscription.reliability': 'best_effort',
        'qos_overrides./in/foxglove.subscription.durability': 'volatile',
        'qos_overrides./in/foxglove.subscription.history': 'keep_last',
        'qos_overrides./in/foxglove.subscription.depth': 1,
        }]
    )

    """
    Launch
    """
    decode = [
        namespace_arg,
        in_foxglove_arg,
        out_raw_arg,
        foxglove_decoder_node
    ]

    launch_entities = decode 

    return LaunchDescription(launch_entities)