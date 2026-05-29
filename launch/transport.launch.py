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
    # foxglove_decoder_node = Node(
    #     name='image_foxglove_to_raw',
    #     namespace=namespace,
    #     package='image_transport',
    #     executable='republish',
    #     remappings=[
    #         # 1. Map the exact plugin string directly to your input argument
    #         ('in/foxglove', in_foxglove),
    #         ('out', out_raw),
    #     ],
    #     arguments=['foxglove', 'raw'],
    #     parameters=[{
    #         # 2. Because it is now successfully remapped, the QoS override MUST 
    #         # target the final resolved topic name (encoded_video)
    #         'qos_overrides./rover/poe/encoded_video.subscription.reliability': 'best_effort',
    #         'qos_overrides./rover/poe/encoded_video.subscription.durability': 'volatile',
    #         'qos_overrides./rover/poe/encoded_video.subscription.history': 'keep_last',
    #         'qos_overrides./rover/poe/encoded_video.subscription.depth': 1,
    #     }]
    # )

    # Custom image transport republish node for decoding
    foxglove_decoder_node = Node(
        name='image_foxglove_to_raw',
        namespace=namespace,
        package='umrt-ros-poe-cam',
        executable='foxglove_republisher_node',
        remappings=[
            # We still need this exact mapping because image_transport appends 
            # '/foxglove' to our base 'in' topic from the C++ code.
            ('in/foxglove', in_foxglove), 
            ('out', out_raw),
        ]
        # Notice: No QoS overrides or arguments needed here anymore!
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