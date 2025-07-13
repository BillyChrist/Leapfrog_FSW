from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='stm32_bridge',
            executable='stm32_bridge',
            #namespace='stm32_bridge',
            #output='screen',
            arguments=["/dev/stm32","115200"] ,
            #remappings=[
            #    ('/sensors/laser', 'laser'),
            #    ('/flightcontrol/heartbeat', 'heartbeat')
            #]
        ),
        launch_ros.actions.Node(
            package='flightcontrol',
            executable='flightmanager',
            #namespace='flightcontrol',
            #output='screen',
            arguments=["/dev/ttyAMA4","57600"]
        ),
    ])
