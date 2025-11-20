from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    x = 8.54
    y = 9.54
    theta = 0.0
    name = "t2"
    #1. 在目标点生成一只新乌龟
    spawn = ExecuteProcess(
        cmd = ["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': "
               +str(x)+", 'y': "
               +str(y)+", 'theta': "
               +str(theta)+", 'name': '"+name+"'}\""],
        output = "both",
        shell = True
    )
    #2.调用客户端发送目标点坐标
    client = Node(package = "cpp07_exercise", 
                  executable = "exer05_action_client",
                  arguments = [str(x), str(y), str(theta)])

    return LaunchDescription([spawn, client])