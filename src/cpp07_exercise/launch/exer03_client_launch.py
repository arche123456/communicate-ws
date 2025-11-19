
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
def generate_launch_description():
    x = 6.0
    y = 9.0
    theta = 0.0
    name = "t2"
    #1.在目标点生成一只新乌龟
    spawn = ExecuteProcess(
        cmd = ["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': " + str(x) + ",'y': " + str(y) + ",'theta': " + str(theta) +",'name': '"+ name + "'}\""],
        output = "both", #让执行指令的时候日志还可以输出到中断
        shell = True    #让cmd中列表的字符串作为shell命令来执行
        )
    #2.调用客户端发送目标点坐标
    client = Node(package = "cpp07_exercise", 
                  executable = "exer03_client",
                  arguments = [str(x), str(y), str(theta)])
                  #ros2 run cpp07_exercise exer03_client 7.0 9.0 0.0 --ros-args
    return LaunchDescription([spawn, client])