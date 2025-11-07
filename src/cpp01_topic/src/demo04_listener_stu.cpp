/*
  需求：以某个固定频率发送文本“hello world！”，文本后缀编号，每发布一条，编号+1
  流程：
    1.包含头文件
    2.初始化ROS客户端
    3.自定义节点类
      3.1创建消息发布方
      3.2创建定时器
      3.2组织并发布消息
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
//1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "base_interface_demo/msg/student.hpp"

class ListenerStu: public rclcpp::Node{
    public:
        ListenerStu():Node("listenerstu_node_cpp"){
            RCLCPP_INFO(this->get_logger(),"listenerstu_node_cpp节点创建成功");
        }
};

int main(int argc, char ** argv)
{
  //2.初始化ROS客户端
  rclcpp::init(argc, argv);
  //4.调用spin函数并传入节点对象指针
  rclcpp::spin(std::make_shared<ListenerStu>());
  //5.释放资源
  rclcpp::shutdown();
  return 0;
}
