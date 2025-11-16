//1.包含头文件
#include "rclcpp/rclcpp.hpp"

//3.自定义节点类
class MyNode: public rclcpp::Node
{
  public:
    MyNode():Node("mynode_node_cpp", "t1_ns")
    {
      RCLCPP_INFO(this->get_logger(),"mynode_node_cpp节点创建成功！");

    }
  private:
};


int main(int argc, char ** argv)
{
  //2.初始化ROS客户端
  rclcpp::init(argc, argv);
  //4.调用spin函数，传入自定义类对象指针
  rclcpp::spin(std::make_shared<MyNode>());//持续处理传入节点的所有回调
  //5，释放资源
  rclcpp::shutdown();
  return 0;
}
