//1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//3.自定义节点类
class MyNode: public rclcpp::Node
{
  public:
    MyNode():Node("zhen_keng", "zuoxie") //节点名称，命名空间
    {
      //全局话题：命名空间，节点名称无关系
      //pub_ = this->create_publisher<std_msgs::msg::String>("/shi", 10);
      //相对话题：
      //pub_ = this->create_publisher<std_msgs::msg::String>("kaihui", 10);
      //私有话题：
      pub_ = this->create_publisher<std_msgs::msg::String>("~/vip", 10);
      RCLCPP_INFO(this->get_logger(),"mynode_node_cpp节点创建成功！");
    }
  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
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
