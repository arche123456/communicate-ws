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
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;//使用时间命名空间，可以数字加单位
//3.自定义节点类
class Exer01PubSub: public rclcpp::Node
{
  public:
    Exer01PubSub():Node("exer01_pub_sub_node_cpp")
    {
      RCLCPP_INFO(this->get_logger(),"案例1对象创建");
      //3-1创建消息发布方
      /*
        模板：被发布的消息类型
        参数：
          1.话题名称
          2.消息队列长度
        返回值：发布对象指针
      */
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);//创建了一个发布方，发布到话题chatter上，队列长度为10
      //3-2创建定时器
      /*
        参数：
          1.时间间隔
          2.回调函数
        返回值：定时器对象指针
      */
      timer_ = this->create_wall_timer(1s, std::bind(&Exer01PubSub::on_timer, this));
    }
  private:
    void on_timer()
    {
      //3-3组织并发布消息
      auto message = std_msgs::msg::String();
      message.data = "hello world! " + std::to_string(count++);
      RCLCPP_INFO(this->get_logger(), "发布方发布的消息：%s", message.data.c_str());//将std::string字符串转换为C风格字符串const char*
      publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;//创建了一个发布方指针用来接收发布的对象
    rclcpp::TimerBase::SharedPtr timer_;//创建定时器指针
    size_t count;
  };


int main(int argc, char ** argv)
{
  //2.初始化ROS客户端
  rclcpp::init(argc, argv);
  //4.调用spin函数，传入自定义类对象指针
  rclcpp::spin(std::make_shared<Exer01PubSub>());//持续处理传入节点的所有回调
  //5，释放资源
  rclcpp::shutdown();
  return 0;
}
