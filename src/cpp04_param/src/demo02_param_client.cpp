/*
  需求：演示参数API使用
  流程：
    1.包含头文件
    2.初始化ROS客户端
    3.自定义节点类
      3.1参数对象的创建
      3.2参数对象的解析（获取键，值，将获取的值转换成字符串）
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
//1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;//使用时间命名空间，可以数字加单位
//3.自定义节点类
class ParamClient: public rclcpp::Node
{
  public:
    ParamClient():Node("param_client_node_cpp")
    {
      RCLCPP_INFO(this->get_logger(),"参数客户端创建了！");
      //3-1参数对象创建
      rclcpp::Parameter p1("car_name", "tiger");
      rclcpp::Parameter p2("height", 1.68);
      rclcpp::Parameter p3("wheels", 4);
      //3-2参数对象解析（获取键，值，将值转换成字符串）
      //解析值
      RCLCPP_INFO(this->get_logger(), "car_name = %s", p1.as_string().c_str());
      RCLCPP_INFO(this->get_logger(), "height = %.2f", p2.as_double());
      RCLCPP_INFO(this->get_logger(), "wheels = %ld", p3.as_int());

      //获取参数键
      RCLCPP_INFO(this->get_logger(), "name = %s", p1.get_name().c_str());
      RCLCPP_INFO(this->get_logger(), "type = %s", p1.get_type_name().c_str());
      RCLCPP_INFO(this->get_logger(), "value2string = %s", p2.value_to_string().c_str());

      //rclcpp::ParameterValue value(, );
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
      timer_ = this->create_wall_timer(1s, std::bind(&ParamClient::on_timer, this));
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
  rclcpp::spin(std::make_shared<ParamClient>());//持续处理传入节点的所有回调
  //5，释放资源
  rclcpp::shutdown();
  return 0;
}
