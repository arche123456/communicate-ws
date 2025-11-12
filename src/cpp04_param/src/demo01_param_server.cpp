/*
  需求：演示参数API使用
  流程：
    1.包含头文件
    2.初始化ROS客户端
    3.自定义节点类
      3.1增
      3.2查
      3.3改
      3.4删  
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
//1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;//使用时间命名空间，可以数字加单位
//3.自定义节点类
class ParamServer: public rclcpp::Node
{
  public:
    //如果允许删除参数，那么需要通过NodeOptions声明
    ParamServer():Node("param_server_node_cpp", rclcpp::NodeOptions().allow_undeclared_parameters(true))
    {
      RCLCPP_INFO(this->get_logger(),"参数服务端创建了");
    }
    // 3.1增
    void declare_param()
    {
        RCLCPP_INFO(this->get_logger(), "-----增-----");
        this->declare_parameter("car_name", "tiger");
        this->declare_parameter("width", 1.55);
        this->declare_parameter("wheels", 5);

        this->set_parameter(rclcpp::Parameter("height", 2.00));
    }
    // 3.2查
    void get_param()
    {
        RCLCPP_INFO(this->get_logger(), "-----查-----");
        //this->get_parameter()
        //this->get_parameters()
        //this->has_parameter()
        //获取指定参数
        auto car = this->get_parameter("car_name");
        RCLCPP_INFO(this->get_logger(), "key = %s, value = %s", car.get_name().c_str(), car.as_string().c_str());
        //获取一些参数
        auto params = this->get_parameters({"car_name", "width", "wheels"});
        for(auto &&param : params)
        {
          RCLCPP_INFO(this->get_logger(), "(%s = %s)", param.get_name().c_str(), param.value_to_string().c_str());
        }
        //判断是否包含
        RCLCPP_INFO(this->get_logger(), "是否包含car_name? %d", this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(), "是否包含car_name? %d", this->has_parameter("height"));
    }
    // 3.3改
    void update_param()
    {
        RCLCPP_INFO(this->get_logger(), "-----改-----");
        this->set_parameter(rclcpp::Parameter("width", 1.75));
        RCLCPP_INFO(this->get_logger(), "width = %.2f", this->get_parameter("width").as_double());
    }
    // 3.4删  
    void del_param()
    {
        RCLCPP_INFO(this->get_logger(), "-----删-----");
        //this->undeclare_parameter("car_name"); //不能删除声明的参数
        this->undeclare_parameter("height");//可以删除没有声明然后设置的参数
        RCLCPP_INFO(this->get_logger(), "删除后还包含height吗？ %d", this->has_parameter("height"));
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
  auto node = std::make_shared<ParamServer>();
  node->declare_param();
  node->get_param();
  node->update_param();
  node->del_param();
  rclcpp::spin(node);//持续处理传入节点的所有回调
  //5，释放资源
  rclcpp::shutdown();
  return 0;
}
