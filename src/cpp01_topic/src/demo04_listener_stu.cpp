/*
  需求：订阅学生需求并打印在终端
  流程：
    1.包含头文件
    2.初始化ROS客户端
    3.自定义节点类
      3.1创建消息订阅方
      3.2回调函数订阅并解析数据
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
//1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "base_interface_demo/msg/student.hpp"
using base_interface_demo::msg::Student;
class ListenerStu: public rclcpp::Node{
    public:
        ListenerStu():Node("listenerstu_node_cpp"){
          // 3-1创建订阅方
          //模板为订阅消息类型，参数1为话题名称，参数2为队列长度，参数3为当前类下的回调函数，参数4为调用对象为当前对象，参数5为回调函数参数为所占字节
          subscription_ = this->create_subscription<Student>("chatter_stu", 10, std::bind(&ListenerStu::do_back, this, std::placeholders::_1));
          RCLCPP_INFO(this->get_logger(),"listenerstu_node_cpp节点创建成功");
        }
    private:
        void do_back(const Student::SharedPtr stu){
          //3-2回调函数订阅并解析数据
          RCLCPP_INFO(this->get_logger(), "订阅的学生信息:name=%s, age=%d, height=%.2f", stu->name.c_str(), stu->age, stu->height);

        }
        rclcpp::Subscription<Student>::SharedPtr subscription_;
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
