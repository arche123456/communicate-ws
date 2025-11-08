/*
  需求：编写服务端实现，解析提交的请求数据，将解析的数据相加并相应到客户端
  流程：
    1.包含头文件
    2.初始化ROS客户端
    3.自定义节点类
      3.1创建服务端
      3.2回调函数解析请求并发送响应
    4.调用spin函数，传入节点对象指针
    5.释放资源
*/
//1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "base_interface_demo/srv/add_ints.hpp"//包含服务的头文件

using base_interface_demo::srv::AddInts;//使用时间命名空间，可以数字加单位
using std::placeholders::_1;
using std::placeholders::_2;
//3.自定义节点类
class AddIntsServer: public rclcpp::Node
{
  public:
    AddIntsServer():Node("add_ints_serve_node_cpp")
    {
      service_ = this->create_service<AddInts>("add_Ints", std::bind(&AddIntsServer::add, this, _1, _2));
      RCLCPP_INFO(this->get_logger(),"服务端节点创建成功！");
    }

  private:
    //request客户端请求数据，response服务端响应数据
    void add(const AddInts::Request::SharedPtr req, const AddInts::Response::SharedPtr res)
    {
      //3.2回调数函数解析请求并发送响应
      res->sum = req->num1 + req->num2;
      RCLCPP_INFO(this->get_logger(),"请求数据:num1+num2=sum->%d+%d=%d", req->num1, req->num2, res->sum);
    }
    rclcpp::Service<AddInts>::SharedPtr service_;
};


int main(int argc, char ** argv)
{
  //2.初始化ROS客户端
  rclcpp::init(argc, argv);
  //4.调用spin函数，传入自定义类对象指针
  rclcpp::spin(std::make_shared<AddIntsServer>());//持续处理传入节点的所有回调
  //5，释放资源
  rclcpp::shutdown();
  return 0;
}
