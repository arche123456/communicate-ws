/*
  需求：修改turtlesim_node的背景颜色
  流程：
    1.包含头文件
    2.初始化ROS客户端
    3.自定义节点类
      3.1创建参数客户端
      3.2连接参数服务端
      3.3更新参数
    4.创建节点对象指针，并调用其函数
    5.释放资源
*/
//1.包含头文件
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;//使用时间命名空间，可以数字加单位
//3.自定义节点类
class Exer06Param: public rclcpp::Node
{
  public:
    Exer06Param():Node("exer06_param_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(),"参数客户端");
        //   3.1创建参数客户端
        client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/turtlesim");
    }
    //   3.2连接参数服务端
    bool connect_server()
    {
        while(client_->wait_for_service(1s) == 0)
        {
            if(rclcpp::ok() == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "客户端强制退出");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "服务连接中");
        }
        return true;
    }
    //   3.3更新参数 
    void update_param()
    {
        //背景色递进修改
        //1.获取参数
        int red = client_->get_parameter<int>("background_r");
        //2.编写循环，修改参数（通过休眠控制更改频率）
        rclcpp::Rate rate(30.0);//hz
        /*
            需求：背景色渐变，由浅变深，再由深变浅
            实现：
                1.一个完整周期，计数是511个，中间值是255；
                2.创建一个计数器，初始值和red相同，递增的步进值也和red相同，取值是[0，511]
                3.当计数器再[0,255]之间时，red递增，如果在[256, 511]时，red递增
                4.当计数器大于等于511时，让他归零
        */
       int count = red;
        while(rclcpp::ok())
        {
            if(count <= 255)
            {
                red += 5;
            }
            else
            {
                red -= 5;
            }
            count += 5;
            if(count > 511)
            {
                count = 0;
            }
            //修改服务端参数
            client_->set_parameters({rclcpp::Parameter("background_r", red)});
            rate.sleep();
        }
    }   
  private:
    rclcpp::SyncParametersClient::SharedPtr client_;
  };


int main(int argc, char ** argv)
{
  //2.初始化ROS客户端
  rclcpp::init(argc, argv);
  //4.调用spin函数，传入自定义类对象指针
  auto client = std::make_shared<Exer06Param>();
  if(client->connect_server() == 0)
  {
    return 1;
  }
  //调用其函数
  client->update_param();
  //5，释放资源
  rclcpp::shutdown();
  return 0;
}
