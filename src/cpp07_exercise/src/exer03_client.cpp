/*
  需求：客户端需要提交目标点坐标，并解析响应结果
  流程：
    0.解析动态传入的数据，作为目标点坐标
    1.包含头文件
    2.初始化ROS客户端
    3.自定义节点类
      3.1构造函数创建客户端
      3.2客户端需要连接服务端
      3.3发送请求数据
    4.调用节点对象指针的相关函数
    5.释放资源
*/
//1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "base_interface_demo/srv/distance.hpp"

using base_interface_demo::srv::Distance;
using namespace std::chrono_literals; //使用时间命名空间，可以数字加单位
//3.自定义节点类
class Exer03Client: public rclcpp::Node
{
  public:
    Exer03Client():Node("exer03_client_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(),"案例3对象创建");
        // 3.1构造函数创建客户端
        client_ = this->create_client<Distance>("distance");//服务名称distance要和server端对应
    }
    // 3.2客户端需要连接服务端
    bool connect_server()
    {
        while(client_->wait_for_service(1s) == 0)
        {
            if(rclcpp::ok() == false)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"节点被强制退出");
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"正在等待服务端连接...");
        }
        return true;
    }
    // 3.3发送请求数据    
    rclcpp::Client<Distance>::SharedFuture send_goal(float x, float y, float theta)
    {
        //创建一个数据请求
        auto request = std::make_shared<Distance::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        return client_->async_send_request(request);
    }
  private:
        rclcpp::Client<Distance>::SharedPtr client_;
};


int main(int argc, char ** argv)
{
    //0.解析动态传入的数据，作为目标点坐标
    if(argc != 5)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"使用错误！请提交x坐标y坐标theta三个参数");
        return 1;
    }
    //解析提交的参数
    float goal_x = atof(argv[1]);
    float goal_y = atof(argv[2]);
    float goal_theta = atof(argv[3]);
    //2.初始化ROS客户端
    rclcpp::init(argc, argv);
    //4.调用节点对象指针的相关函数
    //创建节点对象指针
    auto client = std::make_shared<Exer03Client>();
    bool flag = client->connect_server();
    if(flag == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"连接服务端失败，程序退出");
        return 0;
    }
    //发送请求并处理相应
    //不会阻塞进程会立即返回一个future对象用于存放结果和状态
    auto future = client->send_goal(goal_x, goal_y, goal_theta);
    //判断响应结果状态
    if(rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(client->get_logger(),
            "请求结果：两只王八距离为%.2f米", future.get()->distance);
    }
    else
    {
        RCLCPP_ERROR(client->get_logger(),"请求服务失败！");
    }
    //5，释放资源
    rclcpp::shutdown();
    return 0;
}
