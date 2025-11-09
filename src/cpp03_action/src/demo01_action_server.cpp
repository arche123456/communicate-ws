/*
  需求：编写动作服务端，需要解析客户端提交的数字，
       遍历该数字并累加求和，最终结果响应回客户端，且请求响应过程中需要生成连续反馈
  分析：
    1.创建动作服务端对象
    2.处理提交的目标值
    3.生成连续反馈

  流程：
    1.包含头文件
    2.初始化ROS客户端
    3.自定义节点类
      3-1创建动作服务端对象
      3-2处理提交的目标值（回调函数）
      3-3处理取消请求（回调函数）
      3-4生成连续反馈与最终相应（回调函数）
   
    4.调用spin函数，传入自定义类对象指针
    5释放资源
*/
//1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interface_demo/action/progress.hpp"

using base_interface_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;
//3.自定义节点类
class ProgressActionServer: public rclcpp::Node
{
  public:
    ProgressActionServer():Node("progress_action_node_cpp")
    {
      RCLCPP_INFO(this->get_logger(),"action服务端创建成功!");
      // 3-1创建动作服务端对象
       server_ = rclcpp_action::create_server<Progress>(
        this,
        "get_sum",
        std::bind(&ProgressActionServer::handle_goal, this, _1, _2),
        std::bind(&ProgressActionServer::handle_cancel, this, _1),
        std::bind(&ProgressActionServer::handle_accepted, this, _1)
        );
    }
    // 3-2处理提交的目标值（回调函数）
    //using GoalCallback = std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Progress::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;//接到请求并立即执行
    }
    // 3-3处理取消请求（回调函数）
    //using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
      return rclcpp_action::CancelResponse::ACCEPT;//接受取消请求
    }
    // 3-4生成连续反馈与最终相应（回调函数）
    //using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {

    }
  private:
    rclcpp_action::Server<Progress>::SharedPtr server_;
};


int main(int argc, char ** argv)
{
  //2.初始化ROS客户端
  rclcpp::init(argc, argv);
  //4.调用spin函数，传入自定义类对象指针
  rclcpp::spin(std::make_shared<ProgressActionServer>());//持续处理传入节点的所有回调
  //5，释放资源
  rclcpp::shutdown();
  return 0;
}