/*
  需求：编写客户端实现，可以发送一个整形数据到服务端。并处理服务端的连续反馈和最终响应结果
  流程：
    前提：可以解析终端下动态传入的参数   
    1.包含头文件
    2.初始化ROS客户端
    3.自定义节点类
      3.1创建动作客户端
      3.2发送请求
      3.3处理关于目标值的服务端响应（回调函数）
      3.4处理连续反馈
      3.5处理最终响应
    4.调用spin函数，传入节点对象指针
    5.释放资源
*/
//1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interface_demo/action/progress.hpp"//包含服务的头文件

using base_interface_demo::action::Progress;//使用时间命名空间，可以数字加单位
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
//3.自定义节点类
class ProgressActionClient: public rclcpp::Node
{
  public:
    ProgressActionClient():Node("progress_action_node_cpp")
    {
      RCLCPP_INFO(this->get_logger(),"client客户端创建成功!");
      client_ = rclcpp_action::create_client<Progress>(this, "get_sum");
    }
    void send_goal(int num)
    {
      //1.需求连接服务端
      if(!client_->wait_for_action_server(10s))
      {
        RCLCPP_ERROR(this->get_logger(), "服务连接失败");
        return;
      }
      //2.发送具体请求
      /*std::shared_future<rclcpp_action::ClientGoalHandle<base_interface_demo::action::Progress>::SharedPtr>
       async_send_goal(
       const base_interface_demo::action::Progress::Goal &goal, 
       const rclcpp_action::Client<base_interface_demo::action::Progress>::SendGoalOptions &options)*/
      auto goal = Progress::Goal();
      goal.num = num;
      rclcpp_action::Client<Progress>::SendGoalOptions options;
      options.goal_response_callback = std::bind(&ProgressActionClient::goal_response_callback, this, _1);
      options.feedback_callback = std::bind(&ProgressActionClient::feedback_callback, this, _1, _2);
      options.result_callback = std::bind(&ProgressActionClient::result_callback, this, _1);
      auto future = client_->async_send_goal(goal, options);
    }
    //3.3处理关于目标值的服务端响应（回调函数）
    /*
      using GoalHandle = ClientGoalHandle<ActionT>;
      using GoalResponseCallback =
      std::function<void (std::shared_future<typename GoalHandle::SharedPtr>)>
      //rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle;
    */
    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<Progress>::SharedPtr> future)
    {
      auto goal_handle = future.get();
      if(!goal_handle)
      {
        RCLCPP_INFO(this->get_logger(),"目标请求被服务端拒绝");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(),"目标处理中");
      }
    }
    //3.4处理连续反馈（回调函数）
    /*
      std::function<void (
      typename ClientGoalHandle<ActionT>::SharedPtr,
      const std::shared_ptr<const Feedback>)>;
    */
    void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle, const std::shared_ptr<const Progress::Feedback> feedback)
    {
      (void)goal_handle;
      double progress = feedback->progress;//进度值（小数）
      int pro = (int)(progress * 100);
      RCLCPP_INFO(this->get_logger(), "当前进度:%d%%", pro);
    }
    //3.5处理最终响应（回调函数）
    /*
    std::function<void (const WrappedResult & result)>;
    */
    void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult & result)
    {

    }
  private:

    rclcpp_action::Client<Progress>::SharedPtr client_;
};


int main(int argc, char const *argv[])
{
  if(argc != 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交一个整形数据！");
    return 1;
  }
  //2.初始化ROS客户端
  rclcpp::init(argc, argv);
  //4.调用spin函数，传入自定义类对象指针
  auto node = std::make_shared<ProgressActionClient>();
  node->send_goal(atoi(argv[1]));
  rclcpp::spin(node);//持续处理传入节点的所有回调
  //5，释放资源
  rclcpp::shutdown();
  return 0;
}