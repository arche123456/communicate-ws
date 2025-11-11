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
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Progress::Goal> goal)
    {
      (void)uuid;
      //业务逻辑：判断提交的数字是否大于1，是就接收，不是就拒绝
      if(goal->num <= 1)
      {
        RCLCPP_INFO(this->get_logger(), "提交的目标值必须大于1!");
        return rclcpp_action::GoalResponse::REJECT;//接到请求并立即执行
      }
      RCLCPP_INFO(this->get_logger(), "提交的目标值合法");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;//接到请求并立即执行
    }
    // 3-3处理取消请求（回调函数）
    //using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
      (void)goal_handle;
      RCLCPP_INFO(this->get_logger(), "接收到任务取消请求");
      return rclcpp_action::CancelResponse::ACCEPT;//接受取消请求
    }
    // 3-4生成连续反馈与最终相应（回调函数）
    //using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
      //1.生成连续反馈返回给客户端
      //void publish_feedback(std::shared_ptr<base_interface_demo::action::Progress_Feedback> feedback_msg)
      //goal_handle->publish_feedback();
      //首先要获取目标值，然后遍历，遍历中进行累加，且每循环一次就计算进度并作为连续反馈发布
      int num = goal_handle->get_goal()->num;
      int sum = 0;
      auto feedback = std::make_shared<Progress::Feedback>();
      auto result = std::make_shared<Progress::Result>();
      //设置休眠
      rclcpp::Rate rate(1.0);
      for(int i = 1; i <= num; i++)
      {
        sum += i;
        double progress = i / (double)num;//计算进度
        feedback->progress = progress;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "连续反馈中，进度:%.2f", progress);
        //判断是否接收到了取消请求
        //goal_handle->is_canceling()
        //void canceled(std::shared_ptr<base_interface_demo::action::Progress_Result> result_msg)
        //goal_handle->canceled()
        //std::make_shared<typename ActionT::Impl::GetResultService::Response>()
        // auto response = std::make_shared<Progress::Impl::GetResultService::Response>();
        // if(response->status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
        if(!goal_handle->is_active())
        {
          //如果接收到了，终止程序return
          result->sum = sum;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(),"任务被取消了");
          return;
        }
        //如果接收到了，终止程序，return
        rate.sleep();
      }
      //2.生成最终响应结果
      //void succeed(std::shared_ptr<base_interface_demo::action::Progress_Result> result_msg)
      //goal_handle->succeed()
      if(rclcpp::ok())
      {
        result->sum = sum;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "最终结果:%d", sum);
      }
    }
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
      //新建子线程处理主逻辑实现
      std::thread(std::bind(&ProgressActionServer::execute, this, goal_handle)).detach();
      (void)goal_handle;
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