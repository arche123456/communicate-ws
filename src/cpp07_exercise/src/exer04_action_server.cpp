/*
  需求：处理客户端发送的请求数据（目标点），控制小乌龟向目标点运动，且连续反馈剩余距离
  流程：
    1.包含头文件
    2.初始化ROS客户端
    3.自定义节点类
      3.1创建一个订阅方，订阅原生乌龟位姿信息
      3.2创建一个速度指令发布方，控制乌龟运动
      3.3组织并发布消息
      3.4解析动作客户端提交的数据
      3.5处理客户端取消请求操作
      3.6实现主逻辑（耗时操作），启动子线程
      3.7子线程中发布速度指令，产生连续反馈，并响应最终结果
    4.调用spin函数，传入自定义类对象指针
    5.释放资源
*/
//1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interface_demo/action/nav.hpp"

using base_interface_demo::action::Nav;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;//使用时间命名空间，可以数字加单位
//3.自定义节点类
class Exer04ActionServer: public rclcpp::Node
{
  public:
    Exer04ActionServer():Node("exer04_action_server_node_cpp"), x(0.0), y(0.0)
    {
        RCLCPP_INFO(this->get_logger(),"动作服务端");
        //   3.1创建一个订阅方，订阅原生乌龟位姿信息
        sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10,
            std::bind(&Exer04ActionServer::pose_cb, this, _1));
        //   3.2创建一个速度指令发布方，控制乌龟运动
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        //   3.3组织并发布消息
        action_server_ = rclcpp_action::create_server<Nav>(this, 
            "nav",
            std::bind(&Exer04ActionServer::handle_goal, this, _1, _2),
            std::bind(&Exer04ActionServer::handle_cancel, this, _1),
            std::bind(&Exer04ActionServer::handle_accepted, this, _1)
        );
        //   3.4解析动作客户端提交的数据
        //   3.5处理客户端取消请求操作
        //   3.6实现主逻辑（耗时操作），启动子线程
        //   3.7子线程中发布速度指令，产生连续反馈，并响应最终结果
    }
  private:
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp_action::Server<Nav>::SharedPtr action_server_;
        float x, y;
        //获取原生乌龟位姿信息回调函数
        void pose_cb(turtlesim::msg::Pose::SharedPtr pose)
        {
            x = pose->x;
            y = pose->y;
        } 
        //请求目标处理
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Nav::Goal> goal)
        {
            (void)uuid;
            //取出目标中的x y坐标，分别判断是否超出了[0, 11.08]，如果超出则非法
            if(goal->goal_x < 0.0 || goal->goal_x > 11.08 || goal->goal_y < 0.0 || goal->goal_y > 11.08)
            {
                RCLCPP_INFO(this->get_logger(), "接收到的目标点(%.2f, %.2f)非法，拒绝请求", goal->goal_x, goal->goal_y);
                return rclcpp_action::GoalResponse::REJECT;
            }
            RCLCPP_INFO(this->get_logger(), "目标点合法");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        //取消请求处理
        rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(), "接受取消请求");
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle)
        {
            //子线程处理主要逻辑
            RCLCPP_INFO(this->get_logger(), "主逻辑开始执行");
            //最终结果类对象
            auto result = std::make_shared<Nav::Result>();
            //动作通信反馈对象
            auto feedback = std::make_shared<Nav::Feedback>();
            //ROS自带的标准消息类，用来描述物体的运动速度
            geometry_msgs::msg::Twist twist;
            //1.生成连续反馈
            rclcpp::Rate rate(1.0);
            while(true)
            {
                //如果要取消任务，那么需要特殊处理
                if(goal_handle->is_canceling())
                {
                    goal_handle->canceled(result);
                    return;
                }
                //解析目标点坐标与原生乌龟实时坐标并计算当前距离并发布
                float goal_x = goal_handle->get_goal()->goal_x;
                float goal_y = goal_handle->get_goal()->goal_y;
                float distance_x = goal_x - x;
                float distance_y = goal_y - y;
                float distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
                feedback->distance = distance;
                goal_handle->publish_feedback(feedback);
                //2.发布乌龟运动指令
                //根据剩余距离计算速度指令并发布
                float scale = 0.5;
                float linear_x = scale * distance_x;
                float linear_y = scale * distance_y;
                twist.linear.x = linear_x;
                twist.linear.y = linear_y;
                cmd_pub_->publish(twist);
                //循环结束条件
                if(distance <= 0.05)
                {
                    //与目标点的甚于距离小于0.05米，那么结束导航
                    RCLCPP_INFO(this->get_logger(), "到达目标点(%.2f, %.2f)", goal_x, goal_y);
                    break;
                }
                rate.sleep();
            }
            //3.生成最终响应结果
            if(rclcpp::ok())
            {
                result->turtle_x = x;
                result->turtle_y = y;
                goal_handle->succeed(result);
            }
        }        
        //主逻辑处理
        void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle)
        {
            std::thread(std::bind(&Exer04ActionServer::execute, this, goal_handle)).detach();
        }
};  


int main(int argc, char ** argv)
{
  //2.初始化ROS客户端
  rclcpp::init(argc, argv);
  //4.调用spin函数，传入自定义类对象指针
  rclcpp::spin(std::make_shared<Exer04ActionServer>());//持续处理传入节点的所有回调
  //5，释放资源
  rclcpp::shutdown();
  return 0;
}
