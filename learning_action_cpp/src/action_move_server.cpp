/***
@作者: 古月居(www.guyuehome.com)
@说明: ROS2动作示例-负责执行圆周运动动作的服务端
***/

#include <iostream>

#include "rclcpp/rclcpp.hpp"                          // ROS2 C++接口库
#include "rclcpp_action/rclcpp_action.hpp"            // ROS2 动作类
#include "learning_interface/action/move_circle.hpp"  // 自定义的圆周运动接口

using namespace std;

class MoveCircleActionServer : public rclcpp::Node
{
    public:
        // 定义一个自定义的动作接口类，便于后续使用
        using CustomAction = learning_interface::action::MoveCircle;
        // 定义一个处理动作请求、取消、执行的服务器端
        using GoalHandle = rclcpp_action::ServerGoalHandle<CustomAction>;

        explicit MoveCircleActionServer(const rclcpp::NodeOptions & action_server_options = rclcpp::NodeOptions())
        : Node("action_move_server", action_server_options)                                     // ROS2节点父类初始化
        {
            using namespace std::placeholders;

            this->action_server_ = rclcpp_action::create_server<CustomAction>(                  // 创建动作服务器（接口类型、动作名、回调函数）
                this->get_node_base_interface(),
                this->get_node_clock_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                "move_circle",
                std::bind(&MoveCircleActionServer::handle_goal, this, _1, _2),
                std::bind(&MoveCircleActionServer::handle_cancel, this, _1),
                std::bind(&MoveCircleActionServer::handle_accepted, this, _1));
        }

    private:
        rclcpp_action::Server<CustomAction>::SharedPtr action_server_;  // 动作服务器

        // 响应动作目标的请求
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const CustomAction::Goal> goal_request)                             
        {
            RCLCPP_INFO(this->get_logger(), "Server: Received goal request: %d", goal_request->enable);
            (void)uuid;

            // 如请求为enable则接受运动请求，否则就拒绝
            if (goal_request->enable)                                                           
            {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }
            else
            {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // 响应动作取消的请求
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle_canceled_)                            
        {
            RCLCPP_INFO(this->get_logger(), "Server: Received request to cancel action");
            (void) goal_handle_canceled_; 
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // 处理动作接受后具体执行的过程
        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle_accepted_)          
        {
            using namespace std::placeholders;
            // 在线程中执行动作过程
            std::thread{std::bind(&MoveCircleActionServer::execute, this, _1), goal_handle_accepted_}.detach(); 
        }


        void execute(const std::shared_ptr<GoalHandle> goal_handle_)
        {
            const auto requested_goal = goal_handle_->get_goal();       // 动作目标
            auto feedback = std::make_shared<CustomAction::Feedback>(); // 动作反馈
            auto result = std::make_shared<CustomAction::Result>();     // 动作结果

            RCLCPP_INFO(this->get_logger(), "Server: Executing goal");
            rclcpp::Rate loop_rate(1);

            // 动作执行的过程
            for (int i = 0; (i < 361) && rclcpp::ok(); i=i+30)
            {
                // 检查是否取消动作
                if (goal_handle_->is_canceling())
                {
                    result->finish = false;
                    goal_handle_->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Server: Goal canceled");
                    return;
                }

                // 更新反馈状态
                feedback->state = i;
                // 发布反馈状态
                goal_handle_->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Server: Publish feedback");

                loop_rate.sleep();
            }

            // 动作执行完成
            if (rclcpp::ok())
            {
                result->finish = true;
                goal_handle_->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Server: Goal succeeded");
            }
        }
};

// ROS2节点主入口main函数
int main(int argc, char * argv[])                                
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);                        
    
    // 创建ROS2节点对象并进行初始化             
    rclcpp::spin(std::make_shared<MoveCircleActionServer>());   
    
    // 关闭ROS2 C++接口  
    rclcpp::shutdown();                                           
    
    return 0;
}

